// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the main logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_frame.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_var.h"
#include "util/u_threading.h"
#include "util/u_weak_ptr.hpp"

#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include "correspondence_search.h"
#include "led_search_model.h"
#include "pose_optimize.h"
#include "pose_metrics.h"
#include "t_constellation_tracker.h"

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <stdexcept>
#include <array>


namespace os = xrt::auxiliary::os;

DEBUG_GET_ONCE_LOG_OPTION(constellation_tracker_log, "CONSTELLATION_TRACKER_LOG", U_LOGGING_WARN)

#define CT_TRACE(ct, ...) U_LOG_IFL_T(ct->log_level, __VA_ARGS__)
#define CT_DEBUG(ct, ...) U_LOG_IFL_D(ct->log_level, __VA_ARGS__)
#define CT_INFO(ct, ...) U_LOG_IFL_I(ct->log_level, __VA_ARGS__)
#define CT_WARN(ct, ...) U_LOG_IFL_W(ct->log_level, __VA_ARGS__)
#define CT_ERROR(ct, ...) U_LOG_IFL_E(ct->log_level, __VA_ARGS__)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

extern "C" void
constellation_tracker_camera_push_blobs(struct t_blob_sink *tbs, struct t_blob_observation *tbo);

extern "C" void
constellation_tracker_camera_destroy(struct t_blob_sink *tbs);

extern "C" void *
constellation_tracker_camera_slow_thread(void *ptr);

extern "C" void *
constellation_tracker_camera_fast_thread(void *ptr);

extern "C" void
constellation_tracker_node_break_apart(struct xrt_frame_node *node);

extern "C" void
constellation_tracker_node_destroy(struct xrt_frame_node *node);

namespace {

struct CameraSample
{
	bool has_sample;

	struct t_blob blob_storage[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];

	struct t_blob_observation blobservation;

	std::array<t_constellation_device_id_t, XRT_CONSTELLATION_MAX_DEVICES> needs_slow_processing;
	uint32_t num_devices_needing_slow_processing;

	void
	Take(std::optional<CameraSample> &maybe_sample)
	{
		if (!this->has_sample) {
			maybe_sample = std::nullopt;
			return;
		}

		maybe_sample = *this;
		// Make sure the memory is pointing to the correct place
		maybe_sample->blobservation.blobs = maybe_sample->blob_storage;

		this->has_sample = false;
		this->num_devices_needing_slow_processing = 0;
	}

	void
	Push(t_blob_observation &blobservation,
	     std::array<t_constellation_device_id_t, XRT_CONSTELLATION_MAX_DEVICES> &devices_needing_slow_processing,
	     uint32_t num_devices_needing_slow_processing)
	{
		this->blobservation = blobservation;

		this->needs_slow_processing = devices_needing_slow_processing;
		this->num_devices_needing_slow_processing = num_devices_needing_slow_processing;

		// Copy the blobs into safe memory
		memcpy(blob_storage, blobservation.blobs, sizeof(t_blob) * blobservation.num_blobs);
		this->blobservation.blobs = blob_storage;

		this->has_sample = true;
	}

	void
	Push(t_blob_observation &blobservation)
	{
		this->blobservation = blobservation;
		this->num_devices_needing_slow_processing = 0;

		// Copy the blobs into safe memory
		memcpy(blob_storage, blobservation.blobs, sizeof(t_blob) * blobservation.num_blobs);
		this->blobservation.blobs = blob_storage;

		this->has_sample = true;
	}
};

class CameraMosaic;

class Camera
{
public:
	struct t_blob_sink base = {
	    .push_blobs = constellation_tracker_camera_push_blobs,
	    .destroy = constellation_tracker_camera_destroy,
	};

	//! The owner tracker, so we can retrieve it from the blob sink callback
	class ConstellationTracker *tracker;
	std::weak_ptr<CameraMosaic> mosaic;

	struct t_camera_calibration calibration;

	struct camera_model model;

	//! Does "slow" processing for this camera when fast recovery paths fail.
	struct os_thread_helper slow_processing_thread;
	struct
	{
		struct CameraSample sample;

		struct correspondence_search *cs;
	} slow_processing_thread_data;

	/*!
	 * Does "fast" processing for this camera, trying to recover a pose quickly. It's valid for this to happen at
	 * the same time as a slow process.
	 */
	struct os_thread_helper fast_processing_thread;
	struct
	{
		struct CameraSample sample;

		struct correspondence_search *cs;
	} fast_processing_thread_data;

	//! Locks all processing data
	mutable os::Mutex processing_lock;
	//! All data protected by the processing lock
	struct
	{
		struct xrt_pose Txr_origin_cam;
		bool has_concrete_pose;
	} locked_data;

	static Camera *
	Get(struct t_blob_sink *tbs)
	{
		return container_of(tbs, Camera, base);
	}

	Camera(ConstellationTracker *tracker,
	       std::weak_ptr<CameraMosaic> mosaic,
	       const struct t_constellation_tracker_camera &camera_params,
	       enum u_logging_level *log_level_ptr)
	    : tracker(tracker), mosaic(mosaic), calibration(camera_params.calibration)
	{
		this->tracker = tracker;
		this->mosaic = mosaic;
		this->calibration = camera_params.calibration;
		this->model = {
		    .width = calibration.image_size_pixels.w,
		    .height = calibration.image_size_pixels.h,
		    .calib = {},
		};
		t_camera_model_params_from_t_camera_calibration(&this->calibration, &this->model.calib);

		this->locked_data = {
		    .Txr_origin_cam = camera_params.pose_in_origin,
		    .has_concrete_pose = camera_params.has_concrete_pose,
		};

		this->slow_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);
		this->fast_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);

		if (os_thread_helper_init(&this->slow_processing_thread) < 0) {
			throw std::runtime_error("Slow processing thread failed to init");
		}
		if (os_thread_helper_init(&this->fast_processing_thread) < 0) {
			throw std::runtime_error("Fast processing thread failed to init");
		}

		if (os_thread_helper_start(&this->slow_processing_thread, constellation_tracker_camera_slow_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting slow processing thread failed");
		}
		if (os_thread_helper_start(&this->fast_processing_thread, constellation_tracker_camera_fast_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting fast processing thread failed");
		}
	}

	~Camera()
	{
		if (this->slow_processing_thread.initialized) {
			os_thread_helper_destroy(&this->slow_processing_thread);
		}

		if (this->fast_processing_thread.initialized) {
			os_thread_helper_destroy(&this->fast_processing_thread);
		}

		if (this->slow_processing_thread_data.cs) {
			correspondence_search_free(this->slow_processing_thread_data.cs);
			this->slow_processing_thread_data.cs = nullptr;
		}

		if (this->fast_processing_thread_data.cs) {
			correspondence_search_free(this->fast_processing_thread_data.cs);
			this->fast_processing_thread_data.cs = nullptr;
		}
	}
};

class CameraMosaic
{
public:
	std::vector<std::unique_ptr<Camera>> cameras;

	struct t_constellation_tracker_tracking_source *tracking_origin;

	CameraMosaic(ConstellationTracker *tracker, const struct t_constellation_tracker_camera_mosaic &mosaic_params)
	{
		this->tracking_origin = mosaic_params.tracking_origin;

		// NOTE: the stability of this vector is important since we're passing to C callbacks and APIs!
		this->cameras.reserve(mosaic_params.num_cameras);
	}

	~CameraMosaic() = default;
};

class Device
{
public:
	struct t_constellation_tracker_device_params params;
	struct t_constellation_tracker_device *device;

	t_constellation_device_id_t id;

	// @todo remove when clang-format is updated in CI
	// clang-format off
	struct t_constellation_search_model *search_model{nullptr};
	struct xrt_vec3 prior_pos_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	struct xrt_vec3 prior_rot_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	float gravity_error_rad{MIN_ROT_ERROR}; /* Gravity vector uncertainty in radians 0..M_PI */

	mutable os::Mutex data_lock;
	struct
	{
		bool has_last_known;
		struct xrt_pose Txr_world_device_last_known;
	} locked_data;
	// clang-format on

	Device(struct t_constellation_tracker_device_params *params,
	       struct t_constellation_tracker_device *device,
	       t_constellation_device_id_t id)
	    : params(*params), device(device), id(id), data_lock(),
	      locked_data({.has_last_known = false, .Txr_world_device_last_known = {}})
	{
		// Copy the LED model leds into safe memory, since we want to mutate it into OpenCV space
		this->params.led_model.leds = new t_constellation_tracker_led[this->params.led_model.led_count];
		memcpy(this->params.led_model.leds, params->led_model.leds,
		       sizeof(t_constellation_tracker_led) * this->params.led_model.led_count);

		// flip all LEDs from OpenXR -> OpenCV coordinate space, since the tracker works in OpenCV space
		for (size_t i = 0; i < this->params.led_model.led_count; i++) {
			struct t_constellation_tracker_led &dst = this->params.led_model.leds[i];
			struct t_constellation_tracker_led &src = params->led_model.leds[i];

			dst = src;
			dst.position.y = -dst.position.y;
			dst.position.z = -dst.position.z;
			dst.normal.y = -dst.normal.y;
			dst.normal.z = -dst.normal.z;
		}

		this->search_model = t_constellation_search_model_new(this->id, &this->params.led_model);
	}

	~Device()
	{
		if (this->search_model) {
			t_constellation_search_model_free(this->search_model);
			this->search_model = nullptr;
		}

		if (this->params.led_model.leds) {
			delete[] this->params.led_model.leds;
			this->params.led_model.leds = nullptr;
		}
	}
};

class ConstellationTracker
{
public:
	struct xrt_frame_node node = {
	    .next = nullptr,
	    .break_apart = constellation_tracker_node_break_apart,
	    .destroy = constellation_tracker_node_destroy,
	};

	//! Whether the constellation tracker is running
	bool running = true;

	enum u_logging_level log_level = U_LOGGING_WARN;

	struct t_constellation_tracker_params params;

	std::vector<std::shared_ptr<CameraMosaic>> mosaics;

	std::shared_mutex device_lock;
	std::vector<std::unique_ptr<Device>> devices;
	t_constellation_device_id_t next_device_id{0};

	ConstellationTracker(struct t_constellation_tracker_params *params)
	{
		this->log_level = debug_get_log_option_constellation_tracker_log();

		this->mosaics.reserve(params->num_mosaics);

		// Fill in our internal data structures based on the provided params
		for (size_t i = 0; i < params->num_mosaics; i++) {
			const struct t_constellation_tracker_camera_mosaic &mosaic_params = params->mosaics[i];

			std::shared_ptr<CameraMosaic> mosaic = std::make_shared<CameraMosaic>(this, mosaic_params);

			// Assert pointer stability!
			assert(mosaic->cameras.capacity() >= mosaic_params.num_cameras);

			for (size_t i = 0; i < mosaic_params.num_cameras; i++) {
				const struct t_constellation_tracker_camera &camera_params = mosaic_params.cameras[i];

				// This can't be in the constructor since the `shared_ptr` of the `mosaic` isn't formed
				// yet, but the camera needs to own a weak_ptr to it's mosaic.
				mosaic->cameras.push_back(
				    std::make_unique<Camera>(this, mosaic, camera_params, &this->log_level));
			}

			this->mosaics.push_back(mosaic);
		}

		// Fill in the blob sinks for each camera
		for (size_t i = 0; i < this->mosaics.size(); i++) {
			std::shared_ptr<CameraMosaic> &mosaic = this->mosaics[i];

			for (size_t j = 0; j < mosaic->cameras.size(); j++) {
				Camera *camera = mosaic->cameras[j].get();

				params->mosaics[i].cameras[j].blob_sink = &camera->base;
			}
		}

		this->params = *params;

		CT_DEBUG(this, "Created constellation tracker with %zu mosaics", this->mosaics.size());
	}

	~ConstellationTracker()
	{
		CT_DEBUG(this, "Destroying constellation tracker");
	}

	t_constellation_device_id_t
	AddDevice(struct t_constellation_tracker_device_params *params, struct t_constellation_tracker_device *device)
	{
		std::unique_lock lock(this->device_lock);

		if (this->devices.size() >= XRT_CONSTELLATION_MAX_DEVICES) {
			throw std::runtime_error("Maximum number of devices already added to constellation tracker");
		}

		t_constellation_device_id_t id = this->next_device_id++;

		this->devices.push_back(std::make_unique<Device>(params, device, id));

		CT_DEBUG(this, "Added device with ID %d to constellation tracker", id);

		return id;
	}

	void
	RemoveDevice(t_constellation_device_id_t device_id)
	{
		std::unique_lock lock(this->device_lock);

		size_t index = 0;
		for (auto &device : this->devices) {
			if (device->id == device_id) {
				break;
			}
			index++;
		}

		if (index == this->devices.size()) {
			throw std::invalid_argument("The device ID is not present in the device list.");
		}

		// Remove the device
		this->devices.erase(this->devices.begin() + index);
	}

	static ConstellationTracker *
	Get(struct xrt_frame_node *node)
	{
		return container_of(node, ConstellationTracker, node);
	}
};

}; // namespace

static void
mark_matching_blobs(ConstellationTracker *ct,
                    struct t_blob_observation *bwobs,
                    struct t_constellation_tracker_led_model *led_model,
                    t_constellation_device_id_t device_id,
                    struct pose_metrics_blob_match_info *blob_match_info)
{
	/* First clear existing blob labels for this device */
	for (uint32_t i = 0; i < bwobs->num_blobs; i++) {
		struct t_blob *b = bwobs->blobs + i;
		t_constellation_device_id_t led_object_id = b->matched_device_id;

		// Skip blobs which already have an ID not belonging to this device
		if (led_object_id != device_id) {
			continue;
		}

		if (b->matched_device_led_id != XRT_CONSTELLATION_INVALID_LED_ID) {
			// @todo is this needed?
			// b->prev_led_id = b->led_id;
		}

		b->matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
		b->matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
	}

	/* Iterate the visible LEDs and mark matching blobs with this device ID and LED ID */
	for (int i = 0; i < blob_match_info->num_visible_leds; i++) {
		struct pose_metrics_visible_led_info *led_info = blob_match_info->visible_leds + i;
		struct t_constellation_tracker_led *led = led_info->led;

		if (led_info->matched_blob != NULL) {
			struct t_blob *b = led_info->matched_blob;

			b->matched_device_led_id = led->id;
			b->matched_device_id = device_id;

			CT_DEBUG(ct, "Marking LED %d/%d at %f,%f angle %f now %d (was %d)", device_id, led->id,
			         b->center.x, b->center.y, RAD_TO_DEG(acosf(led_info->facing_dot)),
			         b->matched_device_led_id, /* b->prev_led_id */ -1);
		} else {
			CT_DEBUG(ct, "No blob for device %d LED %d @ %f,%f size %f px angle %f", device_id, led->id,
			         led_info->pos_px.x, led_info->pos_px.y, 2 * led_info->led_radius_px,
			         RAD_TO_DEG(acosf(led_info->facing_dot)));
		}
	}
}

static bool
get_tracking_origin_pose(CameraMosaic &mosaic, timepoint_ns when_ns, struct xrt_pose &Txr_world_origin)
{
	Txr_world_origin = XRT_POSE_IDENTITY;
	if (mosaic.tracking_origin) {
		xrt_space_relation relation;
		t_constellation_tracker_tracking_source_get_tracked_pose(mosaic.tracking_origin, when_ns, &relation);

		// If the tracking source has a valid position, grab it
		if ((relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
			Txr_world_origin = relation.pose;
		} else {
			return false;
		}
	}

	return true;
}

static void
get_pose_gravity_vector(struct xrt_pose &pose, struct xrt_vec3 &gravity)
{
	// Extract the gravity vector from the pose's orientation
	gravity.x = 0.0f;
	gravity.y = 1.0f;
	gravity.z = 0.0f;
	math_quat_rotate_vec3(&pose.orientation, &gravity, &gravity);
}

static void
push_pose(Camera *camera,
          std::unique_ptr<Device> &device,
          struct pose_metrics &score,
          struct xrt_pose &Tcv_cam_device,
          t_blob_observation &tbo,
          bool optimize = true)
{
	ConstellationTracker *tracker = camera->tracker;

	// Try to optimize the pose
	if (optimize) {
		int num_leds_out;
		int num_inliers;
		if (!ransac_pnp_pose(tracker->log_level, &Tcv_cam_device, tbo.blobs, tbo.num_blobs,
		                     &device->params.led_model, device->id, &camera->model, &num_leds_out,
		                     &num_inliers)) {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs failed",
			         0, 0, device->id, tbo.num_blobs);
		} else {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs had "
			         "%d LEDs with %d inliers. Produced pose %f,%f,%f,%f pos %f,%f,%f",
			         0, 0, device->id, tbo.num_blobs, num_leds_out, num_inliers,
			         Tcv_cam_device.orientation.x, Tcv_cam_device.orientation.y,
			         Tcv_cam_device.orientation.z, Tcv_cam_device.orientation.w, Tcv_cam_device.position.x,
			         Tcv_cam_device.position.y, Tcv_cam_device.position.z);
		}
	}

	// Move to OpenXR space
	struct xrt_pose Txr_cam_device;
	math_pose_convert_opencv(&Tcv_cam_device, &Txr_cam_device);

	CT_DEBUG(tracker, "Pose: orient %f %f %f %f pos %f %f %f", Txr_cam_device.orientation.x,
	         Txr_cam_device.orientation.y, Txr_cam_device.orientation.z, Txr_cam_device.orientation.w,
	         Txr_cam_device.position.x, Txr_cam_device.position.y, Txr_cam_device.position.z);

	std::shared_ptr<CameraMosaic> mosaic = camera->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	// Get the tracking origin
	struct xrt_pose Txr_world_origin;
	if (!get_tracking_origin_pose(*mosaic, tbo.timestamp_ns, Txr_world_origin)) {
		// Can't do anything if we can't locate the camera in the world.
		return;
	}

	struct xrt_pose Txr_world_cam;
	{
		std::unique_lock<os::Mutex> lock(camera->processing_lock);
		if (!camera->locked_data.has_concrete_pose) {
			// We don't have a concrete pose yet, so we can't actually push samples.
			return;
		}

		math_pose_transform(&Txr_world_origin, &camera->locked_data.Txr_origin_cam, &Txr_world_cam);
	}

	struct xrt_pose Txr_world_device;
	math_pose_transform(&Txr_world_cam, &Txr_cam_device, &Txr_world_device);

	// Push the sample to the device
	struct t_constellation_tracker_sample sample = {
	    .timestamp_ns = tbo.timestamp_ns,
	    .pose = Txr_world_device,
	};
	t_constellation_tracker_device_push_sample(device->device, &sample);

	{
		std::unique_lock<os::Mutex> lock(device->data_lock);

		device->locked_data.has_last_known = true;
		device->locked_data.Txr_world_device_last_known = Txr_world_device;
	}

	CT_DEBUG(tracker, "Found pose for device %d", device->id);
}

static void
slow_thread_process(Camera *camera, CameraSample &sample)
{
	ConstellationTracker *tracker = camera->tracker;

	auto &data = camera->slow_processing_thread_data;
	auto &tbo = sample.blobservation;

	correspondence_search_set_blobs(data.cs, tbo.blobs, tbo.num_blobs);

	std::shared_ptr<CameraMosaic> mosaic = camera->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	std::shared_lock lock(tracker->device_lock);

	for (std::unique_ptr<Device> &device : tracker->devices) {
		auto search_model = device->search_model;

		bool needs_slow_processing = false;
		for (uint32_t i = 0; i < sample.num_devices_needing_slow_processing; i++) {
			if (sample.needs_slow_processing[i] == device->id) {
				// This device needs slow processing, continue to the slow processing code below
				needs_slow_processing = true;
				break;
			}
		}

		if (!needs_slow_processing) {
			continue;
		}

		correspondence_search_flags search_flags =
		    (correspondence_search_flags)(CS_FLAG_STOP_FOR_STRONG_MATCH | CS_FLAG_DEEP_SEARCH);

		struct xrt_vec3 gravity_vector = {0.0, 1.0, 0.0};
		{
			std::unique_lock<os::Mutex> lock(camera->processing_lock);

			struct xrt_pose Txr_world_origin;

			// If we have a pose for the camera and we have a prior pose
			// (required by correspondence for search gravity matching)
			if (camera->locked_data.has_concrete_pose && (search_flags & CS_FLAG_HAVE_POSE_PRIOR) != 0 &&
			    get_tracking_origin_pose(*mosaic, tbo.timestamp_ns, Txr_world_origin)) {
				struct xrt_pose Txr_world_cam;
				math_pose_transform(&Txr_world_origin, &camera->locked_data.Txr_origin_cam,
				                    &Txr_world_cam);

				// Acquire the camera's gravity vector under the processing lock
				get_pose_gravity_vector(Txr_world_cam, gravity_vector);

				// Add in to check gravity
				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_MATCH_GRAVITY);
			}
		}

		struct pose_metrics score;
		struct pose_metrics_blob_match_info blob_match_info;

		struct xrt_pose Tcv_cam_device;
		if (correspondence_search_find_one_pose(data.cs, search_model, search_flags, &Tcv_cam_device,
		                                        &device->prior_pos_error, &device->prior_rot_error,
		                                        &gravity_vector, device->gravity_error_rad, &score)) {
			pose_metrics_match_pose_to_blobs(&Tcv_cam_device, tbo.blobs, tbo.num_blobs,
			                                 &device->params.led_model, device->id, &camera->model,
			                                 &blob_match_info);
			mark_matching_blobs(tracker, &tbo, &device->params.led_model, device->id, &blob_match_info);

			t_blobwatch_mark_blob_device(tbo.source, &tbo, device->id);

			push_pose(camera, device, score, Tcv_cam_device, tbo);
		}
	}
}

void *
constellation_tracker_camera_slow_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->slow_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->slow_processing_thread)) {
		os_thread_helper_wait_locked(&camera->slow_processing_thread);

		std::optional<CameraSample> maybe_sample;
		camera->slow_processing_thread_data.sample.Take(maybe_sample);

		os_thread_helper_unlock(&camera->slow_processing_thread);

		if (auto sample = maybe_sample) {
			slow_thread_process(camera, *sample);
		}

		os_thread_helper_lock(&camera->slow_processing_thread);
	}
	os_thread_helper_unlock(&camera->slow_processing_thread);

	return NULL;
}

static void
defer_to_slow_thread(Camera *camera, CameraSample &sample)
{
	os_thread_helper_lock(&camera->slow_processing_thread);
	camera->slow_processing_thread_data.sample.Push(sample.blobservation, sample.needs_slow_processing,
	                                                sample.num_devices_needing_slow_processing);
	os_thread_helper_signal_locked(&camera->slow_processing_thread);
	os_thread_helper_unlock(&camera->slow_processing_thread);
}

//! Fast matching based on prior pose
static bool
device_try_pose(Camera *camera,
                std::unique_ptr<Device> &device,
                CameraSample &sample,
                struct xrt_pose &Tcv_cam_world,
                struct xrt_pose &Tcv_world_device_prior,
                struct xrt_pose &Tcv_world_device_candidate)
{
	struct xrt_pose Tcv_cam_device_prior;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_device_prior, &Tcv_cam_device_prior);

	struct xrt_pose Tcv_cam_device_candidate;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_device_candidate, &Tcv_cam_device_candidate);

	struct pose_metrics score;
	pose_metrics_evaluate_pose_with_prior(&score, &Tcv_cam_device_candidate, false, &Tcv_cam_device_prior,
	                                      &device->prior_pos_error, &device->prior_rot_error,
	                                      sample.blobservation.blobs, sample.blobservation.num_blobs,
	                                      &device->params.led_model, device->id, &camera->model, NULL);

	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD | POSE_MATCH_LED_IDS)) {
		push_pose(camera, device, score, Tcv_cam_device_candidate, sample.blobservation);
		return true;
	}

	return false;
}

static bool
device_try_blob_recovery(Camera *camera,
                         std::unique_ptr<Device> &device,
                         CameraSample &sample,
                         struct xrt_pose &Tcv_cam_world,
                         struct xrt_pose &Tcv_world_device_prior)
{
	auto tracker = camera->tracker;

	auto &tbo = sample.blobservation;

	const uint32_t needed_blobs = 4;
	uint32_t num_blobs = 0;
	for (uint32_t index = 0; index < tbo.num_blobs; index++) {
		struct t_blob &b = tbo.blobs[index];
		if (b.matched_device_id == device->id) {
			num_blobs++;

			if (num_blobs >= needed_blobs) {
				break;
			}
		}
	}
	if (num_blobs < needed_blobs) {
		return false;
	}

	struct xrt_pose Tcv_cam_device_prior;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_device_prior, &Tcv_cam_device_prior);

	// RANSAC-PnP with the matched blobs
	struct xrt_pose Tcv_cam_device = Tcv_cam_device_prior;
	if (!ransac_pnp_pose(tracker->log_level, &Tcv_cam_device, tbo.blobs, tbo.num_blobs, &device->params.led_model,
	                     device->id, &camera->model, NULL, NULL)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP blob recovery for device %d from %u blobs failed",
		         (void *)camera, device->id, tbo.num_blobs);
		return false;
	}

	// Evaluate the pose
	struct pose_metrics score;
	pose_metrics_evaluate_pose_with_prior(
	    &score, &Tcv_cam_device, true, &Tcv_cam_device_prior, &device->prior_pos_error, &device->prior_rot_error,
	    tbo.blobs, tbo.num_blobs, &device->params.led_model, device->id, &camera->model, NULL);

	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP recovered pose for device %d from %u blobs", (void *)camera,
		         device->id, tbo.num_blobs);
		push_pose(camera, device, score, Tcv_cam_device, tbo, false);
		return true;
	}

	return false;
}

//! Returns whether a slow search is needed
static bool
fast_thread_process(Camera *camera, CameraSample &sample)
{
	ConstellationTracker *tracker = camera->tracker;

	std::shared_ptr<CameraMosaic> mosaic = camera->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera",
	                      false);

	// Get the tracking origin
	struct xrt_pose Txr_world_origin;
	if (!get_tracking_origin_pose(*mosaic, sample.blobservation.timestamp_ns, Txr_world_origin)) {
		// Can't do anything if we can't locate the camera in the world.
		return false;
	}

	struct xrt_pose Txr_world_cam;
	{
		std::unique_lock<os::Mutex> lock(camera->processing_lock);
		if (!camera->locked_data.has_concrete_pose) {
			// We don't have a concrete pose yet, so we can't actually do fast processing.
			return false;
		}

		math_pose_transform(&Txr_world_origin, &camera->locked_data.Txr_origin_cam, &Txr_world_cam);
	}

	struct xrt_pose Tcv_world_cam;
	math_pose_convert_opencv(&Txr_world_cam, &Tcv_world_cam);

	struct xrt_pose Tcv_cam_world;
	math_pose_invert(&Tcv_world_cam, &Tcv_cam_world);

	struct xrt_vec3 gravity_vector;
	get_pose_gravity_vector(Txr_world_cam, gravity_vector);

	bool need_full_search = false;
	std::shared_lock lock(tracker->device_lock);
	for (std::unique_ptr<Device> &device : tracker->devices) {
		struct xrt_space_relation device_prior_relation = XRT_SPACE_RELATION_ZERO; //< AKA "the prior"
		t_constellation_tracker_tracking_source_get_tracked_pose(
		    device->params.tracking_source, sample.blobservation.timestamp_ns, &device_prior_relation);

		struct xrt_pose Tcv_world_device; //< AKA "the prior"
		math_pose_convert_opencv(&device_prior_relation.pose, &Tcv_world_device);

		// if we have a valid prior pose, try to use it for fast matching
		if ((device_prior_relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
		    (device_prior_relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0 &&
		    device_try_pose(camera, device, sample, Tcv_cam_world, Tcv_world_device, Tcv_world_device)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded", device->id);
			continue; // try the next device, we found a pose!
		}

		// Try to get a last known pose
		bool has_last_known = false;
		struct xrt_pose Tcv_world_device_last_known;
		{
			std::unique_lock<os::Mutex> lock(device->data_lock);

			if (device->locked_data.has_last_known) {
				math_pose_convert_opencv(&device->locked_data.Txr_world_device_last_known,
				                         &Tcv_world_device_last_known);
				has_last_known = true;
			}
		}

		if (has_last_known && device_try_pose(camera, device, sample, Tcv_cam_world, Tcv_world_device,
		                                      Tcv_world_device_last_known)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with last known pose", device->id);
			continue; // try the next device, we found a pose!
		}

		if (device_try_blob_recovery(camera, device, sample, Tcv_cam_world, Tcv_world_device)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with blob recovery", device->id);
			continue; // try the next device, we found a pose!
		}

		sample.needs_slow_processing[sample.num_devices_needing_slow_processing++] = device->id;

		need_full_search = true;
	}

	return need_full_search;
}

void *
constellation_tracker_camera_fast_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->fast_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->fast_processing_thread)) {
		os_thread_helper_wait_locked(&camera->fast_processing_thread);

		std::optional<CameraSample> maybe_sample;
		camera->fast_processing_thread_data.sample.Take(maybe_sample);

		os_thread_helper_unlock(&camera->fast_processing_thread);

		// @todo: do fast processing, for now, always defer to slow thread, as if fast processing failed.
		if (auto sample = maybe_sample) {
			if (fast_thread_process(camera, *sample)) {
				defer_to_slow_thread(camera, *sample);
				CT_TRACE(camera->tracker,
				         "Fast processing for camera %p failed, deferring to slow thread",
				         (void *)camera);
			}
		}

		os_thread_helper_lock(&camera->fast_processing_thread);
	}
	os_thread_helper_unlock(&camera->fast_processing_thread);

	return NULL;
}

void
constellation_tracker_camera_push_blobs(struct t_blob_sink *tbs, struct t_blob_observation *tbo)
{
	Camera *camera = Camera::Get(tbs);
	ConstellationTracker *tracker = camera->tracker;

	CT_TRACE(tracker, "Received blob observation with %u blobs", tbo->num_blobs);

	if (tbo->num_blobs == 0) {
		CT_TRACE(tracker, "No blobs in observation, skipping processing");
		return;
	}

	// Send to the fast thread
	os_thread_helper_lock(&camera->fast_processing_thread);
	camera->fast_processing_thread_data.sample.Push(*tbo);
	os_thread_helper_signal_locked(&camera->fast_processing_thread);
	os_thread_helper_unlock(&camera->fast_processing_thread);
}

void
constellation_tracker_camera_destroy(struct t_blob_sink *tbs)
{
	// do nothing, the constellation tracker will clean up the blob sinks when it is destroyed.
}

void
constellation_tracker_node_break_apart(struct xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	tracker->running = false;

	// Stop all the threads
	for (auto &mosaic : tracker->mosaics) {
		for (auto &camera : mosaic->cameras) {
			if (camera->slow_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->slow_processing_thread);
			}

			if (camera->fast_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->fast_processing_thread);
			}
		}
	}
}

void
constellation_tracker_node_destroy(struct xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	u_var_remove_root(tracker);

	delete tracker;
}

int
t_constellation_tracker_create(struct xrt_frame_context *xfctx,
                               struct t_constellation_tracker_params *params,
                               struct t_constellation_tracker **out_tracker)
{
	try {
		ConstellationTracker *tracker = new ConstellationTracker(params);

		// Add us to the frame context
		xrt_frame_context_add(xfctx, &tracker->node);

		*out_tracker = (struct t_constellation_tracker *)tracker;

		// Setup debug variable tracking
		u_var_add_root(tracker, "Constellation Tracker", true);
		u_var_add_log_level(tracker, &tracker->log_level, "Log Level");
	} catch (const std::exception &e) {
		U_LOG_E("Failed to create constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_add_device(struct t_constellation_tracker *raw_tracker,
                                   struct t_constellation_tracker_device_params *params,
                                   struct t_constellation_tracker_device *device,
                                   t_constellation_device_id_t *out_device_id)
{
	ConstellationTracker *tracker = (ConstellationTracker *)raw_tracker;

	try {
		t_constellation_device_id_t device_id = tracker->AddDevice(params, device);
		*out_device_id = device_id;
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to add device to constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_remove_device(struct t_constellation_tracker *raw_tracker, t_constellation_device_id_t device)
{
	ConstellationTracker *tracker = (ConstellationTracker *)raw_tracker;

	try {
		tracker->RemoveDevice(device);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to add device to constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}
