// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal structures for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_frame.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_var.h"
#include "util/u_threading.h"
#include "util/u_weak_ptr.hpp"

#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <stdexcept>
#include <array>

#include "correspondence_search.h"
#include "led_search_model.h"
#include "pose_optimize.h"
#include "pose_metrics.h"
#include "t_constellation_tracker.h"


DEBUG_GET_ONCE_LOG_OPTION(constellation_tracker_log, "CONSTELLATION_TRACKER_LOG", U_LOGGING_WARN)

#define CT_TRACE(ct, ...) U_LOG_IFL_T(ct->log_level, __VA_ARGS__)
#define CT_DEBUG(ct, ...) U_LOG_IFL_D(ct->log_level, __VA_ARGS__)
#define CT_INFO(ct, ...) U_LOG_IFL_I(ct->log_level, __VA_ARGS__)
#define CT_WARN(ct, ...) U_LOG_IFL_W(ct->log_level, __VA_ARGS__)
#define CT_ERROR(ct, ...) U_LOG_IFL_E(ct->log_level, __VA_ARGS__)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

/*
 *
 * Forward declares for C callback functions
 *
 */

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


namespace xrt::tracking::constellation {

namespace os = xrt::auxiliary::os;

struct CameraSample
{
	bool has_sample;

	struct t_blob blob_storage[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];

	struct t_blob_observation blobservation;

	std::array<t_constellation_device_id_t, XRT_CONSTELLATION_MAX_DEVICES> needs_slow_processing;
	uint32_t num_devices_needing_slow_processing;

	void
	Take(std::optional<CameraSample> &maybe_sample);

	void
	Push(t_blob_observation &blobservation,
	     std::array<t_constellation_device_id_t, XRT_CONSTELLATION_MAX_DEVICES> &devices_needing_slow_processing,
	     uint32_t num_devices_needing_slow_processing);

	void
	Push(t_blob_observation &blobservation);
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
	       enum u_logging_level *log_level_ptr);

	~Camera();
};

class CameraMosaic
{
public:
	std::vector<std::unique_ptr<Camera>> cameras;

	struct t_constellation_tracker_tracking_source *tracking_origin;

	CameraMosaic(ConstellationTracker *tracker, const struct t_constellation_tracker_camera_mosaic &mosaic_params);

	~CameraMosaic();
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
	       t_constellation_device_id_t id);

	~Device();
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

	static ConstellationTracker *
	Get(struct xrt_frame_node *node)
	{
		return container_of(node, ConstellationTracker, node);
	}

	ConstellationTracker(struct t_constellation_tracker_params *params);

	~ConstellationTracker();

	t_constellation_device_id_t
	AddDevice(struct t_constellation_tracker_device_params *params, struct t_constellation_tracker_device *device);

	void
	RemoveDevice(t_constellation_device_id_t device_id);
};

}; // namespace xrt::tracking::constellation
