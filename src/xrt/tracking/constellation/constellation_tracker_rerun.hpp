// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal header for constellation tracker usage of the constellation tracker rerun logging.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_config_build.h"

// @note In general usage, we don't use the internal header, rerun is an opt-in debugging feature, so we are OK using
//       internal APIs here to keep code clean and non-invasive. Please look at target_builder_rift.c instead for normal
//       usage of these APIs.
#include "t_constellation_tracker_internal.hpp"

#include <rerun.hpp>


namespace xrt::tracking::constellation {

const std::string rerun_recording_id = "constellation_tracking";

struct RerunContext
{
public: // Fields
	std::unique_ptr<rerun::RecordingStream> stream{};

public: // Methods
	RerunContext()
	{
		stream = std::make_unique<rerun::RecordingStream>(rerun_recording_id);
	}

	/*!
	 * Log a camera sample to rerun.
	 *
	 * @remarks Call with `tracker->device_lock` held.
	 *
	 * @param tracker       The constellation tracker.
	 * @param camera_sample The camera sample to log.
	 */
	void
	LogSample(const ConstellationTracker &tracker, const CameraSample &camera_sample);

private: // Methods
	void
	LogStaticScene(const CameraSample &camera_sample, const t_camera_calibration &calibration);

	void
	LogLedModel(const std::string &entity_name,
	            t_constellation_device_id_t device_id,
	            const t_constellation_tracker_led_model &led_model,
	            bool prior);

	void
	LogBlobSet(const CameraSample &camera_sample);

	void
	LogFrameCameraMetrics(const CameraSample &camera_sample);

	void
	LogFrameDeviceMetrics(const CameraSample &camera_sample, const DeviceState &device_state);
};

}; // namespace xrt::tracking::constellation
