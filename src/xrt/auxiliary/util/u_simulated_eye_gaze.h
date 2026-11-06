// Copyright 2026, BlackJaxDev.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-wide simulated eye gaze, set by a window and read by a device.
 *
 * The compositor window converts the mouse position over an eye preview into a gaze sample. A
 * simulated HMD then reports that sample through XR_EXT_eye_gaze_interaction. Both run in the
 * same process, so a lock-free global sample is enough.
 *
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * How the mouse drives the simulated eye gaze.
 *
 * @ingroup aux_util
 */
enum u_simulated_eye_gaze_mode
{
	//! No eye gaze. The device does not report XR_EXT_eye_gaze_interaction support.
	U_SIMULATED_EYE_GAZE_MODE_OFF = 0,
	//! The gaze follows the mouse while it moves over an eye preview.
	U_SIMULATED_EYE_GAZE_MODE_HOVER = 1,
	//! The gaze moves only while the left mouse button is held over an eye preview.
	U_SIMULATED_EYE_GAZE_MODE_CLICK = 2,
};

/*!
 * One simulated gaze sample: a point on one view, in that view's normalized image coordinates.
 *
 * @ingroup aux_util
 */
struct u_simulated_eye_gaze_sample
{
	//! False until the mouse has set a point. A device then looks straight ahead.
	bool valid;
	//! The view that the point is on.
	uint32_t view_index;
	//! Horizontal position, 0 at the left edge of the view and 1 at the right edge.
	float u;
	//! Vertical position, 0 at the top edge of the view and 1 at the bottom edge.
	float v;
};

/*!
 * Returns the current mode. The first call reads the SIMULATED_EYE_GAZE environment variable
 * (off, hover or click). The default is hover.
 *
 * @ingroup aux_util
 */
enum u_simulated_eye_gaze_mode
u_simulated_eye_gaze_get_mode(void);

/*!
 * Changes the mode at runtime. Changing to off also clears the sample.
 *
 * @ingroup aux_util
 */
void
u_simulated_eye_gaze_set_mode(enum u_simulated_eye_gaze_mode mode);

/*!
 * Stores a gaze point. The values of @p u and @p v are clamped to [0, 1].
 *
 * @ingroup aux_util
 */
void
u_simulated_eye_gaze_set(uint32_t view_index, float u, float v);

/*!
 * Clears the gaze point, so that a device looks straight ahead.
 *
 * @ingroup aux_util
 */
void
u_simulated_eye_gaze_clear(void);

/*!
 * Reads the current gaze point.
 *
 * @ingroup aux_util
 */
void
u_simulated_eye_gaze_get(struct u_simulated_eye_gaze_sample *out_sample);


#ifdef __cplusplus
}
#endif
