// Copyright 2026, BlackJaxDev.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-wide simulated eye gaze, set by a window and read by a device.
 * @ingroup aux_util
 */

#include "util/u_simulated_eye_gaze.h"
#include "util/u_debug.h"

#include <string.h>


/*
 * The sample is packed into one 64-bit atomic so that a reader never sees a torn update:
 *   bit 63      valid
 *   bits 48-55  view index
 *   bits 24-47  u, 24-bit fixed point
 *   bits 0-23   v, 24-bit fixed point
 */
#define SAMPLE_VALID_BIT (UINT64_C(1) << 63)
#define SAMPLE_VIEW_SHIFT 48
#define SAMPLE_VIEW_MASK UINT64_C(0xFF)
#define SAMPLE_U_SHIFT 24
#define SAMPLE_COORD_MASK UINT64_C(0xFFFFFF)

#define MODE_UNSET -1

DEBUG_GET_ONCE_OPTION(simulated_eye_gaze, "SIMULATED_EYE_GAZE", NULL)

static xrt_atomic_s64_t g_sample = 0;
static xrt_atomic_s32_t g_mode = MODE_UNSET;


static enum u_simulated_eye_gaze_mode
parse_mode(const char *value)
{
	if (value == NULL || value[0] == '\0') {
		return U_SIMULATED_EYE_GAZE_MODE_HOVER;
	}
	if (strcmp(value, "off") == 0 || strcmp(value, "0") == 0 || strcmp(value, "false") == 0) {
		return U_SIMULATED_EYE_GAZE_MODE_OFF;
	}
	if (strcmp(value, "click") == 0) {
		return U_SIMULATED_EYE_GAZE_MODE_CLICK;
	}
	return U_SIMULATED_EYE_GAZE_MODE_HOVER;
}

static uint64_t
pack_coord(float value)
{
	if (!(value > 0.0f)) {
		value = 0.0f;
	} else if (value > 1.0f) {
		value = 1.0f;
	}
	// Double precision, because a float cannot hold 2^24 - 0.5 and would round up past the mask.
	return (uint64_t)((double)value * (double)SAMPLE_COORD_MASK + 0.5);
}

enum u_simulated_eye_gaze_mode
u_simulated_eye_gaze_get_mode(void)
{
	int32_t mode = xrt_atomic_s32_load(&g_mode);
	if (mode == MODE_UNSET) {
		// Two threads can parse at once, but they store the same value.
		mode = (int32_t)parse_mode(debug_get_option_simulated_eye_gaze());
		xrt_atomic_s32_cmpxchg(&g_mode, MODE_UNSET, mode);
		mode = xrt_atomic_s32_load(&g_mode);
	}
	return (enum u_simulated_eye_gaze_mode)mode;
}

void
u_simulated_eye_gaze_set_mode(enum u_simulated_eye_gaze_mode mode)
{
	xrt_atomic_s32_store(&g_mode, (int32_t)mode);
	if (mode == U_SIMULATED_EYE_GAZE_MODE_OFF) {
		u_simulated_eye_gaze_clear();
	}
}

void
u_simulated_eye_gaze_set(uint32_t view_index, float u, float v)
{
	uint64_t packed = SAMPLE_VALID_BIT;
	packed |= ((uint64_t)view_index & SAMPLE_VIEW_MASK) << SAMPLE_VIEW_SHIFT;
	packed |= pack_coord(u) << SAMPLE_U_SHIFT;
	packed |= pack_coord(v);
	xrt_atomic_s64_store(&g_sample, (int64_t)packed);
}

void
u_simulated_eye_gaze_clear(void)
{
	xrt_atomic_s64_store(&g_sample, 0);
}

void
u_simulated_eye_gaze_get(struct u_simulated_eye_gaze_sample *out_sample)
{
	uint64_t packed = (uint64_t)xrt_atomic_s64_load(&g_sample);

	out_sample->valid = (packed & SAMPLE_VALID_BIT) != 0;
	out_sample->view_index = (uint32_t)((packed >> SAMPLE_VIEW_SHIFT) & SAMPLE_VIEW_MASK);
	out_sample->u = (float)((packed >> SAMPLE_U_SHIFT) & SAMPLE_COORD_MASK) / (float)SAMPLE_COORD_MASK;
	out_sample->v = (float)(packed & SAMPLE_COORD_MASK) / (float)SAMPLE_COORD_MASK;
}
