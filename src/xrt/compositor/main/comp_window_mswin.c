// Copyright 2019-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Microsoft Windows window code.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "xrt/xrt_compiler.h"
#include "main/comp_window.h"
#include "util/u_misc.h"
#include "os/os_threading.h"


#undef ALLOW_CLOSING_WINDOW

/*
 *
 * Private structs.
 *
 */

/*!
 * A Microsoft Windows window.
 *
 * @implements comp_target_swapchain
 */
struct comp_window_mswin
{
	struct comp_target_swapchain base;
	struct os_thread_helper oth;

	ATOM window_class;
	HINSTANCE instance;
	HWND window;


	bool fullscreen_requested;
	bool should_exit;
	bool thread_started;
	bool thread_exited;

	char base_title[64];
	char current_title[256];
};

static WCHAR szWindowClass[] = L"Monado";
static WCHAR szWindowData[] = L"MonadoWindow";

#define XRE_OPENXR_EYE_RESOLUTION_PRESET_ENV "XRE_OPENXR_EYE_RESOLUTION_PRESET"
#define XRE_OPENXR_EYE_RESOLUTION_SCALE_ENV "XRE_OPENXR_EYE_RESOLUTION_SCALE"
#define XRE_OPENXR_EYE_RESOLUTION_WIDTH_ENV "XRE_OPENXR_EYE_RESOLUTION_WIDTH"
#define XRE_OPENXR_EYE_RESOLUTION_HEIGHT_ENV "XRE_OPENXR_EYE_RESOLUTION_HEIGHT"
#define XRE_UNIT_TEST_OPENXR_EYE_RESOLUTION_PRESET_ENV "XRE_UNIT_TEST_OPENXR_EYE_RESOLUTION_PRESET"
#define SIMULATED_HMD_POSE_MODE_ENV "SIMULATED_HMD_POSE_MODE"
#define SIMULATED_HMD_POSE_MODE_WOBBLE "wobble"
#define SIMULATED_HMD_POSE_MODE_USER_INPUT "user_input"
#define ID_SIMULATED_HMD_POSE_WOBBLE 50001
#define ID_SIMULATED_HMD_POSE_USER_INPUT 50003

#define COMP_ERROR_GETLASTERROR(C, MSG_WITH_PLACEHOLDER, MSG_WITHOUT_PLACEHOLDER)                                      \
	do {                                                                                                           \
		DWORD err = GetLastError();                                                                            \
		char *buf = NULL;                                                                                      \
		if (0 != FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, err,        \
		                        LANG_SYSTEM_DEFAULT, (LPSTR)&buf, 256, NULL)) {                                \
			COMP_ERROR(C, MSG_WITH_PLACEHOLDER, buf);                                                      \
			LocalFree(buf);                                                                                \
		} else {                                                                                               \
			COMP_ERROR(C, MSG_WITHOUT_PLACEHOLDER);                                                        \
		}                                                                                                      \
	} while (0)

static void
comp_window_mswin_refresh_title(struct comp_window_mswin *cwm);

static const char *
get_simulated_hmd_pose_mode(void)
{
	const char *mode = getenv(SIMULATED_HMD_POSE_MODE_ENV);
	return mode != NULL && mode[0] != '\0' ? mode : SIMULATED_HMD_POSE_MODE_WOBBLE;
}

static UINT
get_pose_mode_check_flag(const char *active_mode, const char *mode)
{
	return strcmp(active_mode, mode) == 0 ? MF_CHECKED : MF_UNCHECKED;
}

static void
set_simulated_hmd_pose_mode(const char *mode)
{
	if (mode == NULL) {
		return;
	}

	(void)_putenv_s(SIMULATED_HMD_POSE_MODE_ENV, mode);
	SetEnvironmentVariableA(SIMULATED_HMD_POSE_MODE_ENV, mode);
}

static void
show_simulated_hmd_pose_menu(HWND window, LPARAM lParam)
{
	POINT point = {0};
	if ((int)(short)LOWORD(lParam) == -1 && (int)(short)HIWORD(lParam) == -1) {
		GetCursorPos(&point);
	} else {
		point.x = (int)(short)LOWORD(lParam);
		point.y = (int)(short)HIWORD(lParam);
	}

	HMENU menu = CreatePopupMenu();
	if (menu == NULL) {
		return;
	}

	const char *mode = get_simulated_hmd_pose_mode();
	AppendMenuA(menu, MF_STRING | get_pose_mode_check_flag(mode, SIMULATED_HMD_POSE_MODE_WOBBLE),
	            ID_SIMULATED_HMD_POSE_WOBBLE, "Figure eight transform test");
	AppendMenuA(menu, MF_STRING | get_pose_mode_check_flag(mode, SIMULATED_HMD_POSE_MODE_USER_INPUT),
	            ID_SIMULATED_HMD_POSE_USER_INPUT, "User input HMD");
	AppendMenuA(menu, MF_SEPARATOR, 0, NULL);
	AppendMenuA(menu, MF_STRING | MF_GRAYED, 0, "WASD/QE move, up/down pitch, left/right yaw, comma/period roll");
	AppendMenuA(menu, MF_STRING | MF_GRAYED, 0, "Hold Shift to yaw faster");

	SetForegroundWindow(window);
	UINT command = TrackPopupMenu(menu, TPM_RIGHTBUTTON | TPM_RETURNCMD, point.x, point.y, 0, window, NULL);
	DestroyMenu(menu);

	switch (command) {
	case ID_SIMULATED_HMD_POSE_WOBBLE: set_simulated_hmd_pose_mode(SIMULATED_HMD_POSE_MODE_WOBBLE); break;
	case ID_SIMULATED_HMD_POSE_USER_INPUT: set_simulated_hmd_pose_mode(SIMULATED_HMD_POSE_MODE_USER_INPUT); break;
	default: break;
	}
}
/*
 *
 * Functions.
 *
 */

static void
draw_window(HWND hWnd, struct comp_window_mswin *cwm)
{
	ValidateRect(hWnd, NULL);
}

static LRESULT CALLBACK
WndProc(HWND hWnd, unsigned int message, WPARAM wParam, LPARAM lParam)
{
	struct comp_window_mswin *cwm = GetPropW(hWnd, szWindowData);

	if (!cwm) {
		// This is before we've set up our window, or for some other helper window...
		// We might want to handle messages differently in here.
		return DefWindowProcW(hWnd, message, wParam, lParam);
	}
	switch (message) {
	case WM_PAINT:
		// COMP_INFO(c, "WM_PAINT");
		draw_window(hWnd, cwm);
		break;
	case WM_SIZE: {
		RECT rect = {0};
		if (wParam != SIZE_MINIMIZED && GetClientRect(hWnd, &rect)) {
			LONG width = rect.right - rect.left;
			LONG height = rect.bottom - rect.top;
			if (width > 0 && height > 0) {
				cwm->base.base.width = (uint32_t)width;
				cwm->base.base.height = (uint32_t)height;
				comp_window_mswin_refresh_title(cwm);
			}
		}
		break;
	}
	case WM_CONTEXTMENU:
		show_simulated_hmd_pose_menu(hWnd, lParam);
		break;
	case WM_QUIT:
		// COMP_INFO(c, "WM_QUIT");
		PostQuitMessage(0);
		break;
	case WM_CLOSE:
		// COMP_INFO(c, "WM_CLOSE");
		cwm->should_exit = true;
		DestroyWindow(hWnd);
		cwm->window = NULL;
		break;
	case WM_DESTROY:
		// COMP_INFO(c, "WM_DESTROY");
		// Post a quit message and return.
		PostQuitMessage(0);
		break;
	default: return DefWindowProcW(hWnd, message, wParam, lParam);
	}
	return 0;
}


static inline struct vk_bundle *
get_vk(struct comp_window_mswin *cwm)
{
	return &cwm->base.base.c->base.vk;
}

static const char *
getenv_non_empty(const char *name)
{
	const char *value = getenv(name);
	return value != NULL && value[0] != '\0' ? value : NULL;
}

static uint32_t
getenv_u32_or(const char *name, uint32_t fallback)
{
	const char *value = getenv_non_empty(name);
	if (value == NULL) {
		return fallback;
	}

	char *end = NULL;
	unsigned long parsed = strtoul(value, &end, 10);
	if (end == value || parsed == 0 || parsed > UINT32_MAX) {
		return fallback;
	}

	return (uint32_t)parsed;
}

static void
set_window_title_utf8(HWND window, const char *title)
{
	if (window == NULL || title == NULL) {
		return;
	}

	WCHAR wide_title[256];
	int count = MultiByteToWideChar(CP_UTF8, 0, title, -1, wide_title, ARRAY_SIZE(wide_title));
	if (count <= 0) {
		return;
	}

	SetWindowTextW(window, wide_title);
}

static void
calc_aspect_fit(uint32_t src_width,
                uint32_t src_height,
                uint32_t dst_width,
                uint32_t dst_height,
                uint32_t *out_width,
                uint32_t *out_height)
{
	*out_width = dst_width;
	*out_height = dst_height;

	if (src_width == 0 || src_height == 0 || dst_width == 0 || dst_height == 0) {
		return;
	}

	uint64_t fit_width = dst_width;
	uint64_t fit_height = (fit_width * src_height) / src_width;
	if (fit_height == 0) {
		fit_height = 1;
	}

	if (fit_height > dst_height) {
		fit_height = dst_height;
		fit_width = (fit_height * src_width) / src_height;
		if (fit_width == 0) {
			fit_width = 1;
		}
	}

	*out_width = (uint32_t)fit_width;
	*out_height = (uint32_t)fit_height;
}

static void
comp_window_mswin_refresh_title(struct comp_window_mswin *cwm)
{
	struct comp_target *ct = &cwm->base.base;
	struct xrt_device *xdev = ct->c->xdev;
	uint32_t view_count = xdev->hmd->view_count > 1 ? xdev->hmd->view_count : 1;
	uint32_t source_eye_width = xdev->hmd->views[0].display.w_pixels;
	uint32_t source_eye_height = xdev->hmd->views[0].display.h_pixels;
	uint32_t full_source_width = xdev->hmd->screens[0].w_pixels;
	uint32_t full_source_height = xdev->hmd->screens[0].h_pixels;
	uint32_t content_width = ct->width;
	uint32_t content_height = ct->height;

	calc_aspect_fit(full_source_width, full_source_height, ct->width, ct->height, &content_width, &content_height);

	uint32_t preview_eye_width = content_width / view_count;
	if (preview_eye_width == 0) {
		preview_eye_width = content_width;
	}

	uint32_t title_eye_width = getenv_u32_or(XRE_OPENXR_EYE_RESOLUTION_WIDTH_ENV, source_eye_width);
	uint32_t title_eye_height = getenv_u32_or(XRE_OPENXR_EYE_RESOLUTION_HEIGHT_ENV, source_eye_height);
	double scale = title_eye_width != 0 ? (double)preview_eye_width / (double)title_eye_width : 0.0;

	const char *preset = getenv_non_empty(XRE_OPENXR_EYE_RESOLUTION_PRESET_ENV);
	if (preset == NULL) {
		preset = getenv_non_empty(XRE_UNIT_TEST_OPENXR_EYE_RESOLUTION_PRESET_ENV);
	}
	if (preset == NULL) {
		preset = "RuntimeRecommended";
	}

	const char *requested_scale = getenv_non_empty(XRE_OPENXR_EYE_RESOLUTION_SCALE_ENV);
	if (requested_scale == NULL) {
		requested_scale = "1.00";
	}

	char title[sizeof(cwm->current_title)];
	const char *base_title = cwm->base_title[0] != '\0' ? cwm->base_title : "Monado";
	(void)snprintf(title, sizeof(title),
	               "%s | preset %s @ %sx | internal eye %ux%u | window %ux%u | preview eye %ux%u %.2fx",
	               base_title, preset, requested_scale, title_eye_width, title_eye_height, ct->width, ct->height,
	               preview_eye_width, content_height, scale);

	if (strcmp(cwm->current_title, title) != 0) {
		strncpy(cwm->current_title, title, sizeof(cwm->current_title) - 1);
		cwm->current_title[sizeof(cwm->current_title) - 1] = '\0';
		set_window_title_utf8(cwm->window, cwm->current_title);
	}
}

static void
comp_window_mswin_destroy(struct comp_target *ct)
{
	struct comp_window_mswin *cwm = (struct comp_window_mswin *)ct;

	// Stop the Windows thread first, destroy also stops the thread.
	os_thread_helper_destroy(&cwm->oth);

	comp_target_swapchain_cleanup(&cwm->base);

	//! @todo

	free(ct);
}

static void
comp_window_mswin_update_window_title(struct comp_target *ct, const char *title)
{
	struct comp_window_mswin *cwm = (struct comp_window_mswin *)ct;
	if (title != NULL && title[0] != '\0') {
		strncpy(cwm->base_title, title, sizeof(cwm->base_title) - 1);
		cwm->base_title[sizeof(cwm->base_title) - 1] = '\0';
	}

	comp_window_mswin_refresh_title(cwm);
}

static void
comp_window_mswin_fullscreen(struct comp_window_mswin *w)
{
	//! @todo
}

static VkResult
comp_window_mswin_create_surface(struct comp_window_mswin *w, VkSurfaceKHR *out_surface)
{
	struct vk_bundle *vk = get_vk(w);
	VkResult ret;

	VkWin32SurfaceCreateInfoKHR surface_info = {
	    .sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR,
	    .hinstance = w->instance,
	    .hwnd = w->window,
	};

	VkSurfaceKHR surface = VK_NULL_HANDLE;
	ret = vk->vkCreateWin32SurfaceKHR( //
	    vk->instance,                  //
	    &surface_info,                 //
	    NULL,                          //
	    &surface);                     //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(w->base.base.c, "vkCreateWin32SurfaceKHR: %s", vk_result_string(ret));
		return ret;
	}

	VK_NAME_SURFACE(vk, surface, "comp_window_mswin surface");
	*out_surface = surface;

	return VK_SUCCESS;
}

static bool
comp_window_mswin_init_swapchain(struct comp_target *ct, uint32_t width, uint32_t height)
{
	struct comp_window_mswin *cwm = (struct comp_window_mswin *)ct;
	VkResult ret;

	ret = comp_window_mswin_create_surface(cwm, &cwm->base.surface.handle);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(ct->c, "Failed to create surface '%s'!", vk_result_string(ret));
		return false;
	}

	//! @todo

	return true;
}


static void
comp_window_mswin_flush(struct comp_target *ct)
{
	struct comp_window_mswin *cwm = (struct comp_window_mswin *)ct;
	comp_window_mswin_refresh_title(cwm);
}

static void
comp_window_mswin_window_loop(struct comp_window_mswin *cwm)
{
	struct comp_target *ct = &cwm->base.base;
	RECT rc = {0, 0, (LONG)(ct->width), (LONG)ct->height};

	COMP_INFO(ct->c, "Creating window");
	cwm->window =
	    CreateWindowExW(0, szWindowClass, L"Monado (Windowed)", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
	                    rc.right - rc.left, rc.bottom - rc.top, NULL, NULL, cwm->instance, NULL);
	if (cwm->window == NULL) {
		COMP_ERROR_GETLASTERROR(ct->c, "Failed to create window: %s", "Failed to create window");
		// parent thread will be notified (by caller) that we have exited.
		return;
	}

	COMP_INFO(ct->c, "Setting window properties and showing window");
	SetPropW(cwm->window, szWindowData, cwm);
	SetWindowLongPtr(cwm->window, GWLP_USERDATA, (LONG_PTR)(cwm));
	ShowWindow(cwm->window, SW_SHOWDEFAULT);
	UpdateWindow(cwm->window);

	COMP_INFO(ct->c, "Unblocking parent thread");
	// Unblock the parent thread now that we're successfully running.
	{
		os_thread_helper_lock(&cwm->oth);
		cwm->thread_started = true;
		os_thread_helper_signal_locked(&cwm->oth);
		os_thread_helper_unlock(&cwm->oth);
	}
	COMP_INFO(ct->c, "Starting the Windows window message loop");

	while (os_thread_helper_is_running(&cwm->oth)) {
		// force handling messages.
		MSG msg;
		while (PeekMessageW(&msg, cwm->window, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessageW(&msg);
#ifdef ALLOW_CLOSING_WINDOW
			/// @todo We need to bubble this up to multi-compositor
			/// and the state tracker (as "instance lost")
			if (msg.message == WM_QUIT) {
				COMP_INFO(cwm->base.base.c, "Got WM_QUIT message");
				return;
			}
			if (msg.message == WM_DESTROY) {
				COMP_INFO(cwm->base.base.c, "Got WM_DESTROY message");
				return;
			}
			if (cwm->should_exit) {
				COMP_INFO(cwm->base.base.c, "Got 'should_exit' flag.");
				return;
			}
#endif
		}
	}
	if (cwm->window != NULL) {
		// Got shut down by app code, not by a window message, so we still need to clean up our window.
		if (0 == DestroyWindow(cwm->window)) {
			COMP_ERROR_GETLASTERROR(ct->c, "DestroyWindow failed: %s", "DestroyWindow failed");
		}
		cwm->window = NULL;
	}
}

static void
comp_window_mswin_mark_exited(struct comp_window_mswin *cwm)
{
	// Unblock the parent thread to advise of our exit
	os_thread_helper_lock(&cwm->oth);
	cwm->thread_exited = true;
	os_thread_helper_signal_locked(&cwm->oth);
	os_thread_helper_unlock(&cwm->oth);
}

static void
comp_window_mswin_thread(struct comp_window_mswin *cwm)
{
	struct comp_target *ct = &cwm->base.base;

	WNDCLASSEXW wcex;
	U_ZERO(&wcex);
	wcex.cbSize = sizeof(WNDCLASSEXW);
	wcex.style = CS_HREDRAW | CS_VREDRAW;

	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = cwm->instance;
	wcex.lpszClassName = szWindowClass;
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
//! @todo icon
#if 0
	wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_SAMPLEGUI));
	wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_SAMPLEGUI);
	wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
#endif
	COMP_INFO(ct->c, "Registering window class");
	if (0 == RegisterClassExW(&wcex)) {
		COMP_ERROR_GETLASTERROR(ct->c, "Failed to register window class: %s",
		                        "Failed to register window class");
		comp_window_mswin_mark_exited(cwm);
		return;
	}

	comp_window_mswin_window_loop(cwm);

	COMP_INFO(ct->c, "Unregistering window class");
	if (0 == UnregisterClassW(szWindowClass, NULL)) {
		COMP_ERROR_GETLASTERROR(ct->c, "Failed to unregister window class: %s",
		                        "Failed to unregister window class");
	}

	comp_window_mswin_mark_exited(cwm);
}

static void *
comp_window_mswin_thread_func(void *ptr)
{

	struct comp_window_mswin *cwm = (struct comp_window_mswin *)ptr;
	os_thread_helper_name(&(cwm->oth), "Compositor Window Message Thread");

	comp_window_mswin_thread(cwm);
	os_thread_helper_signal_stop(&cwm->oth);
	COMP_INFO(cwm->base.base.c, "Windows window message thread now exiting.");
	return NULL;
}

static bool
comp_window_mswin_init(struct comp_target *ct)
{
	struct comp_window_mswin *cwm = (struct comp_window_mswin *)ct;
	cwm->instance = GetModuleHandle(NULL);

	ct->width = 1280;
	ct->height = 720;

	if (os_thread_helper_start(&cwm->oth, comp_window_mswin_thread_func, cwm) != 0) {
		COMP_ERROR(ct->c, "Failed to start Windows window message thread");
		return false;
	}

	// Wait for thread to start, create window, etc.
	os_thread_helper_lock(&cwm->oth);
	while (!cwm->thread_started && !cwm->thread_exited) {
		os_thread_helper_wait_locked(&cwm->oth);
	}
	bool ret = cwm->thread_started && !cwm->thread_exited;
	os_thread_helper_unlock(&cwm->oth);
	return ret;
}

XRT_MAYBE_UNUSED static void
comp_window_mswin_configure(struct comp_window_mswin *w, int32_t width, int32_t height)
{
	if (w->base.base.c->settings.fullscreen && !w->fullscreen_requested) {
		COMP_DEBUG(w->base.base.c, "Setting full screen");
		comp_window_mswin_fullscreen(w);
		w->fullscreen_requested = true;
	}
}

#ifdef ALLOW_CLOSING_WINDOW
/// @todo This is somehow triggering crashes in the multi-compositor, which is trying to run without things it needs,
/// even though it didn't do this when we called the parent impl instead of inlining it.
static bool
comp_window_mswin_configure_check_ready(struct comp_target *ct)
{
	struct comp_window_mswin *cwm = (struct comp_window_mswin *)ct;
	return os_thread_helper_is_running(&cwm->oth) && cwm->base.swapchain.handle != VK_NULL_HANDLE;
}
#endif

struct comp_target *
comp_window_mswin_create(struct comp_compositor *c)
{
	struct comp_window_mswin *w = U_TYPED_CALLOC(struct comp_window_mswin);
	if (os_thread_helper_init(&w->oth) != 0) {
		COMP_ERROR(c, "Failed to init Windows window message thread");
		free(w);
		return NULL;
	}

	// The display timing code hasn't been tested on Windows and may be broken.
	comp_target_swapchain_init_and_set_fnptrs(&w->base, COMP_TARGET_FORCE_FAKE_DISPLAY_TIMING);

	w->base.base.name = "MS Windows";
	(void)snprintf(w->base_title, sizeof(w->base_title), "%s", "Monado (Windowed)");
	w->base.display = VK_NULL_HANDLE;
	w->base.base.destroy = comp_window_mswin_destroy;
	w->base.base.flush = comp_window_mswin_flush;
	w->base.base.init_pre_vulkan = comp_window_mswin_init;
	w->base.base.init_post_vulkan = comp_window_mswin_init_swapchain;
	w->base.base.set_title = comp_window_mswin_update_window_title;
#ifdef ALLOW_CLOSING_WINDOW
	w->base.base.check_ready = comp_window_mswin_configure_check_ready;
#endif
	w->base.base.c = c;

	return &w->base.base;
}


/*
 *
 * Factory
 *
 */

static const char *instance_extensions[] = {
    VK_KHR_WIN32_SURFACE_EXTENSION_NAME,
};

static bool
detect(const struct comp_target_factory *ctf, struct comp_compositor *c)
{
	return false;
}

static bool
create_target(const struct comp_target_factory *ctf, struct comp_compositor *c, struct comp_target **out_ct)
{
	struct comp_target *ct = comp_window_mswin_create(c);
	if (ct == NULL) {
		return false;
	}

	*out_ct = ct;

	return true;
}

const struct comp_target_factory comp_target_factory_mswin = {
    .name = "Microsoft Windows(TM)",
    .identifier = "mswin",
    .requires_vulkan_for_create = false,
    .is_deferred = true,
    .required_instance_version = 0,
    .required_instance_extensions = instance_extensions,
    .required_instance_extension_count = ARRAY_SIZE(instance_extensions),
    .optional_device_extensions = NULL,
    .optional_device_extension_count = 0,
    .detect = detect,
    .create_target = create_target,
};
