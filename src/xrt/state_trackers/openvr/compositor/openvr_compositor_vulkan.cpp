// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Implementation of Vulkan-specific compositor functionality for OpenVR.
 *
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup openvr
 */

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_gfx_vk.h"

#include "util/u_pretty_print.h"
#include "util/u_debug.h"

#include "vk/vk_helpers.h"

#include "client/comp_vk_client.h"

#include "common/openvr_error.hpp"

#include "openvr_compositor.hpp"

#include <vector>


#ifndef XRT_HAVE_VULKAN
#error "This file should not be compiled without Vulkan support!"
#endif

namespace xrt::state_trackers::openvr {

DEBUG_GET_ONCE_LOG_OPTION(compositor_log, "XRT_COMPOSITOR_LOG", U_LOGGING_WARN)

/*
 *
 * Compositor implementations
 *
 */

xrt_result_t
Compositor::SetupVulkanCompositor(openvr_logger &logger, vr::VRVulkanTextureData_t &vulkan_data)
{
	VkInstance vkInstance = vulkan_data.m_pInstance;
	VkPhysicalDevice vkPhysicalDevice = vulkan_data.m_pPhysicalDevice;
	VkDevice vkDevice = vulkan_data.m_pDevice;
	uint32_t queueFamilyIndex = vulkan_data.m_nQueueFamilyIndex;

	auto vulkanGetInstanceProcAddr = vkGetInstanceProcAddr;

#define GET_PROC(INST, NAME) PFN_vk##NAME loaded_##NAME = (PFN_vk##NAME)vulkanGetInstanceProcAddr(INST, "vk" #NAME)
	GET_PROC(vkInstance, GetDeviceQueue);
	GET_PROC(vkInstance, GetPhysicalDeviceQueueFamilyProperties);
#undef GET_PROC

	uint32_t queueFamilyCount;
	loaded_GetPhysicalDeviceQueueFamilyProperties(vkPhysicalDevice, &queueFamilyCount, nullptr);
	std::vector<VkQueueFamilyProperties> properties(queueFamilyCount);
	loaded_GetPhysicalDeviceQueueFamilyProperties(vkPhysicalDevice, &queueFamilyCount, properties.data());

	uint32_t queueCount = properties[queueFamilyIndex].queueCount;
	uint32_t queueIndex = 0;
	for (; queueIndex < queueCount; queueIndex++) {
		VkQueue queue;
		// @todo this is very very dumb, OpenVR doesn't give us queue index, only a VkQueue, so we need this
		//       sillyness to find the queue index to pass into Monado. Perhaps there's a cleaner way?
		loaded_GetDeviceQueue(vkDevice, queueFamilyIndex, queueIndex, &queue);
		if (queue == vulkan_data.m_pQueue) {
			break;
		}
	}

	this->xc_vk = xrt_gfx_vk_provider_create( //
	    this->xcn,                            //
	    vkInstance,                           //
	    vkGetInstanceProcAddr,                //
	    vkPhysicalDevice,                     //
	    vkDevice,                             //
	    false,                                //
	    false,                                //
	    false,                                //
	    false,                                //
	    false,                                //
	    false,                                //
	    queueFamilyIndex,                     //
	    queueIndex);                          //

	if (this->xc_vk == nullptr) {
		OPENVR_LOG_ERROR(logger, "Failed to create Vulkan compositor client");
		return XRT_ERROR_VULKAN;
	}

	xrt_begin_session_info bsi = {
	    .view_type = XRT_VIEW_TYPE_STEREO,
	    .ext_hand_tracking_enabled = false,
	    .ext_hand_tracking_data_source_enabled = false,
	    .ext_eye_gaze_interaction_enabled = false,
	    .ext_future_enabled = false,
	    .ext_hand_interaction_enabled = false,
	    .htc_facial_tracking_enabled = false,
	    .fb_body_tracking_enabled = false,
	    .fb_face_tracking2_enabled = false,
	    .meta_body_tracking_full_body_enabled = false,
	    .meta_body_tracking_calibration_enabled = false,
	    .meta_body_tracking_fidelity_enabled = false,
	    .android_face_tracking_enabled = false,
	};
	xrt_result_t xret = xrt_comp_begin_session(&this->xc_vk->base, &bsi);

	if (xret != XRT_SUCCESS) {
		OPENVR_LOG_ERROR_XRET(logger, "Failed to begin Vulkan compositor session", xret);

		// Destroy the compositor we created, since it's not usable.
		struct xrt_compositor *xc = &this->xc_vk->base;
		xrt_comp_destroy(&xc);
		this->xc_vk = nullptr;

		return xret;
	}

	// We've created an active compositor, and begun the session.
	this->active_compositor = &this->xc_vk->base;

	return XRT_SUCCESS;
}

xrt_result_t
Compositor::BlitAppImageToSwapchainImage(openvr_logger &logger,
                                         xrt_swapchain *xsc,
                                         uint32_t dst_index,
                                         vr::VRVulkanTextureData_t &texture_data,
                                         vr::EColorSpace color_space,
                                         const xrt_rect &rect)
{
	client_vk_compositor *c = (client_vk_compositor *)this->xc_vk;

	vk_cmd_first_mip_image src_image = {
	    .base_array_layer = 0,
	    .aspect_mask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .image = (VkImage)texture_data.m_nImage,
	};
	vk_cmd_blit_image_params src_params = {
	    .layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
	    .access_mask = VK_ACCESS_TRANSFER_READ_BIT,
	    .stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT,
	    .rect = rect,
	};

	vk_cmd_blit_image_params dst_params = {
	    .layout = VK_IMAGE_LAYOUT_UNDEFINED,
	    .access_mask = 0,
	    .stage_mask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
	    .rect = rect,
	};

	return client_vk_compositor_blit_to_swapchain(c, xsc, &src_image, &src_params, dst_index, &dst_params);
}

vr::EVRCompositorError
Compositor::SubmitVulkan(openvr_logger &logger,
                         vr::EVREye eye,
                         vr::VRVulkanTextureData_t &texture_data,
                         vr::EColorSpace color_space,
                         const vr::VRTextureBounds_t &bounds)
{
	// @todo validate what parts of `texture_data` we need to validate and return InvalidTexture when invalid

	if (this->active_compositor == nullptr) {
		xrt_result_t xret = this->SetupVulkanCompositor(logger, texture_data);

		if (xret != XRT_SUCCESS) {
			OPENVR_LOG_ERROR(logger, "Failed to setup Vulkan compositor for submission");
			return xret_to_compositor_error(xret);
		}
	}

	assert(this->active_compositor != nullptr);

	SwapchainCache &cache = this->swapchain_caches[eye];

	xrt_result_t xret =
	    cache.EnsureSwapchain(this->active_compositor, texture_data.m_nFormat, texture_data.m_nWidth,
	                          texture_data.m_nHeight, texture_data.m_nSampleCount);
	if (xret != XRT_SUCCESS) {
		OPENVR_LOG_ERROR_XRET(logger, "Failed to get swapchain for submitted texture", xret);
		return xret_to_compositor_error(xret);
	}

	xrt_swapchain *xsc = cache.xsc;

	uint32_t image_index;

	// Acquire image
	xret = xrt_swapchain_acquire_image(xsc, &image_index);
	if (xret != XRT_SUCCESS) {
		OPENVR_LOG_ERROR_XRET(logger, "Failed to acquire swapchain image", xret);
		return xret_to_compositor_error(xret);
	}

	// Wait for it to become available
	xret = xrt_swapchain_wait_image(xsc, XRT_INFINITE_DURATION, image_index);
	if (xret != XRT_SUCCESS) {
		OPENVR_LOG_ERROR_XRET(logger, "Failed to wait for image to be available", xret);
		return xret_to_compositor_error(xret);
	}

	// Barrier it for our own use
	xret = xrt_swapchain_barrier_image(xsc, XRT_BARRIER_TO_APP, image_index);
	if (xret != XRT_SUCCESS) {
		OPENVR_LOG_ERROR_XRET(logger, "Failed to barrier image for app use", xret);
		return xret_to_compositor_error(xret);
	}

	xrt_rect rect = {
	    .offset =
	        {
	            .w = (int)(bounds.uMin * texture_data.m_nWidth),
	            .h = (int)(bounds.vMin * texture_data.m_nHeight),
	        },
	    .extent =
	        {
	            .w = (int)((bounds.uMax - bounds.uMin) * texture_data.m_nWidth),
	            .h = (int)((bounds.vMax - bounds.vMin) * texture_data.m_nHeight),
	        },
	};


	// Copy the app texture into the swapchain image
	xret = BlitAppImageToSwapchainImage(logger, xsc, image_index, texture_data, color_space, rect);
	if (xret != XRT_SUCCESS) {
		OPENVR_LOG_ERROR_XRET(logger, "Failed to blit app image to swapchain image", xret);
		return xret_to_compositor_error(xret);
	}

	// Barrier it for compositor use
	xret = xrt_swapchain_barrier_image(xsc, XRT_BARRIER_TO_COMP, image_index);
	if (xret != XRT_SUCCESS) {
		OPENVR_LOG_ERROR_XRET(logger, "Failed to barrier image for compositor use", xret);
		return xret_to_compositor_error(xret);
	}

	this->frame_eye_states[eye] = {
	    .texture =
	        {
	            .xsc = nullptr,
	            .swapchain_index = image_index,

	            .format = texture_data.m_nFormat,
	            .width = texture_data.m_nWidth,
	            .height = texture_data.m_nHeight,
	            .sample_count = texture_data.m_nSampleCount,

	            .bounds = rect,
	        },
	};
	xrt_swapchain_reference(&this->frame_eye_states[eye]->texture.xsc, xsc);

	return vr::EVRCompositorError::VRCompositorError_None;
}

#define UUID_STR_SIZE (XRT_UUID_SIZE * 3 + 1)

// @todo de-dup this all with oxr_vulkan
static void
snprint_uuid(char *str, size_t size, const xrt_uuid_t *uuid)
{
	for (size_t i = 0, offset = 0; i < ARRAY_SIZE(uuid->data) && offset < size; i++, offset += 3) {
		snprintf(str + offset, size - offset, "%02x ", uuid->data[i]);
	}
}

static void
snprint_luid(char *str, size_t size, xrt_luid_t *luid)
{
	for (size_t i = 0, offset = 0; i < ARRAY_SIZE(luid->data) && offset < size; i++, offset += 3) {
		snprintf(str + offset, size - offset, "%02x ", luid->data[i]);
	}
}

void
Compositor::GetVulkanOutputDevice(openvr_logger &logger, uint64_t *out_device, VkInstance pInstance)
{
	VkInstance vulkanInstance = (VkInstance)pInstance;

	auto vulkanGetInstanceProcAddr = vkGetInstanceProcAddr;

	// @todo TODO: dedup this whole function with oxr_vulkan
#define GET_PROC(INST, NAME) PFN_vk##NAME loaded_##NAME = (PFN_vk##NAME)vulkanGetInstanceProcAddr(INST, "vk" #NAME)
	GET_PROC(vulkanInstance, EnumeratePhysicalDevices);
	GET_PROC(vulkanInstance, GetPhysicalDeviceProperties2KHR);
#undef GET_PROC

	VkResult vk_ret;
	uint32_t count;

	vk_ret = loaded_EnumeratePhysicalDevices(vulkanInstance, &count, NULL);
	if (vk_ret != VK_SUCCESS) {
		OPENVR_LOG_ERROR(logger, "Failed to enumerate physical devices to get output device");
		*out_device = 0;
		return;
	}

	if (count == 0) {
		OPENVR_LOG_WARN(logger, "No Vulkan physical devices found");
		*out_device = 0;
		return;
	}

	std::vector<VkPhysicalDevice> phys(count);
	vk_ret = loaded_EnumeratePhysicalDevices(vulkanInstance, &count, phys.data());
	if (vk_ret != VK_SUCCESS) {
		OPENVR_LOG_ERROR(logger, "Failed to enumerate physical devices to get output device");
		*out_device = 0;
		return;
	}

	char suggested_uuid_str[UUID_STR_SIZE] = {0};
	snprint_uuid(suggested_uuid_str, ARRAY_SIZE(suggested_uuid_str), &this->xsysc->info.client_vk_deviceUUID);

	enum u_logging_level log_level = debug_get_log_option_compositor_log();
	int gpu_index = -1;
	for (uint32_t i = 0; i < count; i++) {
		VkPhysicalDeviceIDProperties pdidp = {
		    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES,
		    .pNext = nullptr,
		    .deviceUUID = {},
		    .driverUUID = {},
		    .deviceLUID = {},
		    .deviceNodeMask = 0,
		    .deviceLUIDValid = VK_FALSE,
		};

		VkPhysicalDeviceProperties2 pdp2 = {
		    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2,
		    .pNext = &pdidp,
		    .properties = {},
		};

		loaded_GetPhysicalDeviceProperties2KHR(phys[i], &pdp2);

		// These should always be true
		static_assert(VK_UUID_SIZE == XRT_UUID_SIZE, "uuid sizes mismatch");
		static_assert(ARRAY_SIZE(pdidp.deviceUUID) == XRT_UUID_SIZE, "array size mismatch");

		char buffer[UUID_STR_SIZE] = {0};
		if (log_level <= U_LOGGING_DEBUG) {
			snprint_uuid(buffer, ARRAY_SIZE(buffer), (xrt_uuid_t *)pdidp.deviceUUID);
			// oxr_log(log, "GPU: #%d, uuid: %s", i, buffer);
			if (pdidp.deviceLUIDValid == VK_TRUE) {
				snprint_luid(buffer, ARRAY_SIZE(buffer), (xrt_luid_t *)pdidp.deviceLUID);
				// oxr_log(log, "  LUID: %s", buffer);
			}
		}

		if (memcmp(pdidp.deviceUUID, this->xsysc->info.client_vk_deviceUUID.data, XRT_UUID_SIZE) == 0) {
			gpu_index = i;
			if (log_level <= U_LOGGING_DEBUG) {
				OPENVR_LOG_DEBUG(logger, "Using GPU #%d with uuid %s suggested by runtime", gpu_index,
				                 buffer);
			}
			break;
		}
	}

	if (gpu_index == -1) {
		OPENVR_LOG_WARN(logger, "Did not find runtime suggested GPU, fall back to GPU 0\n\tuuid: %s",
		                suggested_uuid_str);
		gpu_index = 0;
	}

	*out_device = (uint64_t)phys[gpu_index];
}

}; // namespace xrt::state_trackers::openvr
