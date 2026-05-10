// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Implementation of the latest IVRClientCore interface version.
 *
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup openvr_interfaces
 */

#pragma once

#include "xrt/xrt_instance.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_session.h"
#include "xrt/xrt_compositor.h"

#include "openvr_devices.hpp"
#include "openvr_events.hpp"

#include "compositor/openvr_compositor.hpp"

#include "openvr_interfaces_unified.h"
#include "IVRClientCore_003.h"

#include <memory>


namespace xrt::state_trackers::openvr {

#define SUPPORTED_CORE_INTERFACES(_) _(VRClientCore, _003)
#define SUPPORTED_INTERFACES(_) _(VRCompositor, _029) _(VRRenderModels, _006) _(VRSystem, _026) _(VRSystem, _023)

class XRTVRClientCore_003 : public vr::IVRClientCore_003
{
public: // Fields
	xrt_instance *xinst{nullptr};

	xrt_system *xsys{nullptr};
	xrt_system_devices *xsysd{nullptr};
	xrt_space_overseer *xso{nullptr};
	xrt_system_compositor *xsysc{nullptr};

	xrt_session *xs{nullptr};
	xrt_compositor_native *xcn{nullptr};

	std::shared_ptr<Compositor> compositor{};
	std::shared_ptr<Devices> devices{};
	std::shared_ptr<Events> events{};

#define XRTVR_INTERFACE_MEMBER(name, version) std::shared_ptr<vr::I##name##version> name##version{nullptr};
	SUPPORTED_INTERFACES(XRTVR_INTERFACE_MEMBER)
#undef XRTVR_INTERFACE_MEMBER

public: // Methods
	vr::EVRInitError
	Init(vr::EVRApplicationType eApplicationType, const char *pStartupInfo) override;

	void
	Cleanup() override;

	vr::EVRInitError
	IsInterfaceVersionValid(const char *pchInterfaceVersion) override;

	void *
	GetGenericInterface(const char *pchNameAndVersion, vr::EVRInitError *peError) override;

	bool
	BIsHmdPresent() override;

	const char *
	GetEnglishStringForHmdError(vr::EVRInitError eError) override;

	const char *
	GetIDForVRInitError(vr::EVRInitError eError) override;
};

}; // namespace xrt::state_trackers::openvr
