/*
 * STUB de mixer_module.hpp — OutputModuleInterface e MixingOutput.
 * O mixer é verificado como módulo próprio; aqui só a interface que os
 * drivers usam (o driver recebe outputs[] já limitados pelo mixer).
 */
#pragma once
#include <px4_platform_common/px4_common_stub.h>
#include <px4_platform_common/module_params.h>

#ifndef PWM_OUTPUT_MAX_CHANNELS
#define PWM_OUTPUT_MAX_CHANNELS 16
#endif

class OutputModuleInterface : public ModuleParams
{
public:
	static constexpr int MAX_ACTUATORS = PWM_OUTPUT_MAX_CHANNELS;
	OutputModuleInterface(const char *, const px4::wq_configurations::wq_config_t &) {}
	virtual ~OutputModuleInterface() = default;
	virtual bool updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated) = 0;
	virtual void Run() {}
	void ScheduleNow() {}
	void ScheduleClear() {}
	void ScheduleDelayed(hrt_abstime) {}
	void ScheduleOnInterval(hrt_abstime, hrt_abstime = 0) {}
};

class MixingOutput : public ModuleParams
{
public:
	enum class SchedulingPolicy { Disabled, Auto };
	MixingOutput(const char *, uint8_t, OutputModuleInterface &, SchedulingPolicy, bool, bool = true) {}
	bool update() { return true; }
	void unregister() {}
	void updateSubscriptions(bool) {}
};
