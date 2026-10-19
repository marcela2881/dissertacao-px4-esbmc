/* STUB uORB::SubscriptionCallbackWorkItem — sem chamadas de retorno (agendamento não modelado). */
#pragma once
#include <uORB/Subscription.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
namespace uORB {
class SubscriptionCallbackWorkItem : public Subscription {
public:
	SubscriptionCallbackWorkItem(px4::WorkItem *, int id, uint8_t = 0) : Subscription(id) {}
	bool registerCallback() { return true; }
	void unregisterCallback() {}
	void set_interval_us(uint32_t) {}
	bool updated() { return false; }
	template<class T> bool update(T *) { return false; }
};
}
