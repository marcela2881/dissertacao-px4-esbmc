/* STUB de ScheduledWorkItem.hpp: a fila de trabalho (agendamento) não é modelada.
 * Também traz o semáforo do PX4 (px4_sem_*), usado pelo MixingOutput só como trava. */
#pragma once
#include <px4_platform_common/px4_common_stub.h>

typedef int px4_sem_t;
inline int px4_sem_init(px4_sem_t *s, int, unsigned v) { *s = (int)v; return 0; }
inline int px4_sem_wait(px4_sem_t *) { return 0; }
inline int px4_sem_post(px4_sem_t *) { return 0; }
inline int px4_sem_destroy(px4_sem_t *) { return 0; }

namespace px4
{
typedef wq_configurations::wq_config_t wq_config_t;
namespace wq_configurations { static constexpr wq_config_t rate_ctrl{1}; }

class WorkItem
{
public:
	WorkItem(const char *, const wq_config_t &) {}
	virtual ~WorkItem() = default;
	bool ChangeWorkQueue(const wq_config_t &) { return true; }
	void ScheduleNow() {}
	virtual void Run() {}
};

class ScheduledWorkItem : public WorkItem
{
public:
	ScheduledWorkItem(const char *name, const wq_config_t &config) : WorkItem(name, config) {}
	void ScheduleDelayed(uint32_t) {}
	void ScheduleOnInterval(uint32_t, uint32_t = 0) {}
	void ScheduleClear() {}
};
}
