/* STUB de module.h — ModuleBase (ciclo de vida do módulo, sem lógica verificada). */
#pragma once
#include "px4_common_stub.h"
#include <atomic>

#define PRINT_MODULE_DESCRIPTION(...)        ((void)0)
#define PRINT_MODULE_USAGE_NAME(...)         ((void)0)
#define PRINT_MODULE_USAGE_COMMAND(...)      ((void)0)
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS() ((void)0)
static constexpr int task_id_is_work_queue = -2;

class ModuleBase
{
public:
	struct Descriptor {
		int (*task_spawn)(int, char **);
		int (*custom_command)(int, char **);
		int (*print_usage)(const char *);
		struct { void store(void *) {} } object;
		int task_id{-1};
	};
	virtual ~ModuleBase() = default;
	virtual int print_status() { return 0; }
	static int main(Descriptor &, int, char **) { return 0; }
	bool should_exit() const { return false; }
	void request_stop() {}
	static void exit_and_cleanup(Descriptor &) {}
};
