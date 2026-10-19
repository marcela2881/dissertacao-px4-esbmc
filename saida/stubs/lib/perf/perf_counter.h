/* STUB de lib/perf/perf_counter.h: contadores de desempenho não afetam a lógica verificada. */
#pragma once
#include <stdint.h>
enum perf_counter_type { PC_COUNT, PC_ELAPSED, PC_INTERVAL };
typedef struct perf_ctr_header *perf_counter_t;
inline perf_counter_t perf_alloc(enum perf_counter_type, const char *) { return nullptr; }
inline void perf_free(perf_counter_t) {}
inline void perf_count(perf_counter_t) {}
inline void perf_begin(perf_counter_t) {}
inline void perf_end(perf_counter_t) {}
inline void perf_set_elapsed(perf_counter_t, int64_t) {}
inline void perf_print_counter(perf_counter_t) {}
inline uint64_t perf_event_count(perf_counter_t) { return 0; }
