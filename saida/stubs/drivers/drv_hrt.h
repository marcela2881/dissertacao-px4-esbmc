/*
 * STUB de drivers/drv_hrt.h. hrt_absolute_time() vem de px4_common_stub.h
 * (tempo não modelado). hrt_elapsed_time() devolve g_hrt_elapsed, escolhido
 * pelo harness (qualquer valor, ou uma faixa nas pré-condições).
 */
#pragma once
#include <px4_platform_common/px4_common_stub.h>
extern hrt_abstime g_hrt_elapsed;
inline hrt_abstime hrt_elapsed_time(const hrt_abstime *) { return g_hrt_elapsed; }
