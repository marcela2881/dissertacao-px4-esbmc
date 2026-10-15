/* STUB: constantes e funções de plataforma usadas pelo driver. */
#pragma once
#include <cerrno>
#define PX4_OK 0
#define PX4_ERROR (-1)
inline int px4_usleep(unsigned) { return 0; }   /* tempo não é modelado */
