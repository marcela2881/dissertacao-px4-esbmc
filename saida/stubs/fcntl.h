/*
 * STUB de <fcntl.h>. O <fcntl.h> do sistema (Linux) traz struct iovec
 * (bits/fcntl-linux.h), que o ESBMC 8.5 já define internamente ->
 * "redefinition of 'iovec'" (visto na fm07, 16/10/2026). Como as pastas de
 * stubs vêm antes das do sistema no -I, este arquivo substitui o do sistema
 * para os fontes do PX4 que fazem #include <fcntl.h>. Só o que os drivers usam
 * (valores do Linux/x86-64).
 */
#pragma once
#ifndef O_RDONLY
#define O_RDONLY   00
#endif
#ifndef O_WRONLY
#define O_WRONLY   01
#endif
#ifndef O_RDWR
#define O_RDWR     02
#endif
#ifndef O_NOCTTY
#define O_NOCTTY   0400
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK 04000
#endif
#ifdef __cplusplus
extern "C" {
#endif
int open(const char *path, int flags, ...);
#ifdef __cplusplus
}
#endif
