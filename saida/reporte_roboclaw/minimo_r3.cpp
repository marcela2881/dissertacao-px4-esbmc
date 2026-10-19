/*
 * Reprodução MÍNIMA do R3, para anexar à issue (não precisa do PX4).
 * Mesmo padrão de Roboclaw.cpp:380-385 (writeCommand) e :332-338
 * (writeCommandWithPayload):
 *     size_t bytes_written = write(_uart_fd, buffer, 2);
 *     if (bytes_written < 2) { ... return ERROR; }
 *     return OK;
 * write() devolve ssize_t; em erro devolve -1 (POSIX). Guardado em size_t,
 * -1 vira SIZE_MAX, o teste "< 2" é falso e a falha é tratada como sucesso.
 * Compilar: g++ -std=c++17 minimo_r3.cpp -o minimo_r3 && ./minimo_r3
 */
#include <cstdio>
#include <cstddef>
#include <sys/types.h>

static ssize_t write_que_falha(int, const void *, size_t) { return -1; }   /* erro POSIX */

static int writeCommand()
{
	unsigned char buffer[2] = {0x80, 18};
	size_t bytes_written = write_que_falha(3, buffer, 2);

	if (bytes_written < 2) {
		return -1;   /* ERROR */
	}

	printf("bytes_written = %zu -> check '< 2' is false\n", bytes_written);
	return 0;        /* OK */
}

int main()
{
	printf("write() returned -1, writeCommand() returned %s\n", writeCommand() == 0 ? "OK (wrong)" : "ERROR");
	return 0;
}
