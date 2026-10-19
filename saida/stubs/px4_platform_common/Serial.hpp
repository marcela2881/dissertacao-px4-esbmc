/*
 * STUB de px4_platform_common/Serial.hpp (device::Serial). A UART do outro lado
 * é arbitrária: read() devolve de -1 até o tamanho pedido, com bytes quaisquer.
 * O harness pode fixar a resposta (g_serial_fixed / g_serial_bytes) para os
 * testes de especificação.
 */
#pragma once
#include <px4_platform_common/px4_common_stub.h>

extern "C" { long nondet_long(); unsigned char nondet_uint8(); void __VERIFIER_assume(int); }

extern bool          g_serial_fixed;        /* true: read() devolve g_serial_bytes */
extern unsigned char g_serial_bytes[64];
extern long          g_serial_len;

namespace device
{
class Serial
{
public:
	bool setPort(const char *) { return true; }
	bool setBaudrate(unsigned) { return true; }
	bool setSwapRxTxMode() { return true; }
	bool open() { return true; }
	bool close() { return true; }

	long read(unsigned char *buf, unsigned long n)
	{
		if (g_serial_fixed) {
			long r = g_serial_len;
			for (long i = 0; i < r; i++) { buf[i] = g_serial_bytes[i]; }
			return r;
		}

		long r = nondet_long();
		__VERIFIER_assume(r >= -1 && r <= (long)n);
		for (long i = 0; i < r; i++) { buf[i] = nondet_uint8(); }
		return r;
	}
};
}
