/*
 * harness_roboclaw_bug2.cpp
 * RoboClaw — Bug 2: erro de write() não é detectado.
 *
 * MODULO PX4: src/drivers/roboclaw/Roboclaw.cpp, writeCommandWithPayload()
 *   size_t bytes_written = write(_uart_fd, buffer, packet_size);
 *   if (bytes_written < packet_size) { ... return ERROR; }
 *
 * write() pode retornar -1 (erro, ex.: EIO). Guardado em size_t, o -1 vira um
 * número gigante (ULONG_MAX), então "bytes_written < packet_size" é FALSO e a
 * função retorna OK mesmo tendo falhado. O chamador então espera uma resposta
 * que nunca virá, e só percebe o problema por timeout no select().
 *
 * Propriedade verificada: se write() falhou (retorno < 0), a função TEM que
 * reportar erro (-1).
 *
 * Compilar com -DCORRECAO para guardar o retorno em ssize_t (com sinal) e
 * checar < 0 antes do tamanho, como sugerido na issue.
 *
 *   esbmc harness_roboclaw_bug2.cpp --overflow-check
 *   esbmc harness_roboclaw_bug2.cpp -DCORRECAO --overflow-check
 */
#include <assert.h>

typedef unsigned long size_t_m;   /* modelo de size_t  (64 bits) */
typedef long          ssize_t_m;  /* modelo de ssize_t (64 bits) */

extern long nondet_long(void);    /* retorno simbólico de write() */

int main(void)
{
	size_t_m packet_size = 5;          /* endereço + comando + payload + CRC(2) */
	long ret = nondet_long();          /* write() pode devolver -1 ou 0..packet_size */

#ifdef CORRECAO
	ssize_t_m bytes_written = (ssize_t_m)ret;             /* guarda COM sinal */
	int result = (bytes_written < 0 ||
	              (size_t_m)bytes_written < packet_size) ? -1 : 0;
#else
	size_t_m bytes_written = (size_t_m)ret;               /* código real: SEM sinal */
	int result = (bytes_written < packet_size) ? -1 : 0;  /* -1 vira ULONG_MAX */
#endif

	/* Se o write() falhou, o resultado tem que ser erro (-1). */
	if (ret < 0) {
		assert(result == -1);
	}
	return 0;
}
