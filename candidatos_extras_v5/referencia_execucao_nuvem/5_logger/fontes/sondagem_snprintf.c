/* sondagem_snprintf.c — parecer A7, secao 5.4
 *
 * Testa se o modelo de snprintf do verificador devolve o comprimento certo
 * quando o argumento %s e uma string montada em tempo de execucao.
 * Na libc real, r == 10.
 *
 *   esbmc sondagem_snprintf.c
 *   cbmc  sondagem_snprintf.c
 *   cc sondagem_snprintf.c -o s && ./s
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
	char b[1200];
	char name[11];

	for (int i = 0; i < 10; i++) {
		name[i] = 'A';
	}
	name[10] = '\0';

	assert(strlen(name) == 10);

	int r = snprintf(b, sizeof(b), "%s", name);
	printf("snprintf devolveu %d\n", r);
	assert(r == (int)strlen(name));
	return 0;
}
