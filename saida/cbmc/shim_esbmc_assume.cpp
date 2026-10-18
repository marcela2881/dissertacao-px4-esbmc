/*
 * Shim para o CBMC: os harnesses da 1ª leva usam __ESBMC_assume (função do
 * ESBMC). No CBMC ela vira __CPROVER_assume, com o MESMO significado
 * (pré-condição). Assim os harnesses do artigo são usados sem alteração.
 */
void __ESBMC_assume(int condition) { __CPROVER_assume(condition != 0); }
