/*
 * Erro do ESBMC 8.5.0 (encontrado em 19/10/2026 no harness do mixer_module):
 * construir uma classe POLIMÓRFICA que tem um membro VETOR com inicializador "{}"
 * faz o ESBMC abortar antes da verificação:
 *     ERROR: uncaught exception [St19bad_optional_access]: bad optional access
 * Sem a função virtual, ou sem o "{}" no vetor, o mesmo código verifica normalmente.
 *
 *   esbmc esbmc_crash_vetor_polimorfico.cpp --function harness_main
 */
class B { public: virtual ~B() = default; virtual void u() {} };
class M : public B { public: void u() override {} unsigned short a[4] {}; };
int harness_main() { M m; m.u(); return m.a[1]; }
