/* Reprodutor mínimo: ESBMC 8.1.0 termina com "Segmentation fault" na fase
 * "Converting". Causa: a definição de A::u usa "const float" e a declaração
 * usa "float" (C++ válido: o const de topo no parâmetro não muda o tipo da
 * função). Sem o const (ou sem a chamada em A::s), verifica normalmente.
 * Encontrado no Roboclaw.cpp do PX4 (sendUnsigned7Bit). Testar no ESBMC 8.5:
 *   esbmc esbmc_crash_const_param.cpp ; echo "codigo de saida: $?"   (139 = segfault) */
class A { public: void s(float v); void u(float d); };
void A::s(float v) { u(v); }
void A::u(const float d) { (void)d; }
int main() { A a; a.s(1.0f); return 0; }
