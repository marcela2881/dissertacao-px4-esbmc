/* Reprodutor mínimo: ESBMC 8.1.0 não converte literal definido pelo usuário:
 * "ERROR: Conversion of unsupported clang expr: "UserDefinedLiteral"".
 * Encontrado no Roboclaw.cpp do PX4 (TIMEOUT_US = 11_ms). Testar no ESBMC 8.5:
 *   esbmc esbmc_udl.cpp */
constexpr unsigned long long operator""_ms(unsigned long long v) { return v * 1000ULL; }
int main() { static constexpr int T = 11_ms; return T == 11000 ? 0 : 1; }
