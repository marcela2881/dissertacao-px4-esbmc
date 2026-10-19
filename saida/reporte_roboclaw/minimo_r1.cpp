/*
 * Reprodução MÍNIMA do R1, para anexar à issue (não precisa do PX4).
 * Reproduz só a aritmética do caminho de saída do driver, linha a linha:
 *   Roboclaw.cpp:161  setMotorSpeed(Motor::Right, (outputs[0] - 127.0f) / 127.f);
 *   Roboclaw.cpp:291  uint8_t byte = static_cast<uint8_t>(lroundf(fabs(data) * INT8_MAX));
 * Os valores de entrada são os que o MixingOutput entrega em "termination"
 * (failsafe, sem limitação): RBCLW_FAILn pode ir até 257 (module.yaml).
 * O replay com o arquivo ORIGINAL está em ../replay/replay_roboclaw.cpp.
 * Compilar: g++ -std=c++17 minimo_r1.cpp -o minimo_r1 && ./minimo_r1
 */
#include <cmath>
#include <cstdint>
#include <cstdio>

int main()
{
	const float outputs[] = {0.f, 127.f, 254.f, 255.f, 256.f, 257.f};

	printf("output  data      byte  (Packet Serial cmds 0/1/4/5 accept 0..127)\n");

	for (float out : outputs) {
		float data = (out - 127.0f) / 127.f;                                   /* :161 */
		uint8_t byte = static_cast<uint8_t>(lroundf(fabs(data) * INT8_MAX));   /* :291 */
		printf("%6.1f  %+.5f  %4u  %s\n", out, data, byte, byte > 127 ? "OUT OF RANGE" : "ok");
	}

	return 0;
}
