// replay.cpp (candidato 3) — replay do candidato 3 (ft7_technologies.cpp) que
// CHAMA de fato o laço de collect(). Substitui replay_ft7.cpp, cujo main()
// reimplementava o laço e nunca chamava collect_call().
//
// Corpo do laço copiado de src/drivers/wind_sensor/ft_technologies/
// ft7_technologies.cpp no commit 8be2590c15 (linhas 158-234). Ramo '\n' sem a
// publicação uORB/checksum (não altera índices). Membros na MESMA ORDEM do .hpp
// (linhas 78-96), para que o estouro caia nos mesmos vizinhos do objeto real.
// Compilar com -DFIXED aplica as guardas do commit upstream 3acf7935a0 (#28600).
//
// Como a escrita fora do limite fica DENTRO do objeto, o ASan não a vê
// (mesmo caso do sumd); quem detecta é o UBSan (-fsanitize=bounds).
#include <cstdio>
#include <cstring>
#include <cstdint>

struct Ft7 {
	char _port[20] {};
	char _readbuf[30] {};
	uint64_t _interval{100000};
	bool _collect_phase{false};
	int _fd{-1};
	char _linebuf[20] {};
	uint64_t _last_read{0};
	uint64_t _last_measure{0};
	char _raw_angle[5];
	char _raw_speed[5];
	char _raw_status[2];
	char _raw_checksum[3];
	uint16_t _checksum{0};
	uint32_t _hex_checksum{0};
	int _msg_part_counter{0};
	int _byte_counter{0};
	int _msg_byte_counter{0};
	uint32_t _checksum_counter{0};

	void collect_call(char *readbuf, int ret)
	{
		for (int i = 0; i < ret; i++) {
#ifdef FIXED
			if (_byte_counter < (int)sizeof(_readbuf)) {
				_readbuf[_byte_counter] = readbuf[i];
				_byte_counter += 1;
			}
#else
			_readbuf[_byte_counter]  = readbuf[i];
			_byte_counter += 1;
#endif
			if (readbuf[i] == '\n') {
				_msg_part_counter = 0;
				_byte_counter = 0;
				_msg_byte_counter = 0;
				_checksum_counter = 0;
				memset(_raw_speed, 0, 5);
				memset(_raw_angle, 0, 5);
				memset(_raw_status, 0, 2);
				memset(_raw_checksum, 0, 3);

			} else if (readbuf[i] == '$' || readbuf[i] == ',' || readbuf[i] == '=' || readbuf[i] == '*' || readbuf[i] == '\r') {
				_msg_part_counter += 1;
				_msg_byte_counter = 0;

			} else {
				if (readbuf[i] != '.') {
#ifdef FIXED
					if (_msg_part_counter == 3) { if (_msg_byte_counter < (int)sizeof(_raw_speed) - 1) { _raw_speed[_msg_byte_counter] = readbuf[i]; } }
					else if (_msg_part_counter == 4) { if (_msg_byte_counter < (int)sizeof(_raw_angle) - 1) { _raw_angle[_msg_byte_counter] = readbuf[i]; } }
					else if (_msg_part_counter == 5) { if (_msg_byte_counter < (int)sizeof(_raw_status) - 1) { _raw_status[_msg_byte_counter] = readbuf[i]; } }
					else if (_msg_part_counter == 6) { _checksum_counter += 1; if (_msg_byte_counter < (int)sizeof(_raw_checksum) - 1) { _raw_checksum[_msg_byte_counter] = readbuf[i]; } }
#else
					if (_msg_part_counter == 3) { _raw_speed[_msg_byte_counter] = readbuf[i]; }
					else if (_msg_part_counter == 4) { _raw_angle[_msg_byte_counter] = readbuf[i]; }
					else if (_msg_part_counter == 5) { _raw_status[_msg_byte_counter] = readbuf[i]; }
					else if (_msg_part_counter == 6) { _checksum_counter += 1; _raw_checksum[_msg_byte_counter] = readbuf[i]; }
#endif
					_msg_byte_counter += 1;
				}
			}
		}
	}
};

int main(int argc, char **argv)
{
	// readbuf local de collect(): char readbuf[sizeof(_linebuf)]; readlen = sizeof(readbuf) - 1 = 19
	char readbuf[20];
	int cenario = argc > 1 ? argv[1][0] - '0' : 1;
	setvbuf(stdout, NULL, _IONBF, 0);
	// objeto estatico: sem new/delete (parecer C7) e sem vazamento no LeakSanitizer
	static Ft7 dev_obj;
	Ft7 *dev = &dev_obj;

	if (cenario == 1) {
		// Cenário 1: duas leituras de 19 bytes sem '\n' (19+19 = 38 > 30)
		memset(readbuf, 'X', 19);
		dev->collect_call(readbuf, 19);
		printf("apos 1a chamada: _byte_counter=%d\n", dev->_byte_counter);
		dev->collect_call(readbuf, 19);
		printf("apos 2a chamada: _byte_counter=%d (sizeof _readbuf = 30)\n", dev->_byte_counter);

	} else {
		// Cenário 2: "$A,WVP=" seguido de dígitos -> campo speed sem delimitador
		const char *msg = "$A,WVP=1234567890";
		int n = (int)strlen(msg);
		memcpy(readbuf, msg, n);
		dev->collect_call(readbuf, n);
		printf("_msg_part_counter=%d _msg_byte_counter=%d (sizeof _raw_speed = 5)\n",
		       dev->_msg_part_counter, dev->_msg_byte_counter);
	}

	printf("_interval=%llu (inicial 100000)\n", (unsigned long long)dev->_interval);

	return 0;
}
