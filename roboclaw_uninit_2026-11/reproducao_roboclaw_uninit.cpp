// reproducao_roboclaw_uninit.cpp
//
// Reconstrucao do padrao de uso de memoria nao inicializada do driver RoboClaw
// (PX4, commit 8be2590c15, src/drivers/roboclaw/Roboclaw.hpp e Roboclaw.cpp).
//
// O que e copiado do driver, sem alteracao de logica:
//   - as declaracoes dos membros, na mesma ordem e SEM inicializador
//         bool _uart_initialized{false};  int _uart_fd{0};
//         fd_set _uart_fd_set;  struct timeval _uart_fd_timeout;          (Roboclaw.hpp, linhas 133-136)
//   - a chamada de readAcknowledgement()                                    (Roboclaw.cpp, linha 346)
//         select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &_uart_fd_timeout)
//   - a ordem de Run(): o mixer atualiza as saidas ANTES de initializeUART() (Roboclaw.cpp, linhas 166-181)
//   - initializeUART() so define timeout e fd_set depois                    (Roboclaw.cpp, linhas 64-69 e 141-142)
//   - o construtor e fornecido pelo usuario e nao inicializa esses membros  (Roboclaw.cpp, linha 49)
//
// O que NAO e o driver: nao ha PX4, mixer nem UART. Este programa mostra o PADRAO (leitura de
// fd_set e timeval indeterminados na primeira chamada de select), e nao a execucao do driver real.
//
// Compilar com -DCORRECAO para a variante com as duas linhas corrigidas: fd_set _uart_fd_set{}; struct timeval _uart_fd_timeout{};
#include <sys/select.h>
#include <sys/time.h>
#include <cstdio>
#include <cstring>
#include <new>

class RoboclawLike
{
public:
	RoboclawLike() {}   // como Roboclaw::Roboclaw(): nao inicializa _uart_fd_set nem _uart_fd_timeout

	void Run()
	{
		updateOutputs();                 // equivalente a _mixing_output.update(): chama o driver antes da UART
		if (!_uart_initialized) {
			initializeUART();
			_uart_initialized = true;
		}
	}

private:
	void updateOutputs()
	{
		// no driver: updateOutputs -> sendUnsigned7Bit -> writeCommandWithPayload -> readAcknowledgement
		(void)readAcknowledgement();
	}

	int readAcknowledgement()
	{
		int select_status = select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &_uart_fd_timeout);
		std::printf("select_status=%d\n", select_status);
		return select_status;
	}

	void initializeUART()
	{
		static constexpr int TIMEOUT_US = 11000;                  // 11_ms
		_uart_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };
		FD_ZERO(&_uart_fd_set);
		FD_SET(_uart_fd, &_uart_fd_set);
	}

	bool _uart_initialized{false};
	int _uart_fd{0};
#ifdef CORRECAO
	fd_set _uart_fd_set{};
	struct timeval _uart_fd_timeout{};
#else
	fd_set _uart_fd_set;
	struct timeval _uart_fd_timeout;
#endif
};

int main()
{
	// memoria dinamica sem preencher, como o driver (criado com new, construtor do usuario)
	void *mem = ::operator new(sizeof(RoboclawLike));
	RoboclawLike *r = new (mem) RoboclawLike();
	r->Run();      // primeira chamada de Run(): select com fd_set e timeval indeterminados
	r->Run();      // segunda: ja inicializados por initializeUART()
	r->~RoboclawLike();
	::operator delete(mem);
	return 0;
}
