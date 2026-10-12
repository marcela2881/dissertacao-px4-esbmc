/**
 * @file test_common_event_buffer.cpp
 *
 * OBJETIVO: Teste de código REAL extraído de
 * src/modules/commander/HealthAndArmingChecks/Common.cpp e Common.hpp
 * FUNÇÃO TESTADA: Report::addEvent() / Report::addEventToBuffer() -
 * guarda de tamanho antes de escrever no buffer de eventos.
 * MÉTODO: Bounded Model Checking com ESBMC / CBMC
 *
 * Candidato #1 (confiança moderada): mistura de tipo sinalizado (_next_buffer_idx,
 * int) e nao-sinalizado (sizeof(...), size_t) na guarda
 *     if (total_size > sizeof(_event_buffer) - _next_buffer_idx)
 * Nos pontos de chamada reais (addEvent, addExternalEvent) essa guarda
 * roda ANTES de qualquer escrita, então o padrão normal de uso parece
 * protegido; este harness testa se existe algum estado de
 * _next_buffer_idx alcançável que quebre essa garantia.
 *
 * NOTA DE ENGENHARIA DO HARNESS: a versão anterior usava uma struct
 * EventBufferHeader com cast de ponteiro sobre o buffer, igual ao código
 * real. O CBMC (mas não o ESBMC) falhava ao tipar essa struct quando
 * usada para dimensionar/indexar o array global ("symbol
 * 'EventBufferHeader' is unknown"), uma limitação do parser e não do
 * código real. Reescrito aqui usando offsets de byte diretos, que
 * preservam exatamente a mesma propriedade (a guarda de tamanho), sem
 * depender de cast de ponteiro para struct.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern unsigned nondet_uint();
extern int nondet_int();

// ================== TAMANHO REAL DO HEADER, extraído de Common.hpp ==================
// struct __attribute__((__packed__)) EventBufferHeader {
//     uint8_t size;      // offset 0
//     uint32_t id;       // offset 1..4
//     uint8_t log_levels;// offset 5
// };
// sizeof(EventBufferHeader) com empacotamento = 1 + 4 + 1 = 6 bytes
#define EVENT_BUFFER_HEADER_SIZE 6

// Tamanho real do buffer, extraído de Common.hpp:
//   uint8_t _event_buffer[(event_s::ORB_QUEUE_LENGTH - 2) * (sizeof(EventBufferHeader) + 1 + 1 + 4)];
// ORB_QUEUE_LENGTH não está disponível neste recorte de código (definido em
// uORB/topics/event.h, gerado em build); ASSUMIMOS ORB_QUEUE_LENGTH = 8
// (valor visto em ArmingCheckReply.msg para uma fila irmã) apenas para
// dimensionar o array do harness. Este valor deve ser confirmado antes de
// qualquer conclusão definitiva.
// EVENT_BUFFER_SIZE = (8 - 2) * (6 + 1 + 1 + 4) = 6 * 12 = 72
#define EVENT_BUFFER_SIZE 168

// ================== ESTADO REAL (membros da classe Report) ==================
uint8_t _event_buffer[EVENT_BUFFER_SIZE];
int _next_buffer_idx;          // tipo real: int (Common.hpp linha 366)
bool _buffer_overflowed;

// ================== LÓGICA REAL EXTRAÍDA (addEvent, simplificada sem template) ==================
// Extraído de Common.hpp, Report::addEvent<Args...>() — guarda de tamanho:
//
//   constexpr unsigned args_size = events::util::sizeofArguments(modes, args...);
//   static_assert(args_size <= sizeof(event_s::arguments), "Too many arguments");
//   unsigned total_size = sizeof(EventBufferHeader) + args_size;
//
//   if (total_size > sizeof(_event_buffer) - _next_buffer_idx) {
//       _buffer_overflowed = true;
//       return false;
//   }
//
//   events::util::fillEventArguments(_event_buffer + _next_buffer_idx + sizeof(EventBufferHeader), modes, args...);
//   EventBufferHeader *header = addEventToBuffer(event_id, log_levels, modes, args_size);
//
// addEventToBuffer (Common.cpp):
//   unsigned total_size = sizeof(EventBufferHeader) + args_size;
//   EventBufferHeader *header = (EventBufferHeader *)(_event_buffer + _next_buffer_idx);
//   memcpy(&header->id, &event_id, sizeof(event_id));
//   header->log_levels = ...;
//   header->size = args_size;
//   _next_buffer_idx += total_size;
//
// Reescrito abaixo usando offsets de byte diretos em vez de cast de
// ponteiro para struct (ver NOTA DE ENGENHARIA DO HARNESS acima); a
// guarda de tamanho - a propriedade real sob teste - é idêntica.

bool addEvent(uint32_t event_id, unsigned args_size)
{
	unsigned total_size = EVENT_BUFFER_HEADER_SIZE + args_size;

	// Guarda real extraída de Common.hpp linha 406
	if (total_size > sizeof(_event_buffer) - _next_buffer_idx) {
		_buffer_overflowed = true;
		return false;
	}

	// Escrita real (fillEventArguments + addEventToBuffer), usando
	// offsets de byte diretos no lugar do cast para EventBufferHeader*:
	//   offset 0: size (uint8_t)
	//   offset 1..4: id (uint32_t)
	//   offset 5: log_levels (uint8_t) - omitido aqui, não afeta a propriedade
	//   offset 6..: argumentos (args_size bytes)
	memset(_event_buffer + _next_buffer_idx + EVENT_BUFFER_HEADER_SIZE, 0, args_size);

	_event_buffer[_next_buffer_idx + 0] = (uint8_t)args_size; // header->size
	memcpy(_event_buffer + _next_buffer_idx + 1, &event_id, sizeof(event_id)); // header->id

	_next_buffer_idx += total_size;
	return true;
}

int main()
{
	// _next_buffer_idx pode, em tese, chegar a qualquer estado entre
	// chamadas (estamos testando a guarda isoladamente, sem assumir que
	// só os dois call sites reais o alimentam). Restringimos a um
	// intervalo plausível para evitar VCCs triviais de overflow de int
	// puro, focando na pergunta real: a guarda falha quando
	// _next_buffer_idx já está perto do limite do buffer?
	_next_buffer_idx = nondet_int();
	__VERIFIER_assume(_next_buffer_idx >= 0 && _next_buffer_idx <= (int)(EVENT_BUFFER_SIZE + 4));

	_buffer_overflowed = false;

	unsigned args_size = nondet_uint();
	__VERIFIER_assume(args_size <= 16); // sizeof(event_s::arguments) é pequeno na prática

	uint32_t event_id = nondet_uint();

	addEvent(event_id, args_size);

	// PROPRIEDADE: nunca escrevemos além do buffer real.
	assert(_next_buffer_idx <= (int)EVENT_BUFFER_SIZE);

	return 0;
}
