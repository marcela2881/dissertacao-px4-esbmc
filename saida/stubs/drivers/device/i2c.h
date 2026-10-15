/*
 * STUB de verificação — substitui src/lib/drivers/device/i2c.h do PX4.
 * Modela apenas o que o driver PCA9685 usa: construtor, init(), probe() e transfer().
 * transfer() devolve dados não determinísticos (hardware fora do modelo) e
 * guarda uma cópia do último buffer enviado para que o harness possa checar
 * propriedades derivadas do datasheet sobre os bytes escritos no chip.
 */
#pragma once
#include <cstdint>
#include <cstddef>

#ifndef DRV_PWM_DEVTYPE_PCA9685
#define DRV_PWM_DEVTYPE_PCA9685 0x9A   /* valor irrelevante para a verificação */
#endif
#ifndef MODULE_NAME
#define MODULE_NAME "pca9685_pwm_out"
#endif

extern "C" int nondet_int();
extern "C" uint8_t nondet_uint8();

/* cópia do último envio (65 = 16 canais * 4 registradores + 1 byte de endereço) */
#define STUB_I2C_MAX_SEND 65
extern uint8_t  g_last_send[STUB_I2C_MAX_SEND];
extern unsigned g_last_send_len;

namespace device
{
class I2C
{
public:
	I2C(uint8_t devtype, const char *name, int bus, uint16_t address, uint32_t frequency) {}
	virtual ~I2C() = default;
	virtual int init() { return nondet_int(); }
	uint32_t get_device_id() const { return 0; }
	int get_device_bus() const { return 1; }
	int get_device_address() const { return 0x40; }

protected:
	virtual int probe() { return 0; }

	int transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
	{
		/* lê todos os bytes enviados: o verificador checa os limites do buffer do chamador */
		g_last_send_len = send_len;
		for (unsigned i = 0; i < send_len; i++) {
			uint8_t b = send[i];
			if (i < STUB_I2C_MAX_SEND) { g_last_send[i] = b; }
		}
		/* escreve dados arbitrários na resposta (hardware não modelado) */
		for (unsigned i = 0; i < recv_len; i++) { recv[i] = nondet_uint8(); }
		return nondet_int();   /* 0 = sucesso, outro valor = falha de barramento */
	}
};
}
