#include "Mobo_config.h"
#include "eeprom.h"
#include "I2C.h"
#include "print_funcs.h"
#include "cycle_counter.h"
#include <string.h>

uint8_t eeprom_set_address(uint8_t address)
{
	return twi_write_out(EEPROM_I2C_ADDR, (uint8_t*) &address, 1);
}

uint8_t eeprom_wait_for_ack()
{
	uint8_t ret;

	// XXX: This damn twi_read_in() doesn't transmit a I2C STOP on a zero-size package!
	//while (twi_read_in(EEPROM_I2C_ADDR, NULL, 0) == (uint8_t) TWI_RECEIVE_NACK) ;
	uint8_t dummy;
	while ((ret = twi_read_in(EEPROM_I2C_ADDR, &dummy, 1)) == (uint8_t) TWI_RECEIVE_NACK) ;

	return ret;
}

uint8_t eeprom_read(uint8_t word_address, uint8_t* data, uint8_t len)
{
	eeprom_set_address(word_address);
	return twi_read_in(EEPROM_I2C_ADDR, data, len);
}

uint8_t eeprom_write(uint8_t word_address, uint8_t* data, uint8_t len)
{
	uint8_t d[len + 1];
	d[0] = word_address;
	memcpy(d + 1, data, len);

	twi_write_out(EEPROM_I2C_ADDR, d, len + 1);

	return eeprom_wait_for_ack();
}

void eeprom_test()
{
	uint32_t x32 = 0x11111111;
	//eeprom_put32(0, x32);
	eeprom_write(0x00, (uint8_t*) &x32, sizeof(x32));

	x32 = 0x22222222;
	//eeprom_put32(4, x32);
	eeprom_write(4, (uint8_t*) &x32, sizeof(x32));

	x32 = 0x33333333;
	//eeprom_put32(8, x32);
	eeprom_write(8, (uint8_t*) &x32, sizeof(x32));

	x32 = 0x44444444;
	//eeprom_put32(12, x32);
	eeprom_write(12, (uint8_t*) &x32, sizeof(x32));


	print_dbg("8: ");
	print_dbg_hex(eeprom_get32(0));
	print_dbg("\r\n");

	print_dbg("16: ");
	print_dbg_hex(eeprom_get32(4));
	print_dbg("\r\n");

	print_dbg("32: ");
	print_dbg_hex(eeprom_get32(8));
	print_dbg("\r\n");

	print_dbg("64: ");
	print_dbg_hex(eeprom_get32(12));
	print_dbg("\r\n");




	//	uint8_t x8 = 0x08;
	//	eeprom_put8(0, x8);
	//
	//	uint16_t x16 = 0x16;
	//	eeprom_put8(8, x8);
	//
	//	uint32_t x32 = 0x32;
	//	eeprom_put8(16, x32);
	//
	//	uint64_t x64 = 0x64;
	//	eeprom_put8(32, x64);
	//
	//
	//	uint8_t d1[5] = { 16, 0x01, 0x02, 0x03, 0x04, };
	//	twi_write_out(EEPROM_I2C_ADDR, d1, 5);
	//	eeprom_wait_for_ack();
	//
	//	x8 = x16 = x32 = x64 = 0;
	//
	//
	//	x8 = eeprom_get8(0);
	//	x16 = eeprom_get16(8);
	//	x32 = eeprom_get32(16);
	//	x64 = eeprom_get64(32);
	//
	//
	//	print_dbg("8: ");
	//	print_dbg_char_hex(x8);
	//	print_dbg("\r\n");
	//
	//	print_dbg("16: ");
	//	print_dbg_short_hex(x16);
	//	print_dbg("\r\n");
	//
	//	print_dbg("32: ");
	//	print_dbg_hex(x32);
	//	print_dbg("\r\n");
	//
	//	print_dbg("64: ");
	//	print_dbg_hex(x64 >> 32);
	//	print_dbg_hex(x64 & 0xffffffff);
	//	print_dbg("\r\n");

	//return;
	int x;
	uint8_t success;

	gpio_set_gpio_pin(AVR32_PIN_PB00);

	/* Write */
//	uint8_t d1[5] = { 0x00, 0x01, 0x02, 0x03, 0x04, };
//	twi_write_out(EEPROM_I2C_ADDR, d1, 5);
//	eeprom_wait_for_ack();
//
//	uint8_t d2[5] = { 0x02, 0x04, 0x03, 0x02, 0x01, };
//	twi_write_out(EEPROM_I2C_ADDR, d2, 5);
//	eeprom_wait_for_ack();

	x = 0x12345678;
	eeprom_write(0x00, (uint8_t*) &x, sizeof(x));
	x = 0x87654321;
	eeprom_write(0x04, (uint8_t*) &x, sizeof(x));
	x = 0x12121212;
	eeprom_write(0x08, (uint8_t*) &x, sizeof(x));

	x=42;
	success = eeprom_read(0x00, (uint8_t*) &x, sizeof(x));
	print_dbg("SUCCESS: ");
	print_dbg_char_hex(success);
	print_dbg(": ");
	print_dbg_hex(x);
	print_dbg("\r\n");

	x=42;
	success = eeprom_read(0x04, (uint8_t*) &x, sizeof(x));
	print_dbg("SUCCESS: ");
	print_dbg_char_hex(success);
	print_dbg(": ");
	print_dbg_hex(x);
	print_dbg("\r\n");

	x=42;
	success = eeprom_read(0x08, (uint8_t*) &x, sizeof(x));
	print_dbg("SUCCESS: ");
	print_dbg_char_hex(success);
	print_dbg(": ");
	print_dbg_hex(x);
	print_dbg("\r\n");



	print_dbg("16: ");
	print_dbg_hex(eeprom_get32(0));
	print_dbg("\r\n");

	print_dbg("32: ");
	print_dbg_hex(eeprom_get32(4));
	print_dbg("\r\n");

	print_dbg("64: ");
	print_dbg_hex(eeprom_get32(8));
	print_dbg("\r\n");


	return;
	gpio_tgl_gpio_pin(AVR32_PIN_PB00);

	eeprom_wait_for_ack();
	gpio_tgl_gpio_pin(AVR32_PIN_PB00);

	//x = 0x1234;
	//eeprom_write(0x02, (uint8_t*) &x, sizeof(int));


	//return;

	x = 0xaaaa;

	//eeprom_set_address(0x02);
	eeprom_set_address(0x00);
	gpio_tgl_gpio_pin(AVR32_PIN_PB00);
//	xSemaphoreTake( mutexI2C, portMAX_DELAY );
//	gpio_tgl_gpio_pin(AVR32_PIN_PB00);
//
//	twi_package_t packet =
//	{
//		.chip = EEPROM_I2C_ADDR,
//		.addr = 2,
//		//.addr_length = 0,
//		.addr_length = 1,
//		.buffer = &x,
//		.length = sizeof(x)
//	};
//
//	// Returns TWI_SUCCESS (0) on successful read
//	success = twi_master_read(MOBO_TWI, &packet);
//
//	gpio_tgl_gpio_pin(AVR32_PIN_PB00);
//
//	// Release I2C port
//	xSemaphoreGive( mutexI2C );

	success = twi_read_in(EEPROM_I2C_ADDR, (uint8_t*) &x, sizeof(x));
	//success = twim_read(twi, package->buffer, package->length,package->chip, 0);

	print_dbg("SUCCESS: ");
	print_dbg_char_hex(success);
	print_dbg(": ");
	print_dbg_hex(x);
	print_dbg("\r\n");


	gpio_clr_gpio_pin(AVR32_PIN_PB00);

//	eeprom_set_address(0x02);
//	success = twi_read_in(EEPROM_I2C_ADDR, (uint8_t*) &x, sizeof(x));
//
//	print_dbg("SUCCESS: ");
//	print_dbg_char_hex(success);
//	print_dbg(": ");
//	print_dbg_hex(x);
//	print_dbg("\r\n");
}
