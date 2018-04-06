#include "Mobo_config.h"
#include "eeprom.h"
#include "I2C.h"
#include "print_funcs.h"
#include "cycle_counter.h"
#include <string.h>

uint8_t eeprom_set_address(uint16_t word_address)
{
	uint8_t lsb = word_address & 0xff;
	return twi_write_out(EEPROM_I2C_GET_ADDR(word_address), (uint8_t*) &lsb, 1);
}

uint8_t eeprom_wait_for_ack(uint16_t word_address)
{
	uint8_t ret;

	// XXX: This damn twi_read_in() doesn't transmit a I2C STOP on a zero-size package!
	//while (twi_read_in(EEPROM_I2C_ADDR, NULL, 0) == (uint8_t) TWI_RECEIVE_NACK) ;
	uint8_t dummy;
	while ((ret = twi_read_in(EEPROM_I2C_GET_ADDR(word_address), &dummy, 1)) == (uint8_t) TWI_RECEIVE_NACK) ;

	return ret;
}

uint8_t eeprom_read(uint16_t word_address, uint8_t* data, uint8_t len)
{
	eeprom_set_address(word_address);
	return twi_read_in(EEPROM_I2C_GET_ADDR(word_address), data, len);
}

uint8_t eeprom_write_page(uint16_t word_address, uint8_t* data, uint8_t len)
{
	uint8_t d[len + 1];
	d[0] = word_address & 0xff;
	memcpy(d + 1, data, len);

	twi_write_out(EEPROM_I2C_GET_ADDR(word_address), d, len + 1);

	return eeprom_wait_for_ack(word_address);
}

uint8_t eeprom_write(uint16_t word_address, uint8_t* data, uint8_t len)
{
    while (len > 0)
    {
    	int l = len > EEPROM_PAGE_SIZE ? EEPROM_PAGE_SIZE : len;

    	uint8_t ret = eeprom_write_page(word_address, data, l);
    	if (ret != 0)
    	{
    		return ret;
    	}

    	len -= l;
    	word_address += l;
    	data += l;
    }

    return 0;
}
