#ifndef EEPROM_H
#define EEPROM_H

#define EEPROM_I2C_ADDR				0x50
#define EEPROM_I2C_ADDR_GET_MSB(x)	(((x) >> 8) & 0x03)
#define EEPROM_I2C_GET_ADDR(x)		((EEPROM_I2C_ADDR) | (EEPROM_I2C_ADDR_GET_MSB(x)))

#define EEPROM_SIZE					256
#define EEPROM_PAGE_SIZE			8

uint8_t eeprom_read(uint16_t word_address, uint8_t* data, uint8_t len);
uint8_t eeprom_write_page(uint16_t word_address, uint8_t* data, uint8_t len);
uint8_t eeprom_write(uint16_t word_address, uint8_t* data, uint8_t len);
void eeprom_test();

#define EEPROM_GET_TYPE(name, t) static inline t name(uint16_t word_address)	\
{																			\
	t x;																	\
	eeprom_read(word_address, (uint8_t*) &x, sizeof(t));					\
	return x;																\
}

EEPROM_GET_TYPE(eeprom_get8, uint8_t)
EEPROM_GET_TYPE(eeprom_get16, uint16_t)
EEPROM_GET_TYPE(eeprom_get32, uint32_t)
EEPROM_GET_TYPE(eeprom_get64, uint64_t)

static inline void eeprom_put8(uint16_t word_address, uint8_t x) { eeprom_write(word_address, (uint8_t*) &x, sizeof(x)); }
static inline void eeprom_put16(uint16_t word_address, uint16_t x) { eeprom_write(word_address, (uint8_t*) &x, sizeof(x)); }
static inline void eeprom_put32(uint16_t word_address, uint32_t x) { eeprom_write(word_address, (uint8_t*) &x, sizeof(x)); }
static inline void eeprom_put64(uint16_t word_address, uint64_t x) { eeprom_write(word_address, (uint8_t*) &x, sizeof(x)); }


#endif // EEPROM_H
