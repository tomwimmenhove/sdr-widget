#include "Mobo_config.h"
#include "PCM1792.h"
#include "cycle_counter.h"

uint8_t register_cache[8] =
{
		0xff, // Left channel attenuator
		0xff, // Right channel attenuator
		0xd0,//PCM1792A_ATLD_DISABLED | PCM1792A_FMT_24BI2S | PCM1792A_DMF_DISABLE | PCM1792A_DME_DISABLED | PCM1792A_MUTE_DISABLED,				// Register 18
		PCM1792A_REV_DISABLED | PCM1792A_ATS_LRCK_1 | PCM1792A_OPE_DISABLED | PCM1792A_DFMS_MONAURAL | PCM1792A_FLT_SHARP | PCM1792A_INZD_DISABLED,	// Register 19
		0,0,0,0
};

void pcm1792_write_register(uint8_t reg, uint8_t data)
{
	uint8_t d[2] = { reg, data, };
	twi_write_out(PCM1792_I2C_ADDR, d, 2);
}

uint8_t pcm1792_read_register(uint8_t reg)
{
	uint8_t val;
	twi_read_reg_in(PCM1792_I2C_ADDR, reg, (uint8_t*) &val, 1);
	return val;
}

void pcm1792_write_register_field(uint8_t reg, uint8_t mask, uint8_t value, int write_to_device)
{
	uint8_t cache = register_cache[reg - 0x10];

	cache &= ~mask;
	cache |= value;

	register_cache[reg - 0x10] = cache;

	if (write_to_device)
	{
		pcm1792_write_register(reg, cache);
	}
}

void pcm1792_set_volume_left(uint8_t volume)
{
	pcm1792_write_register(16, volume);
}

void pcm1792_set_volume_right(uint8_t volume)
{
	pcm1792_write_register(17, volume);
}

void pcm1792_set_mute(uint8_t mute)
{
	pcm1792_write_register_field(18, PCM1792A_MUTE_MASK, mute, 1);
}

void pcm1792_set_atld(uint8_t atld)
{
	pcm1792_write_register_field(18, PCM1792A_ATLD_MASK, atld, 1);
}

void pcm1792_set_fmt(uint8_t fmt)
{
	pcm1792_write_register_field(18, PCM1792A_FMT_MASK, fmt, 1);
}

void pcm1792_set_dmf(uint8_t dmf)
{
	pcm1792_write_register_field(18, PCM1792A_DMF_MASK, dmf, 1);
}

void pcm1792_set_dme(uint8_t dme)
{
	pcm1792_write_register_field(18, PCM1792A_DME_MASK, dme, 1);
}

void pcm1792_set_os(uint8_t os)
{
	pcm1792_write_register_field(20, PCM1792A_OS_MASK, os, 1);
}

