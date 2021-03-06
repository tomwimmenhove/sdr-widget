#include "Mobo_config.h"
#include "SI5351A.h"
#include "SI5351A_22MHz.h"
#include "SI5351A_24MHz.h"
#include "I2C.h"
#include "eeprom.h"
#include "features.h"

void SI5351A_write_register(uint8_t reg, uint8_t data)
{
	uint8_t d[2] = { reg, data, };
	twi_write_out(SI5351A_ADDRESS, d, 2);
}

uint8_t SI5351A_read_register(uint8_t reg)
{
	uint8_t val;
	twi_read_reg_in(SI5351A_ADDRESS, reg, (uint8_t*) &val, 1);
	return val;
}

static void write_registers(const uint8_t regs[SI5351A_REVB_REG_CONFIG_NUM_REGS][2])
{
	int i;
	for (i=0;i<SI5351A_REVB_REG_CONFIG_NUM_REGS;i++)
	{
		twi_write_out(SI5351A_ADDRESS, (uint8_t*) regs[i], 2);
	}
}

void SI5351A_set_regs_preset(int preset)
{
	switch (preset)
	{
	case SI5351A_22MHZ:
		write_registers(si5351a_revb_registers_22MHz);
		break;
	case SI5351A_24MHZ:
		write_registers(si5351a_revb_registers_24MHz);
		break;
	}

	//SI5351A_write_register(SI5351A_XTAL_CL_REG, FEATURE_XTAL_CL);
	switch (FEATURE_XTAL_CL)
	{
	case feature_xtal_cl_0pf:
		SI5351A_write_register(SI5351A_XTAL_CL_REG, SI5351A_XTAL_CL0PF);
		break;
	case feature_xtal_cl_6pf:
		SI5351A_write_register(SI5351A_XTAL_CL_REG, SI5351A_XTAL_CL6PF);
		break;
	case feature_xtal_cl_8pf:
		SI5351A_write_register(SI5351A_XTAL_CL_REG, SI5351A_XTAL_CL8PF);
		break;
	case feature_xtal_cl_10pf:
		SI5351A_write_register(SI5351A_XTAL_CL_REG, SI5351A_XTAL_CL10PF);
		break;
	}
}
