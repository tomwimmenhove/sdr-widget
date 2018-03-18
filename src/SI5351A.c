#include "Mobo_config.h"
#include "SI5351A.h"
#include "SI5351A_22MHz.h"
#include "SI5351A_24MHz.h"
#include "I2C.h"

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
}
