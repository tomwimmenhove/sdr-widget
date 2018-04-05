#ifndef SI5351A_H
#define SI5351A_H

#include "Mobo_config.h"

#define SI5351A_REVB_REG_CONFIG_NUM_REGS 49

#define SI5351A_ADDRESS	(uint8_t)0x60

#define SI5351A_22MHZ 1
#define SI5351A_24MHZ 2

#define SI5351A_XTAL_CL_REG	183
#define SI5351A_XTAL_CL0PF	0
#define SI5351A_XTAL_CL6PF	1
#define SI5351A_XTAL_CL8PF	2
#define SI5351A_XTAL_CL10PF	3 // Default

void SI5351A_set_regs_preset(int preset);
void SI5351A_write_register(uint8_t reg, uint8_t data);
uint8_t SI5351A_read_register(uint8_t reg);

#endif // SI5351A_H
