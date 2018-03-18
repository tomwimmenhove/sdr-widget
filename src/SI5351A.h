#ifndef SI5351A_H
#define SI5351A_H

#include "Mobo_config.h"

#define SI5351A_REVB_REG_CONFIG_NUM_REGS 49

#define SI5351A_ADDRESS	(uint8_t)0x60

#define SI5351A_22MHZ 1
#define SI5351A_24MHZ 2

void SI5351A_set_regs_preset(int preset);

#endif // SI5351A_H
