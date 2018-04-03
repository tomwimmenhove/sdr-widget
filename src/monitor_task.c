#ifdef FREERTOS_USED
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "print_funcs.h"
#include "monitor_task.h"
#include "usart.h"
#include "flashc.h"
#include "widget.h"
#include "wdt.h"
#include "features.h"
#include "usb_specific_request.h"
#include "device_audio_task.h"
#include "gpio.h"
#include "eeprom.h"
#include "taskPCM1792A.h"
#include "PCM1792.h"
#include "usb_drv.h"
#include "conf_usb.h"

xSemaphoreHandle line_semaphore;

static char line_buffer[LINEBUFFERSIZE][2];
static char* line = NULL;
static int line_size = -1;
static int line_buffer_n = 0;
static int line_buffer_pos = 0;

__attribute__((__interrupt__)) static void usart_int_handler(void)
{
	int x;

	if(usart_read_char(DBG_USART, &x) == 0)
	{
		char ch = (char) x;

		// Backspace
		if (ch == 8)
		{
			if (line_buffer_pos > 0)
			{
				print_dbg_char(ch);
				line_buffer_pos--;
				return;
			}
		}

		// Enter
		if (ch == '\r' || ch == '\n')
		{
			print_dbg_char('\n');

			line_size = line_buffer_pos;
			line_buffer[line_buffer_n][line_buffer_pos] = 0;
			line = line_buffer[line_buffer_n];
			line_buffer_pos = 0;
			line_buffer_n ^= 1;

			xSemaphoreGiveFromISR(line_semaphore, NULL);
			return;
		}

		if (line_buffer_pos < LINEBUFFERSIZE - 1)
		{
			print_dbg_char(ch);
			line_buffer[line_buffer_n][line_buffer_pos++] = ch;
		}
	}
}

static int read_line(char** buf)
{
	static int ret;

	while (line_size == -1)
	{
		xSemaphoreTake(line_semaphore, portMAX_DELAY);
	}

	if (buf != NULL)
	{
		*buf = line;
	}
	ret = line_size;
	line_size = -1;

	return ret;
}

static void print_warrenty();

void monitor_task_init()
{
	vSemaphoreCreateBinary(line_semaphore);

	INTC_register_interrupt(&usart_int_handler, configDBG_USART_IRQ, AVR32_INTC_INT0);
	DBG_USART->ier = AVR32_USART_IER_RXRDY_MASK;

#ifdef FREERTOS_USED
	xTaskCreate(monitor_task,
				configTSK_MONITOR_NAME,
				configTSK_MONITOR_STACK_SIZE,
				NULL,
				configTSK_MONITOR_PRIORITY,
				NULL);
	print_dbg("Monitor task created\r\n");
#endif  // FREERTOS_USED
}

static const struct variable *find_variable_entry(const struct variable *variables, int n, char *name)
{
	int i;
    for (i = 0; i < n; i++)
    {
    	const struct variable *var = &variables[i];
    	if (strcmp(var->name, name) == 0)
    	{
    		return var;
    	}
    }

    return NULL;
}

static void var_image_setter(char* value, void* data)
{

	if (strcmp(value, "uac1") == 0)
	{
		feature_set_nvram(feature_image_index, feature_image_uac1_audio);
	}
	else if (strcmp(value, "uac2") == 0)
	{
		feature_set_nvram(feature_image_index, feature_image_uac2_audio);
	}
	else
	{
		print_dbg("invalid value\r\n");
	}
}

static void var_image_getter(void* data)
{
	if (feature_get_nvram(feature_image_index) == feature_image_uac1_audio)
	{
		print_dbg("uac1\r\n");
	}
	else if (feature_get_nvram(feature_image_index) == feature_image_uac2_audio)
	{
		print_dbg("uac2\r\n");
	}
	else
	{
		print_dbg("Unknown\r\n");
	}
}

static void var_freq_getter()
{
	print_dbg_ulong(current_freq.frequency);
	print_dbg("\r\n");
}

static void var_version_getter()
{
	print_dbg_ulong(feature_get_nvram(feature_major_index));
	print_dbg_char('.');
	print_dbg_ulong(feature_get_nvram(feature_minor_index));
	print_dbg("\r\n");
}

void var_volusbl_setter(char* value, void* data)
{
	spk_vol_usb_L = strtol(value, NULL, 0);
	xSemaphoreGive(mutexVolume);
}

void var_volusbl_getter(void* data)
{
	print_dbg_short_hex(spk_vol_usb_L);
	print_dbg("\r\n");
}

void var_volusbr_setter(char* value, void* data)
{
	spk_vol_usb_R = strtol(value, NULL, 0);
	xSemaphoreGive(mutexVolume);
}

void var_volusbr_getter(void* data)
{
	print_dbg_short_hex(spk_vol_usb_R);
	print_dbg("\r\n");
}

void var_muteusb_setter(char* value, void* data)
{
	spk_mute = strtol(value, NULL, 0);
	xSemaphoreGive(mutexVolume);
}

void var_muteusb_getter(void* data)
{
	print_dbg_char_hex(spk_mute);
	print_dbg("\r\n");
}

static const struct variable variables[] =
{
		{ "version", NULL, var_version_getter },
		{ "image", var_image_setter, var_image_getter },
		{ "freq", NULL, var_freq_getter },
		{ "volusbl", var_volusbl_setter, var_volusbl_getter },
		{ "volusbr", var_volusbr_setter, var_volusbr_getter },
		{ "muteusb", var_muteusb_setter, var_muteusb_getter },
};
static int num_variables = sizeof(variables) / sizeof(struct variable);

static void display_variable(const struct variable *var, void* data)
{
	if (var->get == NULL)
	{
		print_dbg("Variable not readable\r\n");
		return;
	}
	print_dbg(var->name);
	print_dbg(": ");
	var->get(data);
}

static void set_command_handler(char **argv, int argc, void* data)
{
	if (argc != 3)
	{
		print_dbg("Invalid number of arguments given\r\n");
		return;
	}

	const struct variable *var = find_variable_entry(variables, num_variables, argv[1]);
	if (var == NULL)
	{
		print_dbg("No such variable\r\n");
		return;
	}

	if (var->set != NULL)
	{
		var->set(argv[2], data);
	}
	else
	{
		print_dbg("Variable not writable\r\n");
	}
}

static void get_command_handler(char **argv, int argc, void* data)
{
	if (argc == 1)
	{
		int i;
	    for (i = 0; i < num_variables; i++)
	    {
	    	display_variable(&variables[i], data);
	    }
		return;
	}

	if (argc != 2)
	{
		print_dbg("Invalid number of arguments given\r\n");
		return;
	}

	const struct variable *var = find_variable_entry(variables, num_variables, argv[1]);
	if (var == NULL)
	{
		print_dbg("No such variable\r\n");
		return;
	}

	display_variable(var, data);
}

static void help_command_handler(char **argv, int argc, void* data)
{
	print_dbg("help    : This\r\n");
	print_dbg("set     : Set variable\r\n");
	print_dbg("get     : Get/read a variable\r\n");
	print_dbg("reboot  : Reboot the board\r\n");
	print_dbg("dfu     : Erase flash and boot into DFU bootloader\r\n");
	print_dbg("read    : Read from memory\r\n");
	print_dbg("write   : Write to memory\r\n");
	print_dbg("dump    : Dump a bunch of memory\r\n");
	print_dbg("eeread  : Read from EEPROM\r\n");
	print_dbg("eewrite : Write to EEPROM\r\n");
	print_dbg("eedump  : Dump a bunch of EEPROM\r\n");
	print_dbg("eenull  : Write zeroes to EEPROM\r\n");
	print_dbg("ps      : List tasks\r\n");
	print_dbg("setpin  : Set a GPIO pin to a logic '1' or '0'\r\n");
	print_dbg("getpin  : Read the logic level from a GPIO pin\r\n");
	print_dbg("pcmset  : Set a register in the PCM1792A\r\n");
	print_dbg("pcmget  : Get a register from the PCM1792A\r\n");
	print_dbg("sstat   : Show sample skipping/inserting stats\r\n");
#ifdef USBHID
	print_dbg("hid     : Send HID report\r\n");
#endif
}

static void reboot_command_handler(char **argv, int argc, void* data)
{
	print_dbg("Rebooting...\r\n");
	widget_reset();
}

static void dfu_command_handler(char **argv, int argc, void* data)
{
	print_dbg("Setting fuses\r\n");
	/* Reset into Bootloader */
	flashc_erase_gp_fuse_bit(31, true);
	flashc_write_gp_fuse_bit(31, true);

	print_dbg("Rebooting into DFU in 500ms\r\n");
	widget_reset();
}

static void read_command_handler(char **argv, int argc, void* data)
{
	if (argc != 3)
	{
		print_dbg("Usage: read <bitwidth> <address>\r\n");
		return;
	}

	int bit_width = strtol(argv[1], NULL, 0);
	unsigned long int address = strtoll(argv[2], NULL, 0);

	switch (bit_width)
	{
	case 8:
	{
		uint8_t value = * ((uint8_t*)address);
		print_dbg_char_hex(value);
		break;
	}
	case 16:
	{
		uint16_t value = * ((uint16_t*)address);
		print_dbg_short_hex(value);
		break;
	}
	case 32:
	{
		uint32_t value = * ((uint32_t*)address);
		print_dbg_hex(value);
		break;
	}

	default:
		print_dbg("Incorrect bitwidth\r\n");
		break;
	}
}

static void write_command_handler(char **argv, int argc, void* data)
{
	if (argc != 4)
	{
		print_dbg("Usage: read <bitwidth> <address> <value>\r\n");
		return;
	}

	int bit_width = strtol(argv[1], NULL, 0);
	unsigned long int address = strtoll(argv[2], NULL, 0);
	unsigned long int value = strtoll(argv[3], NULL, 0);

	switch (bit_width)
	{
	case 8:
	{
		*((uint8_t*)address) = value;
		break;
	}
	case 16:
	{
		*((uint16_t*)address) = value;
		break;
	}
	case 32:
	{
		*((uint32_t*)address) = value;
		break;
	}

	default:
		print_dbg("Incorrect bitwidth\r\n");
		break;
	}
}

static void dump_hex_row(unsigned long int address, uint8_t* row, int len)
{
	print_dbg_hex(address);
	print_dbg_char(':');
	int i;
	for (i = 0; i < len; i++)
	{
		if (i % 4 == 0) print_dbg_char(' ');
		print_dbg_char_hex(row[i]);
		print_dbg_char(' ');
	}
	for (i = 0; i < len; i++)
	{
		char ch = row[i];
		print_dbg_char(isprint(ch) ? ch : '.');
	}
	print_dbg("\r\n");
}

static void dump_command_handler(char **argv, int argc, void* data)
{
	if (argc != 3)
	{
		print_dbg("Usage: dump <address> <length>\r\n");
		return;
	}

	unsigned long int start_address = strtoll(argv[1], NULL, 0);
	unsigned long int end_address = start_address + strtol(argv[2], NULL, 0);

	// Align */
	start_address &= ~0xf;
	end_address = (end_address + 0xf) & ~0xf;

	unsigned long int address;
	for (address = start_address; address < end_address; address += 16)
	{
		uint8_t *dmp_data = (uint8_t*) address;
		dump_hex_row(address, dmp_data, 16);
	}
}

static void eeread_command_handler(char **argv, int argc, void* data)
{
	if (argc != 3)
	{
		print_dbg("Usage: eeread <bitwidth> <address>\r\n");
		return;
	}

	int bit_width = strtol(argv[1], NULL, 0);
	unsigned long int address = strtoll(argv[2], NULL, 0);

	switch (bit_width)
	{
	case 8:
	{
		print_dbg_char_hex(eeprom_get8(address));
		break;
	}
	case 16:
	{
		print_dbg_short_hex(eeprom_get16(address));
		break;
	}
	case 32:
	{
		print_dbg_hex(eeprom_get32(address));
		break;
	}

	default:
		print_dbg("Incorrect bitwidth\r\n");
		break;
	}
}

static void eewrite_command_handler(char **argv, int argc, void* data)
{
	if (argc != 4)
	{
		print_dbg("Usage: eeread <bitwidth> <address> <value>\r\n");
		return;
	}

	int bit_width = strtol(argv[1], NULL, 0);
	unsigned long int address = strtoll(argv[2], NULL, 0);
	unsigned long int value = strtoll(argv[3], NULL, 0);

	switch (bit_width)
	{
	case 8:
	{
		eeprom_put8(address, value);
		break;
	}
	case 16:
	{
		eeprom_put16(address, value);
		break;
	}
	case 32:
	{
		eeprom_put32(address, value);
		break;
	}

	default:
		print_dbg("Incorrect bitwidth\r\n");
		break;
	}
}

static void eedump_command_handler(char **argv, int argc, void* data)
{
	uint8_t row[16];

	if (argc != 3)
	{
		print_dbg("Usage: eedump <address> <length>\r\n");
		return;
	}

	unsigned long int start_address = strtoll(argv[1], NULL, 0);
	unsigned long int end_address = start_address + strtol(argv[2], NULL, 0);

	// Align */
	start_address &= ~0xf;
	end_address = (end_address + 0xf) & ~0xf;

	unsigned long int address;
	for (address = start_address; address < end_address; address += sizeof(row))
	{
		eeprom_read(address, row, sizeof(row));
		dump_hex_row(address, row, sizeof(row));
	}
}

static void eenull_command_handler(char **argv, int argc, void* data)
{
	uint8_t row[EEPROM_PAGE_SIZE];

	memset(row, 0, sizeof(row));

	unsigned long int address;
	for (address = 0; address < EEPROM_SIZE; address += sizeof(row))
	{
		eeprom_write(address, row, sizeof(row));
	}
}

static void ps_command_handler(char **argv, int argc, void* data)
{
	print_dbg("Name\t\tState\tPri\tStack\tNum\r\n");
	char *buf = (char*) malloc(1024);
	vTaskList((signed char*) buf);
	print_dbg(buf);
	free(buf);
}

static void setpin_command_handler(char **argv, int argc, void* data)
{
	if (argc != 3)
	{
		print_dbg("Usage: setpin <GPIO> <logic level>\r\n");
		return;
	}

	long int gpio = strtol(argv[1], NULL, 0);
	long int level = strtol(argv[2], NULL, 0);

	if (level)
	{
		gpio_set_gpio_pin(gpio);
	}
	else
	{
		gpio_clr_gpio_pin(gpio);
	}
}

static void getpin_command_handler(char **argv, int argc, void* data)
{
	if (argc != 2)
	{
		print_dbg("Usage: setpin <GPIO>\r\n");
		return;
	}

	int level = gpio_get_pin_value(strtol(argv[1], NULL, 0));
	print_dbg("Level: ");
	print_dbg(level ? "'1'\r\n" : "'0'\r\n");
}

static void pcmset_command_handler(char **argv, int argc, void* data)
{
	if (argc != 3)
	{
		print_dbg("Usage: pcmset <register> <value>\r\n");
		return;
	}

	uint8_t reg = strtoll(argv[1], NULL, 0);
	uint8_t val = strtoll(argv[2], NULL, 0);

	pcm1792_write_register(reg, val);
}

static void pcmget_command_handler(char **argv, int argc, void* data)
{
	if (argc != 2)
	{
		print_dbg("Usage: pcmget <register>\r\n");
		return;
	}

	uint8_t reg = strtoll(argv[1], NULL, 0);
	uint8_t val = pcm1792_read_register(reg);

	print_dbg_char_hex(val);
	print_dbg("\r\n");
}

static void pcmdump_command_handler(char **argv, int argc, void* data)
{
	int i;
	for(i = 16; i <= 22; i++)
	{
		uint8_t val = pcm1792_read_register(i);

		print_dbg_ulong(i);
		print_dbg(": 0x");
		print_dbg_char_hex(val);
		print_dbg("\r\n");
	}
}

static void sstat_command_handler(char **argv, int argc, void* data)
{
	print_dbg("USB heart beat   : ");
	print_dbg_ulong(spk_usb_heart_beat);
	print_dbg("\r\n");
	print_dbg("Sample count     : ");
	print_dbg_ulong(spk_usb_sample_counter);
	print_dbg("\r\n");
	print_dbg("Samples skipped  : ");
	print_dbg_ulong(spk_usb_sample_skip);
	print_dbg("\r\n");
	print_dbg("samples inserted : ");
	print_dbg_ulong(spk_usb_sample_insert);
	print_dbg("\r\n");
	print_dbg("Current error acc: ");
	print_dbg_ulong(FB_error_acc);
	print_dbg("\r\n");
}

#ifdef USBHID
static void hid_command_handler(char **argv, int argc, void* data)
{
	if (argc != 4)
	{
		print_dbg("Usage: hid <byte0> <byte1> <byte2)\r\n");
		return;
	}

	uint8_t ReportByte0 = strtoll(argv[1], NULL, 0);
	uint8_t ReportByte1 = strtoll(argv[2], NULL, 0);
	uint8_t ReportByte2 = strtoll(argv[3], NULL, 0);

	if (Is_usb_in_ready(UAC1_EP_HID_TX))
	{
	   Usb_reset_endpoint_fifo_access(UAC1_EP_HID_TX);
	   Usb_write_endpoint_data(UAC1_EP_HID_TX, 8, ReportByte0);
	   Usb_write_endpoint_data(UAC1_EP_HID_TX, 8, ReportByte1);
	   Usb_write_endpoint_data(UAC1_EP_HID_TX, 8, ReportByte2);
	   Usb_ack_in_ready_send(UAC1_EP_HID_TX);
	}
	else
	{
		print_dbg("fail\r\n");
	}
}
#endif

static const struct menu_function *find_menu_entry(const struct menu_function *menu_functions, int n, char *name)
{
	int i;
    for (i = 0; i < n; i++)
    {
    	const struct menu_function *mf = &menu_functions[i];
    	if (strcmp(mf->command, name) == 0)
    	{
    		return mf;
    	}
    }

    return NULL;
}

static const struct menu_function menu_functions[] =
{
		{ "help", help_command_handler },
		{ "set", set_command_handler },
		{ "get", get_command_handler },
		{ "reboot", reboot_command_handler },
		{ "dfu", dfu_command_handler },
		{ "read", read_command_handler },
		{ "write", write_command_handler },
		{ "dump", dump_command_handler },
		{ "eeread", eeread_command_handler },
		{ "eewrite", eewrite_command_handler },
		{ "eedump", eedump_command_handler },
		{ "eenull", eenull_command_handler },
		{ "ps", ps_command_handler },
		{ "setpin", setpin_command_handler },
		{ "getpin", getpin_command_handler },
		{ "pcmset", pcmset_command_handler },
		{ "pcmget", pcmget_command_handler },
		{ "pcmdump", pcmdump_command_handler },
		{ "sstat", sstat_command_handler },
#ifdef USBHID
		{ "hid", hid_command_handler },
#endif
};
static int num_commands = sizeof(menu_functions) / sizeof(struct menu_function);

#ifdef FREERTOS_USED
void monitor_task(void *pvParameters)
#else
void monitor_task(void)
#endif
{
	static char *line;
	static char *saveptr;
	static char *parameters[MAX_PARAMETERS];
	static int nparams;

	print_dbg("\rMonitor task running. Press enter to activate.\r\n");
	read_line(NULL);
	print_warrenty();
	print_dbg("Console activated\r\n");

	while (1)
	{
		print_dbg("> ");
		read_line(&line);

		static char *str2, *subtoken;
        for (nparams = 0, str2 = line; ; str2 = NULL)
        {
            subtoken = strtok_r(str2, " ", &saveptr);
            if (subtoken == NULL)
                break;
            parameters[nparams++] = subtoken;
        }

        if (nparams == 0 || *parameters[0] == 0)
        {
        	continue;
        }

        const struct menu_function *mf = find_menu_entry(menu_functions, num_commands, parameters[0]);
        if (mf != NULL)
        {
    		mf->handler(parameters, nparams, NULL);
    		print_dbg("\r\n");
        }
        else
        {
        	print_dbg("Unknwon command\r\n");
        }
	}
}

static void print_warrenty()
{
	print_dbg("\r\nThis program is free software; you can redistribute it and/or modify\r\n");
	print_dbg("it under the terms of the GNU General Public License as published by\r\n");
	print_dbg("the Free Software Foundation; either version 2 of the License , or\r\n");
	print_dbg("(at your option) any later version.\r\n");
	print_dbg("\r\n");
	print_dbg("This program is distributed in the hope that it will be useful,\r\n");
	print_dbg("but WITHOUT ANY WARRANTY; without even the implied warranty of\r\n");
	print_dbg("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\r\n");
	print_dbg("GNU General Public License for more details.\r\n");
	print_dbg("\r\n");
	print_dbg("You should have received a copy of the GNU General Public License\r\n");
	print_dbg("along with this program. If not, write to\r\n");
	print_dbg("\r\n");
	print_dbg("   The Free Software Foundation, Inc.\r\n");
	print_dbg("   51 Franklin Street, Fifth Floor\r\n");
	print_dbg("   Boston, MA 02110-1301  USA\r\n\n\n");
}
