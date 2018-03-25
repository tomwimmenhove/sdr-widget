#ifdef FREERTOS_USED
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif

#include <stdio.h>
#include <string.h>

#include "print_funcs.h"
#include "monitor_task.h"
#include "usart.h"
#include "flashc.h"
#include "widget.h"
#include "wdt.h"
#include "features.h"
#include "usb_specific_request.h"

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
		xSemaphoreTake(line_semaphore, 999999999);
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
#endif  // FREERTOS_USED

	print_dbg("Monitor ready\r\n");
}

static const struct variable *find_variable_entry(const struct variable *variables, int n, char *name)
{
	static int i;
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
	printf("%ld\r\n", current_freq.frequency);
}

static void var_version_getter()
{
	printf("%d.%d\r\n", feature_get_nvram(feature_major_index), feature_get_nvram(feature_minor_index));
}

static const struct variable variables[] =
{
		{ "image", var_image_setter, var_image_getter },
		{ "freq", NULL, var_freq_getter },
		{ "version", NULL, var_version_getter },
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
		static int i;
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
	print_dbg("\"help\"		:	This\r\n");
	print_dbg("\"set\"		:	Set variable\r\n");
	print_dbg("\"get\"		:	Get/read a variable\r\n");
	print_dbg("\"reboot\"	:	Reboot the board\r\n");
	print_dbg("\"dfu\")		:	Erase flash and boot into DFU bootloader\r\n");
}

static void reboot_command_handler(char **argv, int argc, void* data)
{
	print_dbg("Rebooting...\r\n");
	widget_reset();
}

static void dfu_command_handler(char **argv, int argc, void* data)
{
	print_dbg("Erasing flash\r\n");
	/* Reset into Bootloader */
	flashc_erase_gp_fuse_bit(31, true);
	flashc_write_gp_fuse_bit(31, true);

	print_dbg("Disabling interrupts\r\n");
	DISABLE_ALL_INTERRUPTS();

	print_dbg("Rebooting into DFU in 500ms\r\n");
	wdt_enable(500000);
	while (1);				// Wait for it to fire
}

static const struct menu_function *find_menu_entry(const struct menu_function *menu_functions, int n, char *name)
{
	static int i;
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
        	print_dbg("Unknwon comman\r\n");
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
