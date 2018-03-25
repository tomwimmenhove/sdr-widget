#ifndef MONITOR_TASK_H
#define MONITOR_TASK_H

#define LINEBUFFERSIZE	128 // This includes the terminating zero.

void monitor_task_init();

#ifdef FREERTOS_USED
void monitor_task(void *pvParameters);
#else
void monitor_task(void);
#endif

#define MAX_PARAMETERS 16

struct menu_function
{
	const char* command;
	void (*handler)(char **argv, int argc, void* data);
};

struct variable
{
	const char* name;
	void (*set)(char* value, void* data);
	void (*get)(void* data);
};

#endif // #define MONITOR_TASK_H
