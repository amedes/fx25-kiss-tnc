#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern int cap_queue_err;

void mcpwm_initialize(xQueueHandle queue);
