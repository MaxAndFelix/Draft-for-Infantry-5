#ifndef BSP_LED_H
#define BSP_LED_H

#include "main.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#define RGB_FLOW_COLOR_CHANGE_TIME 1000
#define RGB_FLOW_COLOR_LENGHT 6

unsigned int RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

extern void LED_RGB_task(void const *argument);

#endif