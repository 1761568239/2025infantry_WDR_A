#ifndef SUPERCAP_H
#define SUPERCAP_H

#include "main.h"
#include "CAN_receive.h"
#include "cmsis_os.h"

#define supercap_threshold  12.5f
#define POWER_ADD        	5.0f
#define POWER_INIT       	45.0f

extern void supercap_task(void const *pvParameters);

#endif
