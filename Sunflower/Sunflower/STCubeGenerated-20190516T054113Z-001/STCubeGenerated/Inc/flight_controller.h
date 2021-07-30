#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include <TJ_MPU6050.h>
#include <dwt_stm32_delay.h>
#include <math.h>

void turnOnRedLED(void);
void turnOnBlueLED(void);

#endif