#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include <TJ_MPU6050.h>
#include <dwt_stm32_delay.h>
#include <math.h>

void turnOnRedLED(char* command);
void turnOnBlueLED(char* command);
void startPWM(TIM_HandleTypeDef htim1);
void stopPWM(TIM_HandleTypeDef htim1);
void startBlueTooth(char rx_buffer[20], char tx_buffer[20], char status[20], UART_HandleTypeDef huart1);
void setMotorSpeed(int dutyCycle, TIM_HandleTypeDef htim1);

#endif
