#include <TJ_MPU6050.h>
#include <dwt_stm32_delay.h>
#include <math.h>
#include "stm32f4xx_hal.h"

void turnOnRedLED() {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

void turnOnBlueLED() {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
}

/*
void startUp() {

}

void turnOff() {

}

void sendBlueToothMessage() {

}

void calibrateESCs() {

}

double calcAccDisplacement() {

}

void calcGyroDisplacement() {

}

void correctDrift() {

}

void calcPIDErrorValues() {

}

void tunePIDValues() {
    
}

void calcMotorOutputValues() {

}*/
