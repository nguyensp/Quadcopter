#include "flight_controller.h"

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
