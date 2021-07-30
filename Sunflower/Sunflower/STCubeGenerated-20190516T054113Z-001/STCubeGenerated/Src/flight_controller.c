#include "flight_controller.h"

void turnOnRedLED(char* command) {
    if (!strcmp(command, "set"))
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
    else if (!strcmp(command, "toggle"))
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

void turnOnBlueLED(char* command) {
    if (!strcmp(command, "set"))
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
    else if (!strcmp(command, "toggle"))
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
}


void startPWM(TIM_HandleTypeDef htim1) {
	turnOnRedLED("set");
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

void stopPWM(TIM_HandleTypeDef htim1) {
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
}


void startBlueTooth(char rx_buffer[20], char tx_buffer[20], char status[20], UART_HandleTypeDef huart1) {
  turnOnBlueLED("toggle");
  HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, sprintf(tx_buffer, "\nConnected: "), 500);
  HAL_UART_Receive(&huart1, (uint8_t*)rx_buffer, 10, 500);
	HAL_UART_Transmit(&huart1, (uint8_t *)rx_buffer, 10, 500);
	status = rx_buffer;
}

void setMotorSpeed(int dutyCycle, TIM_HandleTypeDef htim1) {
		htim1.Instance->CCR1 = dutyCycle;
		htim1.Instance->CCR2 = dutyCycle;
		htim1.Instance->CCR3 = dutyCycle;
		htim1.Instance->CCR4 = dutyCycle;
}
/*
void turnOff() {

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
