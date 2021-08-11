/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TJ_MPU6050.h"
#include "dwt_stm32_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMPI2C_HandleTypeDef hfmpi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;
/* USER CODE END 0 */
	float sampling_period;
	float PI = 3.1415926535897;
	float acc_x, acc_y, acc_z;
	float gyro_x, gyro_y, gyro_z;
	float gyro_x_cal, gyro_y_cal, gyro_z_cal;
	float angle_pitch_gyro, angle_roll_gyro, angle_yaw_gyro;
	float angle_pitch_acc, angle_roll_acc, angle_yaw_acc, acc_total_vector;
	float angle_pitch_output, angle_roll_output, angle_yaw_output;
	bool set_gyro_angles = false;
	uint16_t dutyCycle = 2;
	//PID Variables
	float pid_error_roll, pid_error_pitch, pid_error_yaw;
	float pid_i_mem_roll, pid_i_mem_pitch, pid_i_mem_yaw;
	float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
	float pid_d_last_roll_error, pid_d_last_pitch_error, pid_d_last_yaw_error;
	float pid_out_roll, pid_out_pitch, pid_out_yaw;
	//Tune
	float pid_p_gain_roll = 1, pid_i_gain_roll = 1, pid_d_gain_roll =1;
	float pid_p_gain_pitch = 1, pid_i_gain_pitch = 1, pid_d_gain_pitch =1;
	float pid_p_gain_yaw = 1, pid_i_gain_yaw = 1, pid_d_gain_yaw = 1;
	//Motors
	float motor_1_fr, motor_2_rr, motor_3_rl, motor_4_fl;
	//Loop Timer
	uint32_t oldtime;
	int count;
	uint32_t newtime;
	int test, result;
	bool temp;
	long looptimer;
	float t1, t2, t3, t4, t5, t6, t7, t8, t9;
	char rx_buffer[50], tx_buffer[50];
	bool led_state = false;
	
	// bluetooth code
	char rxData[30];

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_FMPI2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	// added blut code
		char txData[30] = "Hello World\r\n";
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxData, 10);
	
	//1. Initialize the MPU6050 module and I2C
	MPU6050_Init(&hfmpi2c1);
	//2. Configure Accel and Gyro Parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0; //0 is normal mode
	MPU6050_Config(&myMpuConfig);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		//Output is 200Hz accordingly google calculation + Liz (caveat emptor)
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
		


		count = 0;
		temp = false;
		oldtime = 0;
		DWT_Delay_Init();
		
		//Aggregate loop average not working, numbers derived by observation
		gyro_x_cal = -140;
		gyro_y_cal = 45;
		gyro_z_cal = 70;
		
		t1 = 0;
		t2 = 0;
		t3 = 0;
		t4 = 0;
		t5 = 0;
		t6 = 0;
		t7 = 0;
		t8 = 0;
		t9 = 0;
		
					 //Solution
			//MPU6050_Get_Gyro_RawData(&myGyroRaw);
			//t1 = myGyroRaw.x;
			//t2 += t1;
			
		/*
		for (int cal_int = 0; cal_int < 2000; cal_int++)
		{
			MPU6050_Get_Gyro_RawData(&myGyroRaw);
			t1 = myGyroRaw.x;
			gyro_x_cal += t1;
			t2 = myGyroRaw.y;
			gyro_y_cal += t2;
			t3 = myGyroRaw.z;
			gyro_z_cal += t3;
			count++;
		}
		gyro_x_cal /= 2000;
		gyro_y_cal /= 2000;
		gyro_z_cal /= 2000;
		*/

		pid_i_mem_roll = 0;
		pid_i_mem_pitch = 0;
		pid_i_mem_yaw = 0;
		
		test = 49;
		result = sqrt(test);
		looptimer = HAL_GetTick();

  while (1)
  {

//				HAL_UART_Transmit(&huart1, myTxData, 13, 10);
//				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
//				HAL_Delay(1000);
				
//			HAL_UART_Receive(&huart1, (uint8_t*)rx_buffer, 50, 500);
//		
//			if(rx_buffer[0] == 'o' && rx_buffer[1] == 'n')
//			{
//				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);	//Bloo
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//				if(led_state != true)
//				HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, sprintf(tx_buffer, "Led is on\n"), 500);	
//				led_state = true;
//			}
//			else if(rx_buffer[0] == 'o' && rx_buffer[1] == 'f' && rx_buffer[2] == 'f')
//			{
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//				HAL_Delay(10);
//				if(led_state != false)
//				HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, sprintf(tx_buffer, "Led is off\n"), 500);	
//				led_state = false;
//				
//			}
		
//			char buff2[ 6 ] = "\r\n>>";
//			HAL_UART_Transmit (&huart1, buff2, strlen( buff2 ), 10 );
//			char buff[50];
//			memset(buff,0,50);
//			HAL_UART_Receive(&huart1,buff,50,5000);
//			if(strcmp(buff, "on") == 0 )
//			{
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
//			} else if (strcmp(buff,"off") == 0) {
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
//				
//			}


			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);		// changed this!!!!!!!
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); 	//RED
			/*Solution
			MPU6050_Get_Gyro_RawData(&myGyroRaw);
			t1 = myGyroRaw.x;
			t2 += t1;
			*/
			
		//Microsecond Test
//			if(temp == false)
//			{
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
//				temp = true;
//			}
//			else
//			{
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
//				temp = false;
//			}
//			
//			DWT_Delay_us(1000000);

				
			
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); //Greem				
	
//		//Time Splice
//		newtime = DWT->CYCCNT;
//		
//		if((newtime - oldtime) > 3000000)
//		{
//			oldtime = newtime;
//			if(temp == false)
//			{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
//			temp = true;
//			}
//			else
//			{
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
//				temp = false;
//			}
//		}
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		//Raw Data
		MPU6050_Get_Accel_RawData(&myAccelRaw);
//		MPU6050_Get_Gyro_RawData(&myGyroRaw);
//		//Scaled Data
//		MPU6050_Get_Accel_Scale(&myAccelScaled);
//		MPU6050_Get_Gyro_Scale(&myGyroScaled);

		acc_x = myAccelRaw.x;
		acc_y = myAccelRaw.y;
		acc_z = myAccelRaw.z;
		

		/* Solution Must be in While(1)
		for (int cal_int = 0; cal_int < 2000; cal_int++)
		{
			MPU6050_Get_Gyro_RawData(&myGyroRaw);
			t1 = myGyroRaw.x;
			t4 += t1;
			t2 = myGyroRaw.y;
			t5 += t2;
			t3 = myGyroRaw.z;
			t6 += t3;
		}
		gyro_x_cal = t4/2000;
		gyro_y_cal = t5/2000;
		gyro_z_cal = t6/2000;
		*/

		MPU6050_Get_Gyro_RawData(&myGyroRaw);
		t4 = myGyroRaw.x;
		gyro_x = t4-gyro_x_cal;
		t5 = myGyroRaw.y;
		gyro_y = t5-gyro_y_cal;
		t6 = myGyroRaw.z;
		gyro_z = t6-gyro_z_cal;
//		
//		gyro_x -= gyro_x_cal;                                                
//		gyro_y -= gyro_y_cal;                                                
//		gyro_z -= gyro_z_cal;
//		
		//Gyro angle calculations, 250Hz = 1 loop every 4000uS
		t1 = gyro_x / (6.6666*65.5);
		angle_pitch_gyro += t1;      		
		t2 = gyro_y / (6.6666*65.5);
		angle_roll_gyro += t2; 		
		t3 = gyro_z / (-6.6666*65.5);
		angle_yaw_gyro += t3;
//		
//	
//		//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
//		angle_pitch_gyro += angle_roll_gyro * sin(gyro_z * (PI/180));               //If the IMU has yawed transfer the roll angle to the pitch angel
//		angle_roll_gyro -= angle_pitch_gyro * sin(gyro_z * (PI/180));               //If the IMU has yawed transfer the pitch angle to the roll angel
//		
		//Accelerometer Angle Calculations
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
		  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
		angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
		angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
		angle_yaw_acc = asin(acc_z/acc_total_vector)* -57.296;       //Calculate the yaw angle
		
		//angle_pitch_acc -= 0.0;
		//angle_roll_acc -= 0.0;
		
		if(set_gyro_angles){                                                 //If the IMU is already started
			angle_pitch_gyro = angle_pitch_gyro * 0.9 + angle_pitch_acc * 0.1;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
			angle_roll_gyro = angle_roll_gyro * 0.9 + angle_roll_acc * 0.1;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
			angle_yaw_gyro = angle_yaw_gyro * 0.9 + angle_yaw_acc * 0.1;
		}
		else{                                                                //At first start
			angle_pitch_gyro = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
			angle_roll_gyro = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
			angle_yaw_gyro = angle_yaw_acc;
			set_gyro_angles = true;                                            //Set the IMU started flag
		}		
		
		angle_pitch_output = angle_pitch_gyro;
		angle_roll_output = angle_roll_gyro;
		angle_yaw_output = angle_yaw_gyro + 87;
		
		//angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch_gyro * 0.1;
		//angle_roll_output = angle_roll_output * 0.9 + angle_roll_gyro * 0.1;
//		
//		/*Update new angle values, Sum Delays for Sampling Period
//		sampling_period = (looptimer*3)*.001;
//		angle_pitch_gyro += myGyroScaled.x*(180/PI) * sampling_period; 
//		angle_roll_gyro += myGyroScaled.y*(180/PI) * sampling_period;
//		angle_yaw_gyro += myGyroScaled.z*(-180/PI) * sampling_period;		
//		
//		//Correct Gyro's Drift with Accelerometer Angles
//		if(set_gyro_angles)
//		{
//			angle_pitch_gyro = angle_pitch_gyro * 0.90 + angle_pitch_acc * 0.1;
//			angle_roll_gyro = angle_roll_gyro * 0.90 + angle_roll_acc * 0.1;
//			angle_yaw_gyro = angle_yaw_gyro * 0.90 + angle_yaw_acc * 0.1;

//		}
//		else
//		{
//			angle_pitch_gyro = angle_pitch_acc;
//			angle_roll_gyro = angle_roll_acc;
//			angle_yaw_gyro = angle_yaw_acc;
//			set_gyro_angles = true;
//		}
//		*/
//		
		//Calculate Error
		pid_roll_setpoint = 0;
		pid_pitch_setpoint = 0;
		pid_yaw_setpoint = 0;
		pid_error_roll = angle_roll_output - pid_roll_setpoint;
		pid_error_pitch = angle_pitch_output - pid_pitch_setpoint;
		pid_error_yaw = angle_yaw_output - pid_yaw_setpoint;
		

		//PID Calculations
		//Roll Calculations P:(P_Gain * Error), I:(+=I_Gain * Error), D:(D_Gain * (Error - Last Error)
		t7 = pid_i_gain_roll * pid_error_roll;
		pid_i_mem_roll += t7;
		pid_out_roll = pid_p_gain_roll * pid_error_roll + pid_i_mem_roll + pid_d_gain_roll * (pid_error_roll - pid_d_last_roll_error);
		pid_d_last_roll_error = pid_error_roll;
		//Pitch Calculations
		t8 = pid_i_gain_pitch * pid_error_pitch;
		pid_i_mem_pitch += t8;
		pid_out_pitch = pid_p_gain_pitch * pid_error_pitch + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_pitch - pid_d_last_pitch_error);
		pid_d_last_pitch_error = pid_error_pitch;
		//Yaw Calculations
		t9 = pid_i_gain_yaw * pid_error_yaw;
		pid_i_mem_yaw +=  t9;
		pid_out_yaw = pid_p_gain_yaw * pid_error_yaw + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_yaw - pid_d_last_yaw_error);
		pid_d_last_yaw_error = pid_error_yaw; 
		
		//PID Corrections Applied
		motor_1_fr = pid_out_roll - pid_out_pitch - pid_out_yaw;
		motor_2_rr = pid_out_roll + pid_out_pitch + pid_out_yaw;
		motor_3_rl = -pid_out_roll + pid_out_pitch - pid_out_yaw;
		motor_4_fl = -pid_out_roll - pid_out_pitch + pid_out_yaw;
		
		//Motor Output
		htim1.Instance->CCR1 = dutyCycle;
		htim1.Instance->CCR2 = dutyCycle;
		htim1.Instance->CCR3 = dutyCycle;
		htim1.Instance->CCR4 = dutyCycle;
		dutyCycle+= 2;
		if(dutyCycle == 30) dutyCycle=28;
		HAL_Delay(1000);
		
//		
//		count++;
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
//		while(HAL_GetTick() - looptimer < 1);

			
			//Use this to figure out loop time of code
			if(HAL_GetTick() - looptimer > 150)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1); //rad
			}; 			

			//Control Looptime
			while(HAL_GetTick() - looptimer < 150);
			looptimer = HAL_GetTick();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_FMPI2C1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Fmpi2c1ClockSelection = RCC_FMPI2C1CLKSOURCE_APB;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x20303E5D;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  //huart1.Init.BaudRate = 115200;  changed this
	huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12 
                           PB13 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	HAL_UART_Transmit(&huart1, (uint8_t *)rxData, strlen(rxData), 10);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
