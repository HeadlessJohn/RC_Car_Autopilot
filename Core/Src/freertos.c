/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc.h"
#include "usart.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include "vl53l0x_api.h"
// #include "vl53l0x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_VAL_LEFT_X	adc_value[0]
#define ADC_VAL_LEFT_Y	adc_value[1]
#define ADC_VAL_RIGHT_Y	adc_value[2]
#define ADC_VAL_RIGHT_X	adc_value[3]
#define BTN_VAL			adc_value[4]

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define RC_IDLE			    0
#define RC_STOP			    1
#define RC_FORWARD		    2
#define RC_FAST_FORWARD	    3
#define RC_BACKWARD			4
#define RC_FAST_BACKWARD	5
#define RC_TURN_LEFT		6
#define RC_FAST_TURN_LEFT	7
#define RC_TURN_RIGHT	    8
#define RC_FAST_TURN_RIGHT	9


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t thread1_Handle;
const osThreadAttr_t thread1_attr = {
  .name = "thread1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 30,
};
osThreadId_t thread2_Handle;
const osThreadAttr_t thread2_attr = {
  .name = "thread2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 29,
};
osThreadId_t thread3_Handle;
const osThreadAttr_t thread3_attr = {
  .name = "thread3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 28,
};
osThreadId_t thread4_Handle;
const osThreadAttr_t thread4_attr = {
  .name = "thread4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 27,
};
osThreadId_t thread5_Handle;
const osThreadAttr_t thread5_attr = {
  .name = "thread5",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 27,
};
osThreadId_t thread6_Handle;
const osThreadAttr_t thread6_attr = {
  .name = "thread6",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 29,
};


// global variables
uint8_t state = RC_IDLE;
uint8_t next_state = RC_IDLE;

uint8_t bt_recv_done_flag = 0;
uint8_t auto_pilot_flag = 0;
uint8_t body_control = 1;
uint8_t dir_l = FORWARD;
uint8_t dir_r = FORWARD;
int16_t speed_l		   = 0;
int16_t speed_r		   = 0;
uint8_t bt_rx_data[1];
uint8_t rx_data[1];
uint8_t bt_rx_buffer[50] = {0,};
uint8_t rx_buffer[50] = {0,};
int16_t foward_speed = 0;
int16_t turn_speed = 0;
uint16_t diff_lr = 0;

uint16_t adc_value[5] = {0,};

uint16_t avg_left = 0;
uint16_t avg_front = 0;
uint16_t avg_right = 0;

// VL53L0X variables extern 선언
extern VL53L0X_RangingMeasurementData_t RangingData;
extern VL53L0X_Dev_t  vl53l0x_c; 
extern VL53L0X_DEV    Dev;

extern VL53L0X_RangingMeasurementData_t RangingData2;
extern VL53L0X_Dev_t  vl53l0x_c2;
extern VL53L0X_DEV    Dev2;

extern VL53L0X_RangingMeasurementData_t RangingData3;
extern VL53L0X_Dev_t  vl53l0x_c3;
extern VL53L0X_DEV    Dev3;

extern uint16_t distance_arr_left[5];
extern uint16_t distance_arr_front[5];
extern uint16_t distance_arr_right[5];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void thread1(void *argument);
void thread2(void *argument);
void thread3(void *argument);
void thread4(void *argument);
void thread5(void *argument);
void thread6(void *argument);

PUTCHAR_PROTOTYPE{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	static uint8_t bt_rx_cnt = 0;

	if (huart->Instance == USART6) {
		// Bluetooth
		HAL_UART_Receive_IT(&huart6, bt_rx_data, 1);

		if (bt_rx_data[0]=='\n' || bt_rx_data[0]=='\r') {
			bt_rx_buffer[bt_rx_cnt] = '\0';
			HAL_UART_Transmit(&huart6, bt_rx_buffer, bt_rx_cnt, 1000);
			bt_rx_cnt = 0;
			bt_recv_done_flag = 1;
		}
		else {
			bt_rx_buffer[bt_rx_cnt++] = bt_rx_data[0];
		}
	}

	else if (huart->Instance == USART2) {
		// USB
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_data, 1);
		HAL_UART_Transmit(&huart2, (uint8_t *)rx_data, 1, 1000);
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	// Initialize the motors
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	// Initialize the bluetooth and USB DMA
  	// HAL_UART_Receive_DMA(&huart6, (uint8_t *)bt_rx_buffer, 1);
	// HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_buffer, 1);

	// Initialize the bluetooth and USB Interrupt
	HAL_UART_Receive_IT(&huart6, (uint8_t *)bt_rx_data, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_data, 1);

	printf("RTOS Begin\n");
	
	wheel_stop();

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	ssd1306_SetCursor(10, 20);
	ssd1306_WriteString("Left Front Right", Font_7x10, White);
	ssd1306_UpdateScreen();

	VL53L0X_Init();

	HAL_UART_Transmit(&huart2, (uint8_t *)"VL53L0X Start\n", 12, 50);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  thread1_Handle = osThreadNew(thread1, NULL, &thread1_attr);
  thread2_Handle = osThreadNew(thread2, NULL, &thread2_attr);
  thread3_Handle = osThreadNew(thread3, NULL, &thread3_attr);
  thread4_Handle = osThreadNew(thread4, NULL, &thread4_attr);
  thread5_Handle = osThreadNew(thread5, NULL, &thread5_attr);
  thread6_Handle = osThreadNew(thread6, NULL, &thread6_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  	for(;;){
		// Heartbeat
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// VL53L0X 적외선 센서로 부터 거리 데이터 읽기
void thread1(void *argument){
	// RangingData.RangeMilliMeter 에 거리값 저장
	for(;;){
		VL53L0X_MeasurementStart();
		osDelay(33);
	}
}

// 이동평균 계산
void thread2(void *argument){
	uint16_t sum_left = 60;
	uint16_t sum_front = 60;
	uint16_t sum_right = 60;

	for(;;){
		for (uint8_t i = 0; i < 3; i++) {
			sum_left  += distance_arr_left[i];
			sum_front += distance_arr_front[i];
			sum_right += distance_arr_right[i];
		}
		
		avg_left  = sum_left  / 3;
		avg_front = sum_front / 3;
		avg_right = sum_right / 3;

		sum_front = 0;
		sum_left  = 0;
		sum_right = 0;

		osDelay(33);
	}
}


void thread3(void *argument){

	for(;;){
		// 자율주행
		if (auto_pilot_flag == 1) {
			// 좌우 차이 20mm 이상일 때 회전
			diff_lr = abs(avg_left - avg_right);

			state = next_state;
			if (avg_front > 900 && avg_left > 90 && avg_right > 90){
				//전방 여유 600, 좌우 70 : 고속 직진
				next_state = RC_FAST_FORWARD;
			}
			else if (diff_lr <= 30){
				next_state = RC_FORWARD;
			}
			else {
				if (avg_front < 90){
					next_state = RC_BACKWARD;
				}
				else if (avg_left < 50) {
					next_state = RC_FAST_TURN_RIGHT;
				}
				else if (avg_left < 100) {
					next_state = RC_TURN_RIGHT;
				}
				else if (avg_right < 50) {
					next_state = RC_FAST_TURN_LEFT;
				}
				else if (avg_right < 100) {
					next_state = RC_TURN_LEFT;
				}
				else next_state = RC_FORWARD;
			}
		}
		osDelay(10);
	}
}

// 거리 데이터 OLED 출력
void thread4(void *argument){
	char tmp[50];
	for(;;){
		ssd1306_SetCursor(30, 0);
		if (auto_pilot_flag == 1) ssd1306_WriteString(" AUTO ", Font_11x18, White);
		else                      ssd1306_WriteString("MANUAL", Font_11x18, White);
		ssd1306_SetCursor(7, 35);
		sprintf(tmp, "%4d  %4d  %4d", avg_left, avg_front, avg_right);
		ssd1306_WriteString(tmp, Font_7x10, White);
		ssd1306_SetCursor(25, 50);
		sprintf(tmp, "%4d  %4d", speed_l, speed_r);
		ssd1306_WriteString(tmp, Font_7x10, White);
		ssd1306_UpdateScreen();
		osDelay(100);
	}
}

// Bluetooth 데이터 수신
void thread5(void *argument){
	char *adc[5];

	for(;;){
		if (bt_recv_done_flag == 1) {
			bt_recv_done_flag = 0; // clear flag

			// ADC DATA Parsing
			for (uint8_t i = 0; i < 4; i++) {
				adc[i] = strtok( (i==0) ? bt_rx_buffer : NULL , ",");
				adc_value[i] = atoi(adc[i]);
				if(adc_value[i] <= 5){
					adc_value[i] = 2048;
				}
			}
			// Button Parsing
			adc[4] = strtok(NULL, ",");
			adc_value[4] = atoi(adc[4]);

			switch (adc_value[4]){
				case 0:
					break;
				case 1:
					auto_pilot_flag = 0;
					break;
				case 2:
					auto_pilot_flag = 1;
					break;
			}
		}
		osDelay(3);	
	}
}

void thread6(void *argument){
	
	for(;;){
		

		if (auto_pilot_flag == 1){
			/*
			// logic
			switch (state) {
				case RC_IDLE :            speed_l = 0;    speed_r = 0;    break;
				case RC_FORWARD :         speed_l = 500 + avg_right;  
										  speed_r = 500 + avg_left;  break;
				case RC_BACKWARD :        if (avg_left>avg_right) {
											speed_l = -avg_left;
										    speed_r = 0;
										  } 
										  else {
											speed_l = 0;
											speed_r = -avg_right;
										  }break;
				case RC_FAST_FORWARD :    speed_l = 1000; speed_r = 1000; break;
				case RC_TURN_LEFT :       speed_l = 300 - avg_right;  
										  speed_r = 500 - avg_left;  break;
				case RC_FAST_TURN_LEFT :  speed_l = -500;    speed_r = 500; break;
				case RC_TURN_RIGHT :      speed_l = 500 - avg_right;  
										  speed_r = 300 - avg_left;  break;
				case RC_FAST_TURN_RIGHT : speed_l = 500; speed_r = -500;    break;
			}
			*/
			
			if (diff_lr >= 20){
				speed_l = avg_front + avg_right - avg_left;
				speed_r = avg_front + avg_left - avg_right;

				// 좌측 벽이 35mm 이내로 접근한 경우
				if (avg_left < 35){
					speed_l = avg_front + 30 + avg_right/2;
					speed_r = avg_front/2;
				}
				// 우측 벽이 50mm 이내로 접근한 경우
				if (avg_right < 35){
					speed_l = avg_front/2;
					speed_r = avg_front + 30 + avg_left/2;
				}
			}
			// 좌우 차이 20mm 미만일 때 직진
			else{
				if (avg_front > 200) {
					speed_l = 400 + (2 * avg_front ) + avg_right;
					speed_r = 400 + (2 * avg_front ) + avg_left;
					// osDelay(100);
				}
				else {
					speed_l = 100 + avg_front*2/3 + avg_right;
					speed_r = 100 + avg_front*2/3 + avg_left;
				}
			}

			// 전방이 90mm 이하일 때 후진
			// 오른쪽이 좁으면 왼쪽 바퀴를 뒤로 더 돌린다
			if (avg_front < 90) {
				speed_l = -150 - avg_right/2;
				speed_r = -150 - avg_left/2;
			}

			// 휠 방향 전환, 속도 조정
			if (speed_l < 0) {
				dir_l = BACKWARD;
				speed_l = abs(speed_l);
			}
			else {
				dir_l = FORWARD;
				speed_l = speed_l;
			}

			if (speed_r < 0) {
				dir_r = BACKWARD;
				speed_r = abs(speed_r);
			}
			else {
				dir_r = FORWARD;
			}
		}
		// 수동주행
		else {
			foward_speed = (ADC_VAL_LEFT_Y >> 1) - 1024+20 ; // -1000 ~ 1000
			turn_speed   = (ADC_VAL_RIGHT_X >> 1) - 1024+40; // -1000 ~ 1000 오른쪽이 큰값 , offset   음수이면 왼쪽으로 회전

			speed_l = foward_speed + turn_speed ;
			speed_r = foward_speed - turn_speed ;
			
			if (speed_l < 0) {
				dir_l = BACKWARD;
				speed_l = abs(speed_l);
			}
			else {
				dir_l = FORWARD;
				speed_l = speed_l;
			}

			if (speed_r < 0) {
				dir_r = BACKWARD;
				speed_r = abs(speed_r);
			}
			else {
				dir_r = FORWARD;
			}

			if (speed_l > RC_MAX_SPEED) {
				speed_l = RC_MAX_SPEED;
			}
			if (speed_r > RC_MAX_SPEED) {
				speed_r = RC_MAX_SPEED;
			}
			
			if (speed_l < RC_MIN_SPEED+100) {
				speed_l = 0;
			}
			if (speed_r < RC_MIN_SPEED+100) {
				speed_r = 0;
			}
		}

		left_wheel_dir(dir_l);
		right_wheel_dir(dir_r);
		left_wheel_set_speed(speed_l);
		right_wheel_set_speed(speed_r);
		osDelay(10);
	}
}
/* USER CODE END Application */

