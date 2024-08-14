/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
// Number Pi
#define PI 3.14159226535897932384626433832795

#define WHEEL_PULSES 					210.0																// Xung cho banh xe quay 1 vong
#define RESOLVE			 				4.0																// Do phan giai AB = 4
#define CIRCLE_PULSE						( WHEEL_PULSES * RESOLVE )							// Tong so xung Encoder x4 doc duoc
#define WHEEL_DIAMETER				34																	// Duong kinh banh xe (mm)
#define WHEEL_CIRCUMFERENCE		( WHEEL_DIAMETER * PI )									// Chu vi banh xe
#define WHEEL_PULSES_MM				( CIRCLE_PULSE / WHEEL_CIRCUMFERENCE )		// Moi quan he giua chu vi banh xe va xung => 0.5086(mm / pulses)
#define WHEEL_MM_PULSES				( WHEEL_CIRCUMFERENCE / CIRCLE_PULSE )		// Moi quan he gua chu vi banh xe va xung =>  7.864(pulses / mm)
#define MOUSE_WIDE						100.0 // Chieu dai chuot


/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Variable Calculate Speed MicroMouse From Encoder
static volatile uint16_t   u16_Encoder_Left_Velocity;
static volatile uint16_t   u16_Encoder_Right_Velocity;

// Variable Caculate Distance MicroMouse From Speed
static volatile uint16_t u16_Encoder_Left_Distance = 0;
static volatile uint16_t u16_Encoder_Right_Distance = 0;

static volatile float f_Encoder_Velocity_Center = 0;
static volatile float f_Encoder_Distance_Center = 0;

uint16_t u16_BufferADC[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void ADC_Select_CH2( void );

extern void ADC_Select_CH3( void );

extern void ADC_Select_CH4( void );

void Read_ADC( void );

void Read_Encoder( void );

void Observer( void );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern ADC_HandleTypeDef hadc1;

extern DMA_HandleTypeDef hdma_usart1_tx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  Read_ADC( );
  Read_Encoder();
  Observer();
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Read_ADC( void )
{
	HAL_GPIO_WritePin( Sensor_Right_GPIO_Port, Sensor_Right_Pin, GPIO_PIN_SET );			// turn on IR right
	ADC_Select_CH2();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	u16_BufferADC[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_GPIO_WritePin(Sensor_Right_GPIO_Port, Sensor_Right_Pin, GPIO_PIN_RESET );			// turn off IR right
	
	HAL_GPIO_WritePin( Sensor_Left_GPIO_Port, Sensor_Left_Pin, GPIO_PIN_SET );					// turn on IR left
	ADC_Select_CH3();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	u16_BufferADC[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_GPIO_WritePin( Sensor_Left_GPIO_Port, Sensor_Left_Pin, GPIO_PIN_RESET );			// turn off IR right
	
	HAL_GPIO_WritePin( Sensor_Front_GPIO_Port, Sensor_Front_Pin, GPIO_PIN_SET );					// turn on IR left
	ADC_Select_CH4();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	u16_BufferADC[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_GPIO_WritePin( Sensor_Front_GPIO_Port, Sensor_Front_Pin, GPIO_PIN_RESET );			// turn off IR right
}

void Read_Encoder(void)
{
		
	u16_Encoder_Left_Velocity = __HAL_TIM_GET_COUNTER(&htim2);		// Toc do (pulses/ms) = pulse / thoi gian (1ms) = Khoang cach / thoi gian
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	u16_Encoder_Left_Distance += u16_Encoder_Left_Velocity;					// Khoang cach = Toc do / thoi gian (1ms). Thoi gian moi lan doc mac dinh la 1ms. Cu sau 1ms la tong quang duong di duoc	
	
	u16_Encoder_Right_Velocity	= __HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	u16_Encoder_Right_Distance += u16_Encoder_Right_Velocity;
	
	f_Encoder_Velocity_Center = ( u16_Encoder_Left_Velocity + u16_Encoder_Right_Velocity) * 0.5;
	f_Encoder_Distance_Center = ( u16_Encoder_Left_Distance + u16_Encoder_Right_Distance) * 0.5 ;
	
}
// xe di duoc 180mm <=> 1415 xung.
void Observer ( void )
{
	static volatile float tesst = 0;
	tesst =(f_Encoder_Distance_Center * WHEEL_MM_PULSES);
}
/* USER CODE END 1 */
