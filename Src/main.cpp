/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio.h"
#include "maps.h"
#include "soft_spi.h"
#include "PID.h"
#include "data_handling.h"
#include "vehicle_lighting.h"
#include "steering.h"
#include "sensor.h"
#include "supply.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t vehicleToPilotDataBuffer[15] = {0};

HC12* radio = new HC12;
volatile uint8_t radioFlag = 0;
volatile uint8_t receivedDataBuffer[20];
volatile uint8_t byte = 0;

volatile uint8_t us100 = 0;
volatile uint16_t ms1 = 0;
volatile uint8_t s1 = 0;

uint16_t bufferADC[3] = {0};
uint8_t mainSupplyVoltage = 0;
uint16_t dcEngineSupplyVoltage = 0;

uint16_t start_position = 2048;
uint16_t adc_value;
volatile int16_t w = 0;

uint8_t drivingDirection = 1;
uint8_t speedJoystickDeflexion = 0;

uint8_t steeringDirection = 1;
uint8_t steeringJoystickDeflexion = 0;
float steeringValue = 0;

uint16_t speed = 0;
uint16_t rpm = 0;

int8_t temperature = 0;
uint8_t drivingMode = 2;
uint8_t lightRegister = 0;
volatile uint8_t engineRegister = 0;
uint8_t cruiseControlFlag = 0;
uint8_t speedForCruiseControl = 0;
uint8_t AVBSActivated = 0;

volatile uint8_t softStartLightsCnt = 0;
volatile uint8_t softStartLightsFlag = 0;
volatile uint8_t cntPwm = 0;
volatile float pwmValue[4];

SPIDevice* spiLightRegister = new SPIDevice;
SPIDevice* spiEngineRegister = new SPIDevice;

volatile uint16_t capturedNumOfEncoderImpulses = 0;
volatile uint16_t capturedAllImpulsesTime = 0;
volatile uint16_t cntEncoderImpulses = 0;
volatile uint16_t allImpulsesTime = 0;
volatile uint8_t encoderFlag = 0;

volatile uint16_t distance = 0;
uint8_t sensorMeasureFlag = 1;
volatile uint16_t cntLengthImpulse = 0;
volatile uint8_t cntLengthImpulseFlag = 0;

uint8_t lms5 = 0;
uint8_t lms10 = 0;
uint8_t lms20 = 0;
uint8_t lms50 = 0;
uint8_t lms200 = 0;
uint8_t ls1 = 0;

uint8_t serviceModeFlag = 0;
volatile uint16_t cntForIntegralTerm = 0;
volatile uint8_t dir = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&byte, 1);
  HAL_ADC_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)bufferADC, 3);

  PID steeringPID;
  steeringPID.PID_init(1.1f, 1.0f, 0.1);
  steeringPID.SetNewTauIT(0.3f);

  radio->HC12Init(&huart1, (uint8_t*)receivedDataBuffer, (uint8_t*)&byte, (uint8_t*)&radioFlag);


  spiLightRegister->DeviceSPIInit(MOSI_GPIO_Port, MOSI_Pin, SCK_GPIO_Port, SCK_Pin, CS1_GPIO_Port, CS1_Pin);
  spiLightRegister->SendSoftSpi(0);
  spiEngineRegister->DeviceSPIInit(MOSI_GPIO_Port, MOSI_Pin, SCK_GPIO_Port, SCK_Pin, CS2_GPIO_Port, CS2_Pin);
  spiEngineRegister->SendSoftSpi(0);
  radio->SendRadioFrameIT(0, 0, QUESTION_FOR_INITIAL_DATA);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(lms20 != ms1 / 20)
	  {
		  /*if(sensorMeasureFlag)
		  {
			  TrigPinSensor(ON);
			  sensorMeasureFlag = 0;
		  }*/
		  steeringValue = steeringPID.IntegralTermCalculate(steeringValue, (uint16_t*)&cntForIntegralTerm);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500 - steeringValue);

		  lms20 = ms1 / 20;
	  }
	  if(radioFlag) ExecuteCommand((uint8_t*)receivedDataBuffer);

	  if(lms200 != ms1 / 200)
	  {
		  CalculateSupplyVoltage();
		  CalculateTemperature(bufferADC[2]);
		  SendStandardDataToPilot();

		  lms200 = ms1 / 200;
	  }
	  if(encoderFlag || (allImpulsesTime >= 2000 && cntEncoderImpulses == 0))
	  {
		  rpm = 300000 * capturedNumOfEncoderImpulses / capturedAllImpulsesTime;
		  speed = (rpm / 4) * 600 / 4694.84f;
		  allImpulsesTime = 0;
		  encoderFlag = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ENKODER_Pin)
	{
		cntEncoderImpulses++;
		if(allImpulsesTime >= 1000)
		{
			capturedAllImpulsesTime = allImpulsesTime;
			capturedNumOfEncoderImpulses = cntEncoderImpulses;
			cntEncoderImpulses = 0;
			encoderFlag = 1;
		}
	}
	else if(GPIO_Pin == ECHO_Pin)
	{
		if(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
		{
			cntLengthImpulseFlag = 1;
		}
		else
		{
			distance = cntLengthImpulse / 58;
			cntLengthImpulseFlag = 0;
			cntLengthImpulse = 0;
			sensorMeasureFlag = 1;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		radio->GetRadioFrameIT();
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&byte, 1);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2){
		us100++;
		allImpulsesTime++;
		cntPwm++;
		if(cntPwm > 100) cntPwm = 1;
		if(cntLengthImpulseFlag) cntLengthImpulse++;
		if(softStartLightsFlag) SoftStartOfLights();

		if(us100 > 9)
		{
			us100 = 0;
			ms1++;
			cntForIntegralTerm++;
			CalculateFinalValuesOfSteering();
			HBridgeSteering();

			if(softStartLightsFlag)
			{
			 	if((ms1 % 40) == 0)
				{
					if(softStartLightsCnt < 100)
					{
						if(softStartLightsCnt >= 40) softStartLightsCnt += 2;
						else softStartLightsCnt++;
					}
					else softStartLightsFlag = 0;
				}
			}
			if(ms1 > 999)
			{
				ms1 = 0;
				s1++;
			}
		}
		//TrigPinSensor(OFF);
	}
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
