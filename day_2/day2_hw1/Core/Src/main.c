/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 스위치·LED 모두 active-low (day_1 에서 실측으로 확정) */
#define SW_PRESSED              GPIO_PIN_RESET
#define LED_ON                  GPIO_PIN_RESET
#define LED_OFF                 GPIO_PIN_SET

#define PD2_PORT                GPIOD
#define PD2_PIN                 GPIO_PIN_2

#define SW_PORT                 GPIOB
#define LED_PORT                GPIOB
#define SW_NUM                  4u

/* TIM6 : 90MHz / 90 / 100 = 10kHz → 인터럽트 1회 = 0.1ms */
#define TICKS_PER_MS            10u

/* "1초 주기 = 0.5초 ON + 0.5초 OFF" 이므로 토글 간격은 주기의 절반 */
#define PD2_PERIOD_MS           1000u
#define SW_LED_PERIOD_MS        500u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* i 번째 스위치가 i 번째 LED 를 담당한다 */
static const uint16_t SW_PIN [SW_NUM] = { GPIO_PIN_12, GPIO_PIN_13,
                                          GPIO_PIN_14, GPIO_PIN_15 };
static const uint16_t LED_PIN[SW_NUM] = { GPIO_PIN_0,  GPIO_PIN_1,
                                          GPIO_PIN_2,  GPIO_PIN_10 };

/* TIM6 인터럽트가 만드는 1ms 시간축 */
static volatile uint32_t g_ms = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void app_1ms(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 1ms 제어주기마다 TIM6 인터럽트에서 호출된다 */
static void app_1ms(void)
{
  static uint32_t t_pd2 = 0u;
  static uint32_t t_sw[SW_NUM] = { 0u };
  uint8_t i;

  /* 1. PD2 : 스위치와 무관하게 계속 토글 */
  if ((g_ms - t_pd2) >= (PD2_PERIOD_MS / 2u))
  {
    t_pd2 = g_ms;
    HAL_GPIO_TogglePin(PD2_PORT, PD2_PIN);
  }

  /* 2. 스위치를 누르고 있는 동안만 짝이 되는 LED 를 토글 */
  for (i = 0u; i < SW_NUM; i++)
  {
    if (HAL_GPIO_ReadPin(SW_PORT, SW_PIN[i]) == SW_PRESSED)
    {
      if ((g_ms - t_sw[i]) >= (SW_LED_PERIOD_MS / 2u))
      {
        t_sw[i] = g_ms;
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN[i]);
      }
    }
    else
    {
      /* 손을 뗀 동안 기준점을 밀어둔다. 핀에 쓰지 않으므로 직전 상태가 그대로 유지되고,
         다시 누르는 순간 밀린 시간만큼 즉시 토글되는 일도 없다 */
      t_sw[i] = g_ms;
    }
  }
}

/* TIM6 주기(0.1ms)마다 호출 — 10회를 모아 1ms 제어주기를 만든다 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t sub = 0u;

  if (htim->Instance == TIM6)
  {
    sub++;
    if (sub >= TICKS_PER_MS)
    {
      sub = 0u;
      g_ms++;
      app_1ms();
    }
  }
}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* CubeMX 초기 출력은 RESET 이고 active-low 라 그대로 두면 부팅 직후 전부 켜진다 */
  uint8_t i;

  HAL_GPIO_WritePin(PD2_PORT, PD2_PIN, LED_OFF);
  for (i = 0u; i < SW_NUM; i++)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN[i], LED_OFF);
  }

  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 모든 동작은 TIM6 인터럽트에서 처리한다 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
