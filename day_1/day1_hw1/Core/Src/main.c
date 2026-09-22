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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 동작이 반대로 나오면 아래 세 줄을 뒤집는다 */
#define SW_PRESSED              GPIO_PIN_RESET
#define LED_ON                  GPIO_PIN_RESET
#define LED_OFF                 GPIO_PIN_SET

#define SW_PORT                 GPIOB
#define SW_RIGHT_PIN            GPIO_PIN_12
#define SW_INVERT_PIN           GPIO_PIN_13
#define SW_WIDE_PIN             GPIO_PIN_14
#define SW_PAUSE_PIN            GPIO_PIN_15

#define PD2_PORT                GPIOD
#define PD2_PIN                 GPIO_PIN_2

#define LED_NUM                 4u
#define LED_MASK                0x0Fu

#define LED_SHIFT_PERIOD_MS     200u
#define PD2_TOGGLE_PERIOD_MS    500u

/* 1: PB14 누르면 켜지는 LED 2개 / 0: PB14 누르면 2칸씩 이동 */
#define SW14_MEANS_LED_COUNT    1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static GPIO_TypeDef * const LED_PORT[LED_NUM] = { GPIOB, GPIOB, GPIOB, GPIOB };
static const uint16_t       LED_PIN [LED_NUM] = { GPIO_PIN_10, GPIO_PIN_2,
                                                  GPIO_PIN_1, GPIO_PIN_0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t sw_pressed(uint16_t pin);
static uint8_t build_pattern(uint8_t pos, uint8_t width);
static void    led_write(uint8_t pattern);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t sw_pressed(uint16_t pin)
{
  return (HAL_GPIO_ReadPin(SW_PORT, pin) == SW_PRESSED) ? 1u : 0u;
}

/* pos 부터 width 개를 켠 4비트 패턴. 끝을 넘으면 앞으로 돌아온다 */
static uint8_t build_pattern(uint8_t pos, uint8_t width)
{
  uint8_t pattern = 0u;
  uint8_t i;

  for (i = 0u; i < width; i++)
  {
    pattern |= (uint8_t)(1u << ((pos + i) % LED_NUM));
  }
  return (uint8_t)(pattern & LED_MASK);
}

static void led_write(uint8_t pattern)
{
  uint8_t i;

  for (i = 0u; i < LED_NUM; i++)
  {
    GPIO_PinState state = (pattern & (1u << i)) ? LED_ON : LED_OFF;
    HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], state);
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
  /* USER CODE BEGIN 2 */
  uint32_t t_shift  = HAL_GetTick();
  uint32_t t_toggle = HAL_GetTick();

  uint8_t       led_pos   = 0u;
  GPIO_PinState pd2_state = LED_OFF;

  led_write(build_pattern(led_pos, 1u));
  HAL_GPIO_WritePin(PD2_PORT, PD2_PIN, pd2_state);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 스위치는 서로 독립이라 따로 읽는다 (동시 입력 허용) */
    uint8_t sw_right  = sw_pressed(SW_RIGHT_PIN);
    uint8_t sw_invert = sw_pressed(SW_INVERT_PIN);
    uint8_t sw_wide   = sw_pressed(SW_WIDE_PIN);
    uint8_t sw_pause  = sw_pressed(SW_PAUSE_PIN);

#if SW14_MEANS_LED_COUNT
    uint8_t width = sw_wide ? 2u : 1u;
    uint8_t step  = 1u;
#else
    uint8_t width = 1u;
    uint8_t step  = sw_wide ? 2u : 1u;
#endif

    if ((now - t_shift) >= LED_SHIFT_PERIOD_MS)
    {
      t_shift = now;

      if (sw_right)
      {
        /* 우측 이동. 음수 방지를 위해 LED_NUM 을 더하고 나머지를 취한다 */
        led_pos = (uint8_t)((led_pos + LED_NUM - step) % LED_NUM);
      }
      else
      {
        led_pos = (uint8_t)((led_pos + step) % LED_NUM);
      }
    }

    /* 매 루프 출력해야 PB13, PB14 가 다음 shift 를 안 기다리고 바로 반영된다 */
    uint8_t pattern = build_pattern(led_pos, width);

    if (sw_invert)
    {
      pattern = (uint8_t)((~pattern) & LED_MASK);
    }
    led_write(pattern);

    /* PB15 를 누르는 동안은 WritePin 을 안 해서 직전 상태가 유지된다 */
    if (sw_pause)
    {
      t_toggle = now;
    }
    else if ((now - t_toggle) >= PD2_TOGGLE_PERIOD_MS)
    {
      t_toggle  = now;
      pd2_state = (pd2_state == LED_ON) ? LED_OFF : LED_ON;
      HAL_GPIO_WritePin(PD2_PORT, PD2_PIN, pd2_state);
    }
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
