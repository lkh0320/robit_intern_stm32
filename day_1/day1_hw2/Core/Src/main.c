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
#define MA_WINDOW           8u

/* 이동평균 상태. 3단계 요구대로 채널마다 하나씩 따로 가져야 한다 */
typedef struct
{
  uint32_t buf[MA_WINDOW];
  uint32_t sum;
  uint8_t  idx;
  uint8_t  count;
} MovAvg_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CH_NUM          4u
#define PSD_CH              0u          /* PSD 가 연결된 채널. 나머지는 미연결 */

/* PSD 는 가까울수록 출력이 높다. MAX 가 최근접, MIN 이 최원거리다.
 * psd_obs_min / psd_obs_max 를 읽어서 이 두 값에 옮겨 적는다 */
#define PSD_RAW_MIN         300u
#define PSD_RAW_MAX         3500u
#define PSD_NORM_MAX        1000u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* DMA 가 CPU 몰래 갱신하므로 volatile. 없으면 컴파일러가 읽기를 캐싱해 버린다 */
volatile uint32_t adc1_buffer[ADC_CH_NUM] = {0, };
volatile uint8_t  adc_updated_flag = 0;

/* 2-2 : 원본과 이동평균 출력 */
uint32_t adc_raw[ADC_CH_NUM]      = {0, };
uint32_t adc_filtered[ADC_CH_NUM] = {0, };

/* 2-3 : 최근 MA_WINDOW 구간의 변동폭. 필터 효과를 숫자로 비교한다 */
uint32_t adc_raw_pp[ADC_CH_NUM]   = {0, };
uint32_t adc_flt_pp[ADC_CH_NUM]   = {0, };

/* 2-1 : 정규화 결과. 0 = 최원거리, 1000 = 최근접 */
uint16_t psd_norm[ADC_CH_NUM]     = {0, };

/* 2-1 캘리브레이션용. 물체를 움직이며 관측된 범위가 여기 쌓인다 */
uint32_t psd_obs_min[ADC_CH_NUM]  = {0, };
uint32_t psd_obs_max[ADC_CH_NUM]  = {0, };

static MovAvg_t g_ma[ADC_CH_NUM];        /* 3단계 : 채널마다 독립된 필터 상태 */
static MovAvg_t g_flt_hist[ADC_CH_NUM];  /* 필터 출력 이력. 변동폭 계산용 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void     movavg_init(MovAvg_t *f);
static uint32_t movavg_update(MovAvg_t *f, uint32_t sample);
static uint32_t window_pp(const MovAvg_t *f);
static uint16_t psd_normalize(uint32_t raw);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void movavg_init(MovAvg_t *f)
{
  uint8_t i;

  for (i = 0u; i < MA_WINDOW; i++)
  {
    f->buf[i] = 0u;
  }
  f->sum   = 0u;
  f->idx   = 0u;
  f->count = 0u;
}

/* 가장 오래된 값만 빼고 새 값을 더한다. 매번 윈도우 전체를 더하지 않는다 */
static uint32_t movavg_update(MovAvg_t *f, uint32_t sample)
{
  f->sum -= f->buf[f->idx];
  f->buf[f->idx] = sample;
  f->sum += sample;

  f->idx = (uint8_t)((f->idx + 1u) % MA_WINDOW);

  /* 윈도우가 다 차기 전에는 들어온 개수로 나눠야 초반 값이 0쪽으로 끌리지 않는다 */
  if (f->count < MA_WINDOW)
  {
    f->count++;
  }
  return f->sum / f->count;
}

/* 윈도우 안의 최대-최소. 원본과 필터의 흔들림 폭을 숫자로 비교하기 위한 것 */
static uint32_t window_pp(const MovAvg_t *f)
{
  uint32_t lo, hi;
  uint8_t  i;

  if (f->count == 0u)
  {
    return 0u;
  }

  lo = f->buf[0];
  hi = f->buf[0];

  for (i = 1u; i < f->count; i++)
  {
    if (f->buf[i] < lo) { lo = f->buf[i]; }
    if (f->buf[i] > hi) { hi = f->buf[i]; }
  }
  return hi - lo;
}

/* Min-Max Normalization. 전압에 비례하므로 결과는 거리가 아니라 가까운 정도다 */
static uint16_t psd_normalize(uint32_t raw)
{
  if (raw <= PSD_RAW_MIN)
  {
    return 0u;
  }
  if (raw >= PSD_RAW_MAX)
  {
    return (uint16_t)PSD_NORM_MAX;
  }
  return (uint16_t)(((raw - PSD_RAW_MIN) * PSD_NORM_MAX) / (PSD_RAW_MAX - PSD_RAW_MIN));
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
  /* USER CODE BEGIN 2 */
  uint8_t ch;

  for (ch = 0u; ch < ADC_CH_NUM; ch++)
  {
    movavg_init(&g_ma[ch]);
    movavg_init(&g_flt_hist[ch]);

    /* 첫 샘플이 곧바로 둘 다 갱신하도록 반대쪽 끝으로 초기화 */
    psd_obs_min[ch] = 0xFFFFFFFFu;
    psd_obs_max[ch] = 0u;
  }

  HAL_TIM_Base_Start_IT(&htim8);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, ADC_CH_NUM);
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* flag 가 설 때만 읽어야 4채널이 같은 주기의 값으로 맞는다 */
    if (adc_updated_flag)
    {
      adc_updated_flag = 0;

      for (ch = 0u; ch < ADC_CH_NUM; ch++)
      {
        adc_raw[ch]      = adc1_buffer[ch];
        adc_filtered[ch] = movavg_update(&g_ma[ch], adc_raw[ch]);
        psd_norm[ch]     = psd_normalize(adc_filtered[ch]);

        /* 평균은 쓰지 않고 이력만 쌓아서 변동폭을 잰다 */
        (void)movavg_update(&g_flt_hist[ch], adc_filtered[ch]);

        adc_raw_pp[ch] = window_pp(&g_ma[ch]);
        adc_flt_pp[ch] = window_pp(&g_flt_hist[ch]);

        if (adc_filtered[ch] < psd_obs_min[ch]) { psd_obs_min[ch] = adc_filtered[ch]; }
        if (adc_filtered[ch] > psd_obs_max[ch]) { psd_obs_max[ch] = adc_filtered[ch]; }
      }
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == hadc1.Instance)
  {
    adc_updated_flag = 1;
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
