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
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256

#define DXL_ID              14
#define DXL_ADDR_TORQUE_EN  64
#define DXL_ADDR_GOAL_POS   116 // 4byte, 0~4095 이 360도
#define DXL_INST_WRITE      0x03
#define DXL_INST_STATUS     0x55
#define DXL_PARAM_MAX       16u

#define SW_PRESSED          GPIO_PIN_RESET
#define SW_NUM              3u

#define LED_ON              GPIO_PIN_RESET
#define LED_OFF             GPIO_PIN_SET
#define LED_OK_PIN          GPIO_PIN_0 // 응답 정상
#define LED_NG_PIN          GPIO_PIN_1 // 응답 없음 또는 에러
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t PCrxBuffer[RX_BUFFER_SIZE];

static const uint16_t SW_PIN[SW_NUM]   = { GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14 };
static const uint32_t SW_GOAL[SW_NUM]  = { 1024u, 2048u, 0u }; // 90도, 180도, 0도
static uint8_t sw_prev[SW_NUM]         = { 0u }; // 누른 순간만 잡기 위한 직전 상태

uint8_t  rxPacket[RX_BUFFER_SIZE]; // IDLE 인터럽트에서 복사해둔 수신 패킷
volatile uint16_t rxLen  = 0u;
volatile uint8_t  rxFlag = 0u;

volatile uint8_t  dxlError    = 0u; // Status Packet 의 ERR 바이트
volatile uint16_t dxlParamLen = 0u;
volatile uint32_t dxlOkCount  = 0u; // 정상 응답 횟수
volatile uint32_t dxlNgCount  = 0u; // 파싱 실패 횟수
uint8_t  dxlParam[DXL_PARAM_MAX];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void writepacket(uint8_t packet[], int length);
static uint16_t dxlCrc(const uint8_t *data, uint16_t size);
static void dxlWrite(uint8_t id, uint16_t addr, const uint8_t *param, uint16_t n);
static void dxlTorque(uint8_t id, uint8_t on);
static void dxlGoalPosition(uint8_t id, uint32_t pos);
static uint8_t dxlParseStatus(const uint8_t *buf, uint16_t n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1,(uint32_t)PCrxBuffer);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1,(uint32_t)&USART3->DR);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, RX_BUFFER_SIZE);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableIT_IDLE(USART3);

  HAL_GPIO_WritePin(GPIOB, LED_OK_PIN | LED_NG_PIN, LED_OFF);

  HAL_Delay(100);
  dxlTorque(DXL_ID, 1); // 토크를 켜야 Goal Position 이 먹는다
  HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t i;

    for (i = 0u; i < SW_NUM; i++)
    {
      if (HAL_GPIO_ReadPin(GPIOB, SW_PIN[i]) == SW_PRESSED)
      {
        if (sw_prev[i] == 0u) // 누른 순간에만 1회 전송
        {
          sw_prev[i] = 1u;
          dxlGoalPosition(DXL_ID, SW_GOAL[i]);
        }
      }
      else
      {
        sw_prev[i] = 0u;
      }
    }

    if (rxFlag != 0u)
    {
      rxFlag = 0u;

      if (dxlParseStatus(rxPacket, rxLen) != 0u)
      {
        dxlOkCount++;
        HAL_GPIO_WritePin(GPIOB, LED_OK_PIN, (dxlError == 0u) ? LED_ON : LED_OFF);
        HAL_GPIO_WritePin(GPIOB, LED_NG_PIN, (dxlError == 0u) ? LED_OFF : LED_ON);
      }
      else
      {
        dxlNgCount++;
        HAL_GPIO_WritePin(GPIOB, LED_OK_PIN, LED_OFF);
        HAL_GPIO_WritePin(GPIOB, LED_NG_PIN, LED_ON);
      }
    }

    HAL_Delay(10);
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
void writepacket(uint8_t packet[], int length){
	for (int i = 0; i < length; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART3));
		LL_USART_TransmitData8(USART3, packet[i]);
	}
	while(!LL_USART_IsActiveFlag_TC(USART3)); // 반이중이라 마지막 바이트까지 내보내고 버스를 놓는다
}

static uint16_t dxlCrc(const uint8_t *data, uint16_t size) // Protocol 2.0 CRC-16, poly 0x8005
{
  uint16_t crc = 0u;
  uint16_t i;
  uint8_t  b;

  for (i = 0u; i < size; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (b = 0u; b < 8u; b++)
    {
      crc = (crc & 0x8000u) ? (uint16_t)((uint16_t)(crc << 1) ^ 0x8005u)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void dxlWrite(uint8_t id, uint16_t addr, const uint8_t *param, uint16_t n)
{
  uint8_t  packet[16];
  uint16_t len = (uint16_t)(n + 5u); // 주소 2 + 데이터 n + Instruction 1 + CRC 2
  uint16_t crc;
  uint16_t i;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0xFD;
  packet[3] = 0x00;
  packet[4] = id;
  packet[5] = (uint8_t)(len & 0xFFu);
  packet[6] = (uint8_t)(len >> 8);
  packet[7] = DXL_INST_WRITE;
  packet[8] = (uint8_t)(addr & 0xFFu);
  packet[9] = (uint8_t)(addr >> 8);

  for (i = 0u; i < n; i++)
  {
    packet[10 + i] = param[i];
  }

  crc = dxlCrc(packet, (uint16_t)(10u + n));
  packet[10 + n] = (uint8_t)(crc & 0xFFu);
  packet[11 + n] = (uint8_t)(crc >> 8);

  writepacket(packet, (int)(12u + n));
}

static void dxlTorque(uint8_t id, uint8_t on)
{
  uint8_t d = on;

  dxlWrite(id, DXL_ADDR_TORQUE_EN, &d, 1u);
}

static void dxlGoalPosition(uint8_t id, uint32_t pos)
{
  uint8_t d[4];

  d[0] = (uint8_t)(pos);
  d[1] = (uint8_t)(pos >> 8);
  d[2] = (uint8_t)(pos >> 16);
  d[3] = (uint8_t)(pos >> 24);
  dxlWrite(id, DXL_ADDR_GOAL_POS, d, 4u);
}

static uint8_t dxlParseStatus(const uint8_t *buf, uint16_t n)
{
  uint16_t i;
  uint16_t j;
  uint16_t len;
  uint16_t total;
  uint16_t crc;

  for (i = 0u; (uint32_t)i + 11u <= (uint32_t)n; i++) // 앞에 송신 에코가 섞일 수 있어 스캔한다
  {
    if (buf[i] != 0xFF || buf[i + 1] != 0xFF || buf[i + 2] != 0xFD || buf[i + 3] != 0x00)
    {
      continue;
    }
    if (buf[i + 4] != DXL_ID || buf[i + 7] != DXL_INST_STATUS)
    {
      continue;
    }

    len = (uint16_t)buf[i + 5] | (uint16_t)((uint16_t)buf[i + 6] << 8);
    if (len < 4u)
    {
      continue;
    }

    total = (uint16_t)(len + 7u);
    if ((uint32_t)i + (uint32_t)total > (uint32_t)n)
    {
      continue;
    }

    crc = dxlCrc(&buf[i], (uint16_t)(total - 2u));
    if (crc != (uint16_t)((uint16_t)buf[i + total - 1] << 8 | buf[i + total - 2])) // CRC 재계산해서 대조
    {
      continue;
    }

    dxlError    = buf[i + 8];
    dxlParamLen = (uint16_t)(len - 4u);
    if (dxlParamLen > DXL_PARAM_MAX)
    {
      dxlParamLen = DXL_PARAM_MAX;
    }
    for (j = 0u; j < dxlParamLen; j++)
    {
      dxlParam[j] = buf[i + 9u + j];
    }
    return 1u;
  }

  return 0u;
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
