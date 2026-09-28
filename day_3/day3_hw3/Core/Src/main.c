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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS_HOST_ID 0xFD  // 호스트(STM32) ID
#define RS_MOTOR_ID_DEFAULT 14  // RS00 설정 ID

#define RS_TYPE_STOP 4  // 통신 타입: 정지
#define RS_TYPE_ENABLE 3  // 통신 타입: 활성화
#define RS_TYPE_WRITE 18  // 통신 타입: 파라미터 쓰기
#define RS_TYPE_ID_REQUEST 0  // 통신 타입: ID 요청

#define RS_IDX_RUN_MODE 0x7005  // 운전 모드 (2 = 속도 모드)
#define RS_IDX_SPD_REF 0x700A  // 목표 속도 (rad/s)
#define RS_IDX_LIMIT_CUR 0x7018  // 전류 제한 (A)
#define RS_IDX_ACC_RAD 0x7022  // 가속도 (rad/s^2)

#define RS_SPEED_RAD_S 5.0f  // 데이터시트 최대값의 절반 미만
#define RS_LIMIT_CUR_A 4.0f  // 데이터시트 최대값의 절반 미만
#define RS_ACC_RAD_S2 20.0f
#define RS_PHASE_MS 5000  // 방향 전환 주기 (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t motor_id = RS_MOTOR_ID_DEFAULT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void RS_Send(uint8_t comm_type, const uint8_t data[8]);
static void RS_Scan(void);
static void RS_WriteU8(uint16_t index, uint8_t value);
static void RS_WriteFloat(uint16_t index, float value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void RS_Send(uint8_t comm_type, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef header;
  uint32_t mailbox;
  uint32_t start;

  header.ExtId = ((uint32_t)comm_type << 24) | ((uint32_t)RS_HOST_ID << 8) | motor_id;  // 29bit ID = 타입 | 호스트 ID | 모터 ID
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;

  start = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)  // 빈 메일박스 대기 (10ms 제한)
  {
    if (HAL_GetTick() - start > 10)
    {
      return;
    }
  }
  HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox);
}

static void RS_Scan(void)
{
  uint8_t zero[8] = {0};
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint16_t n;

  for (n = 0; n < 127; n++)
  {
    motor_id = (n == 0) ? RS_MOTOR_ID_DEFAULT : n;
    RS_Send(RS_TYPE_ID_REQUEST, zero);  // ID 요청 후 응답 확인
    HAL_Delay(3);
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
      HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
      if (((rx_header.ExtId >> 24) & 0x1F) == 0)
      {
        motor_id = (rx_header.ExtId >> 8) & 0xFF;  // 응답한 모터 ID 저장
        return;
      }
    }
  }
  motor_id = RS_MOTOR_ID_DEFAULT;
}

static void RS_WriteU8(uint16_t index, uint8_t value)
{
  uint8_t data[8] = {0};
  memcpy(&data[0], &index, 2);  // Byte0~1: 파라미터 index
  data[4] = value;  // Byte4: 값
  RS_Send(RS_TYPE_WRITE, data);
}

static void RS_WriteFloat(uint16_t index, float value)
{
  uint8_t data[8] = {0};
  memcpy(&data[0], &index, 2);  // Byte0~1: 파라미터 index
  memcpy(&data[4], &value, 4);  // Byte4~7: float 값
  RS_Send(RS_TYPE_WRITE, data);
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef sFilterConfig = {0};  // 모든 ID 수신 필터
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
  HAL_CAN_Start(&hcan1);  // CAN 1Mbps 시작
  HAL_Delay(100);
  RS_Scan();  // 모터 ID 확인

  uint8_t zero[8] = {0};
  RS_Send(RS_TYPE_STOP, zero);  // 속도 모드 설정 순서: 정지
  HAL_Delay(50);
  RS_WriteU8(RS_IDX_RUN_MODE, 2);  // 속도 모드
  HAL_Delay(10);
  RS_Send(RS_TYPE_ENABLE, zero);  // 모터 활성화
  HAL_Delay(10);
  RS_WriteFloat(RS_IDX_LIMIT_CUR, RS_LIMIT_CUR_A);
  HAL_Delay(10);
  RS_WriteFloat(RS_IDX_ACC_RAD, RS_ACC_RAD_S2);
  HAL_Delay(10);
  RS_WriteFloat(RS_IDX_SPD_REF, 0.0f);  // 초기 정지
  uint32_t t_start = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t phase = ((HAL_GetTick() - t_start) / RS_PHASE_MS) % 2;  // 5초마다 0/1 전환
    RS_WriteFloat(RS_IDX_SPD_REF, (phase == 0) ? RS_SPEED_RAD_S : -RS_SPEED_RAD_S);  // 0: CW, 1: CCW
    HAL_Delay(20);
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
