## HW_1

---

이규환 2026406017

---

### 설정 요약

| 항목 | 값 |
|---|---|
| MCU | STM32F446RET6 (LQFP64) |
| 클럭 | PLL → SYSCLK 180MHz (APB1 45MHz / APB1 Timer 90MHz) |
| 개발환경 | STM32CubeMX 6.18.1 + STM32CubeIDE (Ubuntu) |
| 디버거 | ST-Link / SWD (PA13, PA14) |

**TIM6 설정**

| 항목 | 값 |
|---|---|
| Clock source | APB1 Timer clock 90MHz |
| Prescaler (PSC) | `90 - 1` |
| Counter Period (ARR) | `100 - 1` |
| NVIC | TIM6 global interrupt enable |

```
90,000,000 / 90 / 100 = 10,000Hz  →  인터럽트 1회 = 0.1ms
```

**핀 배치**

| 기능 | 핀 | 모드 |
|---|---|---|
| 상시 TOGGLE LED | `PD2` | GPIO_Output, Push-Pull |
| 스위치 연동 LED | `PB0`, `PB1`, `PB2`, `PB10` | GPIO_Output, Push-Pull |
| 스위치 | `PB12`, `PB13`, `PB14`, `PB15` | GPIO_Input |
| 디버그 | `PA13`(SWDIO), `PA14`(SWCLK) | SYS: Serial Wire |

**신호 극성** — day_1 에서 실측으로 확정한 대로 스위치와 LED 모두 active-low 다.

```c
#define SW_PRESSED   GPIO_PIN_RESET
#define LED_ON       GPIO_PIN_RESET
#define LED_OFF      GPIO_PIN_SET
```

---

### 1. 과제 요구사항

1. `PD2` LED TOGGLE (1초 주기 — 0.5초 켜지고 0.5초 꺼짐)
2. 스위치에 따라 LED TOGGLE (0.5초 주기)
   - `PB12` → `PB0`
   - `PB13` → `PB1`
   - `PB14` → `PB2`
   - `PB15` → `PB10`

**"주기" 해석** — 1번이 "1초 주기 = 0.5초 ON + 0.5초 OFF" 라고 명시하고 있으므로, 2번의 "0.5초 주기" 도 같은 뜻인 전체 사이클로 읽었다. 즉 토글 간격은 주기의 절반이다.

| 대상 | 주기 | 토글 간격 |
|---|---|---|
| PD2 | 1000ms | 500ms |
| 스위치 LED | 500ms | 250ms |

코드에서는 주기만 상수로 두고 토글 간격을 `/ 2` 로 계산해, 두 요구사항이 같은 규칙을 공유하게 했다.

```c
#define PD2_PERIOD_MS      1000u
#define SW_LED_PERIOD_MS    500u
```

---

### 2. 구현

#### 2.1 왜 `HAL_Delay()` 도 `HAL_GetTick()` 도 아닌 TIM6 인가

`HAL_Delay()` 는 그 시간 동안 CPU 가 멈춰서 스위치를 읽지 못한다. day_1 에서는 이를 `HAL_GetTick()` 비교로 바꿔 해결했지만, 그 방식도 **메인 루프가 얼마나 빨리 도느냐에 시간 정밀도가 따라간다.** 루프 안에 무거운 연산이 하나 들어가는 순간 토글 시점이 그만큼 밀린다.

TIM6 인터럽트는 메인 루프와 무관하게 하드웨어 카운터가 만들어내므로, 루프에서 무슨 일이 일어나든 주기가 흔들리지 않는다. 이번 과제는 메인 루프를 **비워두고** 모든 동작을 인터럽트 안에서 처리했다.

```c
int main(void)
{
  ...
  HAL_TIM_Base_Start_IT(&htim6);

  while (1)
  {
    /* 모든 동작은 TIM6 인터럽트에서 처리한다 */
  }
}
```

#### 2.2 0.1ms 타이머로 1ms 제어주기 만들기

TIM6 은 강의자료 설정 그대로 0.1ms 주기다. 그런데 이 과제가 다루는 시간 단위는 250ms / 500ms 라서 0.1ms 해상도가 필요 없고, 0.1ms 마다 스위치 4개를 읽는 것도 낭비다.

그래서 인터럽트 10회를 모아 **1ms 제어주기**를 만들고, 실제 로직은 1ms 함수 하나에 모았다.

```c
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
```

이렇게 해두면 `g_ms` 가 `HAL_GetTick()` 과 같은 ms 시간축이 되어, day_1 에서 쓰던 `(now - last) >= period` 패턴을 그대로 쓸 수 있다. 차이는 그 시간축을 SysTick 이 아니라 **내가 설정한 타이머가 만든다**는 점이다.

> ARR 을 `1000-1` 로 바꾸면 TIM6 이 직접 1ms 를 만들어 분주 코드가 필요 없다. 이번에는 강의자료의 0.1ms 설정을 그대로 두고 소프트웨어로 나누는 쪽을 택했다.

#### 2.3 스위치와 LED 를 배열로 짝짓기

네 쌍의 관계가 전부 동일하므로 `if` 를 네 번 쓰는 대신 인덱스로 묶었다. 스위치가 늘어나도 배열에 한 줄만 추가하면 된다.

```c
static const uint16_t SW_PIN [SW_NUM] = { GPIO_PIN_12, GPIO_PIN_13,
                                          GPIO_PIN_14, GPIO_PIN_15 };
static const uint16_t LED_PIN[SW_NUM] = { GPIO_PIN_0,  GPIO_PIN_1,
                                          GPIO_PIN_2,  GPIO_PIN_10 };
```

각 LED 가 자기 기준 시각 `t_sw[i]` 를 따로 가지므로 네 스위치를 동시에 눌러도 서로 간섭하지 않는다.

#### 2.4 손을 뗐을 때 — 쓰지 않으면 유지된다

스위치를 뗀 동안은 핀에 아무것도 쓰지 않는다. 그래서 별도의 상태 저장 없이 **직전 ON/OFF 가 그대로 남는다.**

```c
else
{
  t_sw[i] = g_ms; // 기준점만 밀어둠, 핀에 안 써서 직전 상태 유지
}
```

기준점을 계속 `g_ms` 로 밀어두는 처리가 핵심이다. 이게 없으면 뗀 채로 1초가 지난 뒤 다시 누르는 순간 밀린 시간이 한꺼번에 계산되어 LED 가 즉시 튀듯 토글된다. 기준점을 밀어두면 다시 누른 시점부터 정확히 250ms 뒤에 첫 토글이 일어난다.

#### 2.5 부팅 직후 초기화

CubeMX 가 생성한 `MX_GPIO_Init()` 은 출력 핀을 `GPIO_PIN_RESET` 으로 둔다. active-low 보드에서는 이게 곧 **켜진 상태**라 그대로 두면 부팅 직후 LED 5개가 전부 켜진 채로 시작한다. 초기화 직후 명시적으로 꺼주었다.

```c
HAL_GPIO_WritePin(PD2_PORT, PD2_PIN, LED_OFF);
for (i = 0u; i < SW_NUM; i++)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN[i], LED_OFF);
}
```

---

### 3. 결론

요구사항 2개를 모두 만족한다. PD2 는 스위치 입력과 무관하게 1초 주기를 유지하고, 스위치 4개는 각각 독립적으로 자기 LED 를 0.5초 주기로 토글한다.

day_1 과 비교했을 때 얻은 것은 **시간축을 어디서 만드느냐**의 차이다. `HAL_GetTick()` 방식은 시간을 "읽어서 비교"하는 것이라 루프 속도에 결과가 묶이지만, 타이머 인터럽트는 시간이 되면 코드를 "불러주는" 구조라 메인 루프가 비어 있든 바쁘든 주기가 보장된다. 제어 주기를 일정하게 유지해야 하는 로봇 제어에서 타이머를 쓰는 이유가 이것이라고 이해했다.

---

**Demo Video** : https://drive.google.com/file/d/1deWP7ENbp99kGDwXU51kYjxcMpiJBKfg/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c)
