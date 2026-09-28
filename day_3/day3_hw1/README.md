## HW_1

---

이규환 2026406017

---

### 설정 요약

`TIM1 (180MHz) → Prescaler 9 → 20MHz → ARR 1000 → 20kHz PWM (PA8) → 모터 드라이버 PWM`

| 항목 | 값 | 이유 |
|---|---|---|
| Timer | TIM1 CH1 PWM Generation | APB2(180MHz) 타이머라 분주 여유가 큼 |
| Prescaler | `9-1` | 180MHz / 9 = 20MHz |
| Counter Period (ARR) | `1000-1` | 20MHz / 1000 = 20kHz, CCR 0~1000 이 곧 duty 0~100% |
| PWM 주파수 | 20kHz | 모터 드라이버 권장값 (가청 대역 밖이라 소음이 적음) |
| Vin | 12V | 과제 조건 |

**핀 배치**

| 기능 | 핀 |
|---|---|
| PWM → 드라이버 | `PA8` (TIM1_CH1) |
| DIR → 드라이버 | `PC8` |

강의 슬라이드 예시는 PA8 = GPIO(DIR), PC8 = TIM8_CH3(PWM) 이지만, 교육용 보드 배선은 **PA8 이 드라이버 PWM, PC8 이 DIR(Motor_Direction_R)** 이라 보드 기준으로 설정했다. duty 0 에서 모터가 멈추는 것으로 배선이 맞음을 확인했다.

---

### 1. 과제 요구사항

PWM 주기 20kHz 생성, duty 0%, 30%, 50%, 70%

---

### 2. 구현

#### 2.1 주파수 계산

```
180,000,000 / 9 / 1000 = 20,000 Hz
```

ARR 을 1000 으로 잡은 이유는 계산이 쉬워서다. CCR 값을 10 으로 나누면 바로 duty(%)가 된다.

| duty | CCR1 |
|---|---|
| 0% | 0 |
| 30% | 300 |
| 50% | 500 |
| 70% | 700 |

#### 2.2 PWM 시작과 duty 변경

```c
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  // 방향 핀 HIGH 고정 (CW)
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);            // TIM1_CH1(PA8) 20kHz PWM 시작
```

```c
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300);   // duty 30%
HAL_Delay(2000);
```

duty 는 `__HAL_TIM_SET_COMPARE` 로 CCR1 만 바꾸면 된다. 주기(ARR)는 그대로라 주파수는 20kHz 로 유지되고 High 구간 길이만 달라진다. 0 → 30 → 50 → 70% 를 2초씩 반복한다.

방향 핀을 먼저 HIGH 로 고정한 이유는, 초기화 직후 PC8 이 LOW 로 잡혀 있어서 방향이 보드 상태에 따라 달라지지 않게 하기 위해서다.

---

### 3. 결론

TIM1 하나로 20kHz PWM 을 만들고 CCR 만 바꿔 duty 0 / 30 / 50 / 70% 를 출력했다. ARR 을 1000 으로 잡아두니 이후 과제에서도 `duty(%) × 10` 으로 바로 CCR 을 계산할 수 있었다.

---

**Demo Video** : https://drive.google.com/file/d/1tNVt3dDOjVx0EPpsIjk2G0fWvY6xNPmJ/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c)
