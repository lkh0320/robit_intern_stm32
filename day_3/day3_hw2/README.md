## HW_2

---

이규환 2026406017

---

### 설정 요약

`motor_value (double, -1 ~ 1) → 부호: PC8 방향 / 크기: CCR1 duty → 모터 드라이버`

| 항목 | 값 | 이유 |
|---|---|---|
| PWM | TIM1 CH1, 20kHz | HW_1 과 동일 |
| DUTY_MAX | `700` | ±1 일 때 duty 70% (과제 조건) |
| STEP_MS | `500` | 0.5초마다 motor_value 를 0.1씩 변경 |

**핀 배치**

| 기능 | 핀 |
|---|---|
| PWM → 드라이버 | `PA8` |
| DIR → 드라이버 | `PC8` (LOW = CCW, HIGH = CW) |

강의 슬라이드 예시는 PA8 = GPIO(DIR), PC8 = TIM8_CH3(PWM) 이지만, 교육용 보드 배선은 **PA8 이 드라이버 PWM, PC8 이 DIR(Motor_Direction_R)** 이라 보드 기준으로 설정했다. duty 0 에서 모터가 멈추는 것으로 배선이 맞음을 확인했다.

---

### 1. 과제 요구사항

double 형 변수를 정의하고 그 값에 따라 모터 돌리기

| motor_value | 동작 |
|---|---|
| -1 | CCW, duty 70% |
| 0 | 정지 |
| 1 | CW, duty 70% |

---

### 2. 구현

#### 2.1 값 하나로 방향과 속도를 같이 결정

```c
if (value < 0.0)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // 음수: CCW
  duty = -value * DUTY_MAX;
}
else
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);    // 양수: CW
  duty = value * DUTY_MAX;
}
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty);
```

**부호는 방향, 크기는 속도**로 나눴다. 모터 드라이버가 PWM(크기) 과 DIR(방향) 을 따로 받기 때문에 그 구조에 맞춘 것이다. 0.5 면 CW 35%, -0.5 면 CCW 35% 처럼 값에 비례해서 속도가 나온다.

계산 전에 값을 -1 ~ 1 로 잘라서, 실수로 범위 밖 값이 들어가도 70% 를 넘지 않게 했다.

#### 2.2 0.1 씩 자동으로 바꾸기

```c
if (HAL_GetTick() - last_tick >= STEP_MS)  // 0.5초마다 0.1씩 변경
{
  last_tick += STEP_MS;
  step += step_dir;
  if (step >= 10) step_dir = -1;
  if (step <= -10) step_dir = 1;
  motor_value = step / 10.0;
}
```

`motor_value += 0.1` 로 바로 더하지 않고 정수 `step` 을 쓴 이유는 **0.1 이 2진수로 정확히 표현되지 않기 때문**이다. 계속 더하면 `0.30000000000000004` 같은 오차가 쌓여서 `== 1.0` 비교가 안 맞거나 1 을 살짝 넘을 수 있다. 정수로 세고 마지막에 10 으로 나누면 항상 정확한 0.1 단위가 나온다.

```
0 → 0.1 → … → 1.0 → 0.9 → … → -1.0 → … → 0  (한 바퀴 20초)
```

---

### 3. 검증

Live Expressions 에 `motor_value` 를 올려두고 값이 바뀔 때마다 모터 속도와 방향이 같이 바뀌는지 확인했다. 0 을 지나는 순간 회전 방향이 바뀌고, ±1 에서 가장 빠르다.

---

### 4. 시행착오

처음에는 Live Expressions 에서 `motor_value` 에 값을 직접 넣어 테스트했는데, **1, 0, -1 은 되는데 0.5 를 넣으면 `5.29e-315` 같은 값**이 들어갔다.

0.5 의 double 비트는 `0x3FE0000000000000` 인데, 위쪽 32비트 `0x3FE00000` 만 아래쪽 자리에 써진 값이 딱 `5.29e-315` 였다. 소수를 쓸 때 64bit double 이 32bit 로만 써지는 것으로 추정된다. 정수는 정상으로 써졌는데, 이 차이의 정확한 원인은 확인하지 못했다.

그래서 값을 **넣는 것은 코드에서 자동으로**, Live Expressions 는 **보는 용도로만** 쓰도록 바꿨다. 읽기는 정상으로 동작한다.

---

### 5. 결론

double 변수 하나로 방향(부호)과 속도(크기)를 동시에 제어했다. 부동소수점을 다룰 때는 **누적 오차**와 **디버거가 값을 쓰는 방식**까지 신경 써야 한다는 것을 확인했다.

---

**Demo Video** : https://drive.google.com/file/d/17C_DqjmJ5X0cX4WzR-7EA8eQE13yT9fn/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c)
