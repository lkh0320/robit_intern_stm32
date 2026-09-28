## HW_4

---

이규환 2026406017

---

### 설정 요약

`duty_percent → TIM1 PWM → 모터 → 엔코더 A/B → TIM3 Encoder CNT → encoder_count / encoder_speed → STM32CubeMonitor`

| 항목 | 값 | 이유 |
|---|---|---|
| TIM3 Mode | Encoder Mode TI1 and TI2 | A/B 상 양쪽 엣지를 모두 세는 4체배 |
| Counter Period | `65535` | 16bit 최대, 카운터를 최대한 늦게 넘치게 |
| Input Filter | `5` | 엔코더 신호 노이즈 제거 |
| GPIO Pull | Pull-up | 강의자료 설정, 신호선이 떠 있을 때 값이 흔들리지 않게 |
| PWM | TIM1 CH1, 20kHz | HW_1 과 동일 |

**핀 배치**

| 기능 | 핀 |
|---|---|
| PWM → 드라이버 | `PA8` |
| DIR → 드라이버 | `PC8` (CW 고정) |
| 엔코더 A / B | `PB4` (TIM3_CH1) / `PB5` (TIM3_CH2) |

강의 슬라이드 예시는 PA8 = GPIO(DIR), PC8 = TIM8_CH3(PWM) 이지만, 교육용 보드 배선은 **PA8 이 드라이버 PWM, PC8 이 DIR(Motor_Direction_R)** 이라 보드 기준으로 설정했다. duty 0 에서 모터가 멈추는 것으로 배선이 맞음을 확인했다.

---

### 1. 과제 요구사항

STM32CubeMonitor 로 모터 Encoder 값을 받아 duty 와 비교, Encoder overflow / underflow 해결

---

### 2. 구현

#### 2.1 overflow / underflow 해결

`TIM3->CNT` 는 16bit 라 65535 다음이 0 이다. CNT 를 그대로 보면 모터가 계속 돌아도 값이 톱니처럼 떨어진다.

```c
int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
encoder_count += ENC_DIR * (int16_t)(cnt - prev_cnt);  // int16 차이로 누적 -> overflow/underflow 해결
prev_cnt = cnt;
```

CNT 값 자체가 아니라 **한 반복(약 2ms) 동안의 차이**만 int16_t 로 구해서 int32_t 에 더한다. 핵심은 int16_t 로 자르는 순간 넘어간 경우가 저절로 보정된다는 것이다.

```
65530 → 5   :  5 - 65530 = -65525  → int16_t 로 자르면  +11  (overflow)
5 → 65530   :  65530 - 5 =  65525  → int16_t 로 자르면  -11  (underflow)
```

한 반복 안에 32767 카운트 이상 움직이지만 않으면 항상 맞다. 누적값은 int32_t 라 약 21억까지 간다.

모터가 정방향으로 돌 때 카운트가 줄어드는 방향으로 배선돼 있어서 `ENC_DIR = -1` 로 부호를 맞췄다.

#### 2.2 속도 계산

```c
if (++speed_cnt >= 10)  // 10회(약 20ms)마다 속도 계산
{
  speed_cnt = 0;
  encoder_speed = encoder_count - last_count;
  last_count = encoder_count;
}
```

`encoder_speed` 는 약 20ms 동안 늘어난 카운트다. 누적값의 차이로 구하기 때문에 여기서는 overflow 를 다시 신경 쓸 필요가 없다.

#### 2.3 duty 변경

| 변수 | 의미 |
|---|---|
| `auto_mode` | 1: duty 를 0 → 30 → 50 → 70% 로 약 4초씩 자동 변경, 0: 직접 입력 |
| `duty_percent` | 현재 duty (%), CubeMonitor write panel 에서 변경 가능 (최대 70%) |

```c
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_percent * 10);  // % -> CCR (ARR 1000)
```

CubeMonitor 가 쓴 값을 매 반복(약 2ms)마다 확인해서 반영하고, 70% 를 넘으면 잘라낸다.

#### 2.4 실제 주기가 2배인 이유

```c
wait += (uint32_t)(uwTickFreq);  // HAL_Delay 내부: 최소 대기 보장을 위해 1틱 추가
```

`RunFor()` 는 반복마다 `HAL_Delay(1)` 을 부르는데, HAL 이 최소 대기를 보장하려고 1틱을 더하기 때문에 실제로는 **반복당 약 2ms** 다. 그래서 코드상 1ms / 10ms / 2초로 의도한 값이 실제로는 약 2ms / 20ms / 4초가 된다. 비교 실험에는 지장이 없어 코드는 그대로 두고 수치를 실제 값으로 적었다. 정확한 주기가 필요하면 TIM6 인터럽트로 바꾸면 된다.

---

### 3. 검증

STM32CubeMonitor 차트로 확인했다.

| 차트 | 확인한 것 |
|---|---|
| `duty_percent` + `encoder_speed` | duty 계단(0/30/50/70, 약 4초씩)에 맞춰 속도도 계단 형태로 증가 |
| `encoder_count` | 65535 를 넘어도 끊기지 않고 계속 증가 → overflow 해결 |

---

### 4. 시행착오

처음에 네 변수를 한 차트에 모두 올렸더니 **encoder_speed 가 계속 0 처럼 평평하게** 나와서 엔코더가 안 읽히는 줄 알았다. 실제로는 `encoder_count` 가 -200,000 까지 내려가면서 Y축이 그 범위로 잡혀, 수십 단위인 `encoder_speed` 와 `duty_percent` 가 0 선에 붙어 보였던 것이다.

`encoder_count` 의 기울기가 duty 가 바뀔 때마다 달라지는 것을 보고 엔코더는 정상이라는 걸 알았다. 스케일이 비슷한 `duty_percent` 와 `encoder_speed` 만 같은 차트에 두어 해결했다. 이때 카운트가 음수로 줄어드는 것도 발견해서 `ENC_DIR` 을 추가했다.

---

### 5. 결론

16bit 카운터를 **값이 아니라 차이로 읽는** 방식으로 overflow / underflow 를 해결했다. 넘어가는 순간을 따로 감지하는 조건문 없이, int16_t 형변환 하나로 양방향이 모두 처리된다는 점이 핵심이었다.

---

**Demo Video** : https://drive.google.com/file/d/1wfS0kkiSn95XOO3Fl8F5d_1Za7M8XY44/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c)
