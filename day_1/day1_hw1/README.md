## HW_1

---

이규환 2026406017

---

### 설정 요약

| 항목 | 값 |
|---|---|
| MCU | STM32F446RET6 (LQFP64) |
| 클럭 | PLL → SYSCLK 180MHz (APB1 45MHz / APB2 90MHz) |
| 개발환경 | STM32CubeMX 6.18.1 + STM32CubeIDE (Ubuntu) |
| 디버거 | ST-Link / SWD (PA13, PA14) |

**핀 배치**

| 기능 | 핀 | 모드 |
|---|---|---|
| 단독 TOGGLE LED | `PD2` | GPIO_Output, Push-Pull |
| Shift LED | `PB0`, `PB1`, `PB2`, `PB10` | GPIO_Output, Push-Pull |
| SW — 우측 이동 | `PB12` | GPIO_Input |
| SW — ON/OFF 반전 | `PB13` | GPIO_Input |
| SW — shift 2개 | `PB14` | GPIO_Input |
| SW — PD2 일시정지 | `PB15` | GPIO_Input |
| 디버그 | `PA13`(SWDIO), `PA14`(SWCLK) | SYS: Serial Wire |

SWD 를 쓴 이유는 JTAG 이 디버깅에 4~5핀을 요구하는 반면 SWD 는 SWCLK / SWDIO 2핀이면 되기 때문이다. 남는 핀을 GPIO 로 쓸 수 있다.

**신호 극성** — 회로도 없이 시작해서 실측으로 확정했다. 스위치와 LED 모두 active-low 였다.

```c
#define SW_PRESSED   GPIO_PIN_RESET
#define LED_ON       GPIO_PIN_RESET
#define LED_OFF      GPIO_PIN_SET
```

---

### 1. 과제 요구사항

1. PD2 LED TOGGLE
2. LED `PB0, PB1, PB2, PB10` 좌측 이동
3. 스위치 `PB12` 누르면 LED 우측 이동
4. 스위치 `PB13` 누르면 LED ON/OFF 반전
5. 스위치 `PB14` 누르면 한번에 shift 되는 LED 개수 2개
6. 스위치 `PB15` 누르면 PD2 TOGGLE 일시정지 (TOGGLE 직전 상태 유지)

**\* 모든 스위치는 동시에 누를 수 있음**

이 마지막 조건이 이번 과제의 핵심이라고 판단했다. 요구사항 3~6 을 각각 따로 구현하는 것은 어렵지 않지만, 넷이 동시에 성립해야 한다는 제약 때문에 구조를 처음부터 다르게 잡아야 했다.

**5번 요구사항 해석** — "한번에 shift 되는 **LED 개수** 2개" 라는 표현에 따라 동시에 점등되는 LED 가 2개가 되는 것으로 해석했다.

```
평소          1000 → 0100 → 0010 → 0001
PB14 누름     1100 → 0110 → 0011 → 1001
```

"2칸씩 건너뛴다"는 해석도 가능하지만, LED 가 4개뿐이라 그 경우 `1000 → 0010 → 1000` 두 위치만 왕복하는 단조로운 동작이 된다. 코드에는 두 해석을 한 줄로 전환할 수 있게 매크로(`SW14_MEANS_LED_COUNT`)를 두었다.

---

### 2. 구현

#### 2.1 스위치를 if/else 체인으로 묶으면 안 되는 이유

가장 먼저 떠오르는 구현은 이런 형태다.

```c
if (PB12 눌림)      { 우측 이동; }
else if (PB13 눌림) { 반전; }
else if (PB14 눌림) { 2개 shift; }
```

이렇게 짜면 **먼저 걸린 조건 하나만 반영**된다. PB12 와 PB13 을 같이 누르면 반전이 무시되어 "모든 스위치는 동시에 누를 수 있음" 조건을 만족할 수 없다.

그래서 네 스위치를 **서로 직교하는 4개의 독립된 축**으로 보았다.

| 스위치 | 제어하는 축 | 값 |
|---|---|---|
| PB12 | 진행 방향 | 좌 / 우 |
| PB13 | 출력 반전 | 정상 / 반전 |
| PB14 | 동시 점등 개수 | 1 / 2 |
| PB15 | PD2 토글 | 진행 / 정지 |

축이 서로 독립이므로 네 개를 다 눌러도 네 효과가 전부 동시에 적용된다. 매 루프마다 네 스위치를 각각 따로 읽어 독립된 변수에 담았다.

```c
uint8_t sw_right  = sw_pressed(SW_RIGHT_PIN);
uint8_t sw_invert = sw_pressed(SW_INVERT_PIN);
uint8_t sw_wide   = sw_pressed(SW_WIDE_PIN);
uint8_t sw_pause  = sw_pressed(SW_PAUSE_PIN);
```

#### 2.2 LED 를 "핀"이 아니라 "위치 + 패턴"으로 관리

핀 5개를 직접 켜고 끄는 방식으로는 방향·개수·반전이 서로 얽힌다. 그래서 상태를 두 단계로 나눴다.

```
led_pos (0~3)  ──build_pattern(pos, width)──▶  4비트 패턴  ──led_write()──▶  실제 핀
    ▲                                              ▲
  방향·step 이 바꿈                          width·invert 가 바꿈
```

- `led_pos` : 현재 패턴의 시작 위치. shift 주기마다 `±step`
- `build_pattern(pos, width)` : 그 위치부터 `width` 개를 켠 비트 패턴 생성 (끝을 넘으면 앞으로 순환)
- `led_write(pattern)` : 비트 패턴을 실제 핀으로 출력

이 구조 덕분에 PB14 로 점등 개수가 1 ↔ 2 로 바뀌어도 위치 정보(`led_pos`)가 유지되어 패턴이 끊기지 않고 자연스럽게 이어진다.

또한 논리 인덱스와 실제 핀을 배열로 분리해두어, 보드의 LED 물리 배치가 예상과 반대였을 때 배열 순서만 뒤집어 해결했다. 방향 계산 로직은 한 줄도 고치지 않았다.

```c
/* index 0 = 오른쪽 끝(PB10), index 3 = 왼쪽 끝(PB0) */
static const uint16_t LED_PIN[LED_NUM] = { GPIO_PIN_10, GPIO_PIN_2,
                                           GPIO_PIN_1,  GPIO_PIN_0 };
```

#### 2.3 HAL_Delay() 대신 HAL_GetTick() 비교

`HAL_Delay(200)` 을 쓰면 그 200ms 동안 CPU 가 멈춰서 **스위치를 아예 읽지 못한다.** 사용자 입장에서는 버튼을 눌러도 최대 200ms 뒤에야 반응하는 것처럼 보인다.

```c
uint32_t now = HAL_GetTick();

if ((now - t_shift) >= LED_SHIFT_PERIOD_MS)
{
  t_shift = now;
  /* 위치만 갱신 */
}
```

비차단(non-blocking) 방식으로 바꿔서 루프가 계속 돌며 스위치를 읽게 했다. shift 주기 200ms 와 PD2 토글 주기 500ms 를 서로 간섭 없이 동시에 운용할 수 있다.

#### 2.4 LED 출력은 매 루프, PD2 출력은 토글 시에만

두 출력의 성격이 다르다는 점을 이용했다.

- **Shift LED** : 매 루프마다 `led_write()` 호출
  → PB13(반전), PB14(개수)가 다음 shift 를 기다리지 않고 **누르는 즉시** 반영된다.
- **PD2** : 토글하는 순간에만 `HAL_GPIO_WritePin()` 호출
  → PB15 를 누르는 동안에는 쓰기 자체가 일어나지 않으므로, 별도 상태 저장 없이 **직전 상태가 그대로 유지**된다 (요구사항 6번).

```c
if (sw_pause)
{
  t_toggle = now;   /* 기준점을 밀어둬야 손 뗀 순간 밀린 만큼 즉시 토글되지 않는다 */
}
else if ((now - t_toggle) >= PD2_TOGGLE_PERIOD_MS)
{
  t_toggle  = now;
  pd2_state = (pd2_state == LED_ON) ? LED_OFF : LED_ON;
  HAL_GPIO_WritePin(PD2_PORT, PD2_PIN, pd2_state);
}
```

일시정지 중 기준점을 계속 `now` 로 밀어두는 처리를 넣었다. 이게 없으면 손을 떼는 순간 정지해 있던 시간만큼 밀린 것으로 계산되어 LED 가 즉시 튀듯 토글된다.

---

### 3. 시행착오

회로도가 없어서 처음에는 스위치와 LED 를 모두 active-high 로 가정하고 `SW_PRESSED` 를 `GPIO_PIN_SET` 으로 두었다. 그 결과 아무것도 누르지 않은 상태가 "전부 눌림"으로 읽혀서, PD2 가 처음부터 정지해 있고 LED 가 기본 2개로 움직였다. 극성을 `#define` 으로 분리해둔 덕분에 한 줄 수정으로 전체가 해결됐다.

---

### 4. 결론

요구사항 6개를 모두 만족하며, 스위치 4개를 임의로 조합해 눌러도 각 효과가 독립적으로 적용된다.

이번 과제에서 얻은 것은 **"동시에 누를 수 있다"는 한 줄의 조건이 구현 구조 전체를 결정한다**는 경험이다. 조건 분기(if/else)로 접근하면 조합 수만큼 경우의 수가 폭발하지만, 서로 직교하는 상태 변수로 분해하면 스위치가 늘어나도 축이 하나 추가될 뿐이다.

또한 하드웨어 의존 정보(극성, 핀 배치)를 상수와 배열로 분리해두니 보드 특성이 예상과 달랐을 때 로직을 건드리지 않고 한 줄 수정으로 대응할 수 있었다.

---

**Demo Video** : https://drive.google.com/file/d/1iL3CZq1XE5EihXpJR3DVHTCF2LT4j8Gw/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c)
