## HW_2

---

이규환 2026406017

---

### 설정 요약

`TIM8(10ms) → TRGO → ADC1 이 IN0~IN3 순차 변환 → DMA2 Stream0(Circular) 이 buffer 로 복사`

| 항목 | 값 | 이유 |
|---|---|---|
| TIM8 Prescaler / Period | `180-1` / `10000-1` | 180MHz ÷ 180 = 1MHz, ÷ 10000 = 100Hz → 10ms |
| TIM8 Trigger Event | Update Event | 이 TRGO 가 ADC 변환 시작 신호 |
| ADC Scan Conversion | Enabled | IN0~IN3 을 순서대로 변환 |
| ADC Continuous | Disabled | 타이머가 변환 시점을 지시하므로 불필요 |
| Number Of Conversion | 4 | Rank 순서 = buffer 인덱스 순서 |
| Sampling Time | 15 Cycles | 4채널을 10ms 안에 변환 가능한 값 |
| DMA Mode / Data Width | Circular / Word | Word = 4byte, `uint32_t` 와 일치해야 buffer 가 안 깨짐 |

센서는 PSD 1개를 **PA0(ADC1_IN0)** 에만 연결했다. PA1~PA3 은 미연결이라 값에 의미가 없지만,
3단계 확인을 위해 코드는 4채널을 동일하게 처리한다.

---

### 1단계 : adc_updated_flag 없이 while(1) 내부에서 adc1_buffer[0] 을 그냥 계속 읽으면 어떤 문제가 생길 수 있는가?

DMA 는 CPU 와 무관하게 buffer 를 갱신하기 때문에, 그냥 읽으면 지금 읽은 값이 언제 쓰인 것인지 알 수 없다. 구체적으로 세 가지 문제가 생긴다.

**① 4채널의 시점이 어긋난다.** ADC 가 IN0 → IN3 순서로 변환하고 DMA 가 `buffer[0]` → `[3]` 에 하나씩 쓰는데, 그 도중에 읽으면 `buffer[0]` 은 이번 주기 값인데 `buffer[3]` 은 지난 주기 값인 상태를 읽게 된다. 센서 여러 개를 비교해서 판단하는 코드라면 서로 다른 시점의 값을 비교하는 셈이 된다.

**② 같은 값을 중복해서 읽는다.** 변환 주기는 10ms 인데 `while(1)` 은 180MHz 로 돈다. 한 주기 사이에 수만 번 읽게 되고, 이 값을 그대로 이동평균에 넣으면 윈도우가 전부 같은 값으로 채워져서 평균이 원본과 똑같아진다. 즉 필터가 아무 일도 하지 않는다.

**③ 컴파일러가 읽기를 없앨 수 있다.** `volatile` 없이 선언하면 "루프 안에서 이 배열을 바꾸는 코드가 없다"고 판단해 읽기를 레지스터에 캐싱한다. DMA 가 메모리를 바꿔도 CPU 는 옛날 값만 보게 된다. Debug 빌드(`-O0`)에서는 드러나지 않다가 Release(`-O2`)에서 터지는 유형이라, 코드에서는 `volatile uint32_t adc1_buffer[4]` 로 선언했다.

flag 는 "DMA 가 4채널을 전부 채운 직후"라는 **시점**을 알려준다. 그 순간에만 읽으면 위 문제가 해결된다. 변환 주기 10ms 에 비해 복사 루프는 수 µs 라 여유도 충분하다.

---

### 2단계

#### 2.1 PSD 정규화 (Min-Max Normalization)

`(raw - MIN) / (MAX - MIN) * 1000` 으로 0~1000 스케일로 정규화했다.

부동소수점 대신 `(raw - MIN)` 을 먼저 1000배 한 뒤 나누는 정수 연산으로 처리했고, 구간을 벗어난 입력은 0 / 1000 으로 클램프해서 음수나 1000 초과가 나오지 않게 했다.

```c
static uint16_t psd_normalize(uint32_t raw)
{
  if (raw <= PSD_RAW_MIN) { return 0u; }
  if (raw >= PSD_RAW_MAX) { return (uint16_t)PSD_NORM_MAX; }

  return (uint16_t)(((raw - PSD_RAW_MIN) * PSD_NORM_MAX) / (PSD_RAW_MAX - PSD_RAW_MIN));
}
```

기준값 MIN/MAX 는 `psd_obs_min[]`, `psd_obs_max[]` 에 관측된 범위를 계속 기록해두고, 물체를 최원거리 ~ 최근접으로 왕복시킨 뒤 그 값을 상수에 옮겨 적는 방식으로 정했다.

PSD 는 물체가 가까울수록 출력 전압이 높기 때문에, 정규화 결과는 거리가 아니라 **"가까운 정도"** 다. (0 = 최원거리, 1000 = 최근접)

#### 2.2 adc1_buffer[0] 값에 이동평균 필터 적용

윈도우 8칸의 순환 버퍼로 이동평균을 구해 `adc_filtered[]` 에 넣는다.

```c
static uint32_t movavg_update(MovAvg_t *f, uint32_t sample)
{
  f->sum -= f->buf[f->idx];      /* 가장 오래된 값 제거 */
  f->buf[f->idx] = sample;
  f->sum += sample;              /* 새 값 추가 */

  f->idx = (uint8_t)((f->idx + 1u) % MA_WINDOW);

  if (f->count < MA_WINDOW) { f->count++; }

  return f->sum / f->count;
}
```

매번 윈도우 8개를 전부 더하지 않고 가장 오래된 값만 빼고 새 값을 더하는 방식이라, 윈도우 크기와 무관하게 연산량이 일정하다.

윈도우가 다 차기 전에는 `MA_WINDOW`(8)가 아니라 실제로 들어온 개수(`count`)로 나눈다. 8로 나누면 초반 8샘플 동안 값이 0쪽으로 끌려 내려가서 실재하지 않는 과도응답이 생기기 때문이다.

윈도우를 키우면 노이즈는 더 줄지만 응답이 느려진다. 8칸이면 10ms × 8 = 최대 80ms 지연이다.

#### 2.3 원본값과 필터값 비교

`adc_raw[]`(원본)과 `adc_filtered[]`(필터)를 따로 두어 Live Expressions 에서 나란히 비교했다. 추가로 최근 8샘플 구간의 변동폭(최대−최소)을 `adc_raw_pp[]`, `adc_flt_pp[]` 로 계산해서 눈으로 보는 것 말고 숫자로도 비교할 수 있게 했다.

영상 참조 :
https://drive.google.com/file/d/18_eppilrHaaV1rARziDE2GttdRndBE0C/view?usp=sharing

---

### 3단계 : 위에서 만든 필터를 adc1_buffer[0]뿐 아니라 4채널 전부(buffer[0] ~ [3])에 똑같이 적용하려면 코드를 어떻게 바꿔야 하는가?

이동평균은 이전 샘플들을 기억해야 하는 필터라서, **필터의 상태(윈도우 버퍼 / 합계 / 인덱스)가 채널마다 독립적으로 있어야 한다.**

한 채널만 처리할 때 흔히 쓰는 형태는 이렇다.

```c
uint32_t filter_ch0(uint32_t sample)
{
    static uint32_t buf[MA_WINDOW];   /* 함수 안 static = 딱 한 벌 */
    static uint32_t sum   = 0;
    static uint8_t  idx   = 0;
    static uint8_t  count = 0;

    sum -= buf[idx];
    buf[idx] = sample;
    sum += sample;
    idx = (idx + 1) % MA_WINDOW;
    if (count < MA_WINDOW) count++;

    return sum / count;
}

adc_filtered[0] = filter_ch0(adc1_buffer[0]);
```

이 상태로 채널만 바꿔가며 호출하면 4채널이 `buf`, `sum`, `idx` 한 벌을 공유하게 된다. `ch0, ch1, ch2, ch3, ch0, …` 순서로 값이 섞여 들어가서 어느 채널의 평균도 아닌 값이 나온다.

그래서 이렇게 바꿨다.

```c
typedef struct
{
  uint32_t buf[MA_WINDOW];
  uint32_t sum;
  uint8_t  idx;
  uint8_t  count;
} MovAvg_t;

static MovAvg_t g_ma[ADC_CH_NUM];   /* 채널마다 독립된 상태 한 벌씩 */

static uint32_t movavg_update(MovAvg_t *f, uint32_t sample) { ... }

/* 호출부 */
for (ch = 0u; ch < ADC_CH_NUM; ch++)
{
  adc_raw[ch]      = adc1_buffer[ch];
  adc_filtered[ch] = movavg_update(&g_ma[ch], adc_raw[ch]);
  psd_norm[ch]     = psd_normalize(adc_filtered[ch]);
}
```

바꾼 것은 세 가지다.

1. 흩어져 있던 `static` 변수를 `MovAvg_t` 구조체로 묶음
2. 채널 수만큼 배열로 만들어 상태를 분리 — `g_ma[ADC_CH_NUM]`
3. 필터 로직이 상태 포인터를 인자로 받게 함 — `movavg_update(MovAvg_t *f, ...)`

힌트에서 말한 "필터 로직을 함수로 뽑아내고, 채널별로 별도의 이전값 저장 공간이 필요하다"가 각각 3번과 2번에 해당한다. 이렇게 해두면 채널이 8개로 늘어나도 `ADC_CH_NUM` 숫자만 바꾸면 되고 필터 코드는 손댈 필요가 없다. 메모리는 채널당 38바이트, 4채널 약 152바이트라 부담도 없다.

---

### 4단계 : flag 방식 대신, buffer 인덱스가 바뀌는 것을 직접 감지하는 다른 방법을 설계하기.

**방법 1 — DMA 잔여 카운터(NDTR) 읽기**

DMA 컨트롤러의 NDTR 레지스터에는 남은 전송 개수가 실시간으로 들어 있다. 이걸 읽으면 DMA 가 지금 몇 번째를 쓰는 중인지 직접 알 수 있다.

```c
uint32_t wr_idx = ADC_CH_NUM - __HAL_DMA_GET_COUNTER(&hdma_adc1);

if (wr_idx != prev_idx)
{
    prev_idx = wr_idx;
    /* 방금 갱신이 끝난 채널만 처리 */
}
```

현재 코드의 `if (adc_updated_flag)` 자리를 그대로 대체할 수 있고, 콜백이나 인터럽트가 필요 없다. 다만 폴링이라 CPU 를 계속 쓰고, 읽는 순간에도 DMA 는 돌고 있어서 경합 자체가 사라지지는 않는다.

**방법 2 — 더블 버퍼 (ping-pong)**

buffer 를 2배로 잡고 앞뒤 절반을 번갈아 쓰게 한다. DMA 가 앞 절반을 채우면 `HAL_ADC_ConvHalfCpltCallback`, 뒤 절반까지 채우면 `HAL_ADC_ConvCpltCallback` 이 호출되므로 어느 쪽이 채워졌는지 구분할 수 있다.

```
DMA 가 [0..3] 을 쓰는 동안  →  CPU 는 [4..7] 을 읽는다
DMA 가 [4..7] 을 쓰는 동안  →  CPU 는 [0..3] 을 읽는다
```

쓰는 영역과 읽는 영역이 절대 겹치지 않기 때문에 1단계에서 지적한 경합이 구조적으로 사라진다. 처리 시간이 한 주기를 조금 넘겨도 데이터가 깨지지 않는다. 대신 메모리가 2배 필요하고 처리 시점이 반 주기 늦다.

현재는 변환 주기 10ms 에 처리 시간이 수 µs 라 flag 방식으로 충분하지만, 채널이 늘어나거나 처리 로직이 무거워져 주기를 넘길 위험이 생기면 방법 2로 가는 것이 맞다고 본다.

---

### 시행착오

처음에 `adc_raw[0]` 이 4095 에 고정되어 손을 움직여도 값이 변하지 않았다. 확인해 보니 보드 몰렉스 커넥터의 핀 순서(`GND / 5V / Analog`)가 PSD 케이블의 표준 순서(`Vo / GND / Vcc`)와 맞지 않았고, 그 상태로 커넥터가 뒤집혀 꽂혀 있어서 **센서에 역전압이 걸려 있었다.** 센서가 손상되면서 출력이 5V 근처로 떠버렸고, 그게 그대로 PA0 에 들어간 상태였다.

GPIO 는 디지털 모드에서 5V 톨러런트지만 아날로그 모드에서는 핀이 ADC 샘플링 회로에 직결되기 때문에 VDDA(3.3V)를 넘는 입력을 넣으면 안 된다는 것을 알게 되었다. 센서를 교체하고 커넥터를 올바른 순서로 다시 꽂은 뒤 정상 동작을 확인했다.

앞으로는 센서를 연결하기 전에 멀티미터로 각 선의 전압을 확인하고, 신호선에 직렬 저항을 하나 넣어 잘못 연결됐을 때 전류가 제한되도록 하는 편이 안전하겠다고 생각했다.

---

### 결론

Timer 트리거 + DMA Circular 구성으로 CPU 개입 없이 4채널 값을 10ms 주기로 최신 상태로 유지하고, 그 위에 이동평균 필터와 Min-Max 정규화를 적용했다.

핵심은 DMA 와 CPU 가 같은 메모리를 동시에 다룬다는 점이었다. flag 하나로 시점을 맞추는 것만으로 대부분 해결되지만, 그 flag 가 무엇을 보장하고 무엇을 보장하지 않는지 구분해서 생각한 것이 4단계 설계로 이어졌다.

필터 상태를 구조체로 묶어 채널별로 분리해둔 덕분에 1채널 → 4채널 확장이 상수 하나 변경으로 끝났다. 상태를 가진 모듈은 처음부터 "상태 + 상태를 받는 함수" 형태로 설계하는 편이 확장에 유리하다는 것을 확인했다.

---

**Demo Video** : https://drive.google.com/file/d/18_eppilrHaaV1rARziDE2GttdRndBE0C/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c)
