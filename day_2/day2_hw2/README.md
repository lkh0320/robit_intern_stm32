## HW_2

---

이규환 2026406017

---

### 설정 요약

`스위치 입력 → Instruction Packet 조립 → USART3(LL) TX → MX106`
`MX106 Status Packet → USART3 RX → DMA1 Stream1(Circular) → IDLE 인터럽트 → 파싱`

| 항목 | 값 | 이유 |
|---|---|---|
| USART3 Mode | Asynchronous | 다이나믹셀은 UART 통신 |
| Baud Rate | `1000000` | 모터 초기 설정값 |
| Driver | HAL 대신 **LL** | 1Mbps 에서 바이트마다 HAL 함수 호출 오버헤드를 피하기 위해 |
| DMA1 Stream1 (RX) | Circular / Byte | 수신 길이가 가변이라 미리 개수를 지정할 수 없음 |
| USART3 IDLE 인터럽트 | Enable | "패킷 하나가 끝났다"는 시점을 알려주는 신호 |
| Protocol | **2.0** | 강의자료의 패킷 구조(`FF FF FD 00`)가 2.0 |
| DXL ID | 14 | 모터 초기 설정값 |

**핀 배치**

| 기능 | 핀 |
|---|---|
| USART3_TX → DXL | `PC10` |
| USART3_RX ← DXL | `PC5` |
| 스위치 | `PB12`, `PB13`, `PB14` |
| 응답 상태 표시 LED | `PB0`(정상), `PB1`(에러/무응답) |

스위치는 day_1 과 동일하게 active-low 다.

---

### 1. 과제 요구사항

DYNAMIXEL(MX106) 제어

| 스위치 | 목표 각도 |
|---|---|
| `PB12` | 90° |
| `PB13` | 180° |
| `PB14` | 0° |

---

### 2. 구현

#### 2.1 왜 HAL 이 아니라 LL 인가

강의자료가 `LL + DMA + Circular queue + IDLE interrupt` 구성을 제시한 이유를 정리하면 이렇다.

HAL 의 `HAL_UART_Receive_IT()` 는 **받을 바이트 수를 미리 알려줘야** 인터럽트가 뜬다. 그런데 다이나믹셀의 Status Packet 은 명령 종류에 따라 길이가 다르다. Write 응답은 11바이트, Read 응답은 읽은 데이터만큼 더 길다. 길이를 모르니 1바이트씩 받게 되고, 1Mbps 에서는 **8µs 마다 인터럽트가 한 번씩** 뜬다. CPU 가 인터럽트 진입/복귀만 하다 끝난다.

DMA + IDLE 방식은 바이트 수집을 하드웨어에 맡긴다. 인터럽트는 **패킷이 끝났을 때 한 번만** 뜬다. 패킷 길이를 몰라도 되는 것이 핵심이다.

```
HAL   : 바이트마다 인터럽트  →  11바이트 = 인터럽트 11회
DMA   : DMA 가 알아서 수집   →  11바이트 = 인터럽트 1회 (IDLE)
```

#### 2.2 Instruction Packet 조립

Protocol 2.0 패킷 구조는 고정이고 바뀌는 건 ID / 주소 / 데이터뿐이라, Write 명령 하나를 만드는 함수로 묶었다.

```
FF FF FD 00  ID  LEN_L LEN_H  0x03  ADDR_L ADDR_H  [DATA...]  CRC_L CRC_H
```

Length 필드는 **그 필드 뒤에 오는 바이트 수**다. Instruction 1 + 주소 2 + 데이터 n + CRC 2 이므로 `n + 5` 가 된다.

```c
uint16_t len = (uint16_t)(n + 5u); // 주소 2 + 데이터 n + Instruction 1 + CRC 2
```

이 부분을 처음에 `n + 3` 으로 착각하기 쉬운데, 로보티스 문서의 "파라미터 수 + 3" 에서 말하는 파라미터에 **주소 2바이트가 이미 포함**되어 있기 때문이다.

#### 2.3 CRC

Protocol 2.0 은 CRC-16(다항식 `0x8005`)을 쓴다. 로보티스가 제공하는 예제 코드는 256칸짜리 룩업 테이블을 쓰지만, 같은 결과를 내는 비트 단위 연산으로 구현해서 테이블을 코드에 넣지 않았다.

```c
static uint16_t dxlCrc(const uint8_t *data, uint16_t size) // Protocol 2.0 CRC-16, poly 0x8005
{
  uint16_t crc = 0u;
  ...
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (b = 0u; b < 8u; b++)
    {
      crc = (crc & 0x8000u) ? (uint16_t)((uint16_t)(crc << 1) ^ 0x8005u)
                            : (uint16_t)(crc << 1);
    }
  ...
}
```

512바이트를 아끼는 대신 바이트당 8번 루프를 돈다. 패킷이 16바이트 이하라 연산량은 무시할 수 있는 수준이다.

#### 2.4 각도 → Goal Position 값

MX106 은 0~4095 한 바퀴(360°)다.

```
4096 / 360 = 11.38 counts/deg
```

| 각도 | 값 |
|---|---|
| 0° | 0 |
| 90° | 1024 |
| 180° | 2048 |

```c
static const uint32_t SW_GOAL[SW_NUM]  = { 1024u, 2048u, 0u }; // 90도, 180도, 0도
```

#### 2.5 Torque Enable

Goal Position 만 써서는 모터가 움직이지 않는다. Torque Enable(주소 64)이 0이면 모터가 힘을 내지 않기 때문이다. 초기화 직후 한 번 켜주었다.

```c
HAL_Delay(100);
dxlTorque(DXL_ID, 1); // 토크를 켜야 Goal Position 이 먹는다
```

앞의 `HAL_Delay(100)` 은 모터 부팅을 기다리는 시간이다. 전원이 동시에 들어가는 구성이라 MCU 가 먼저 깨어나 명령을 보내면 모터가 받지 못한다.

#### 2.6 누른 순간에만 1회 전송

메인 루프는 10ms 마다 돌기 때문에 스위치를 누르고 있으면 초당 100번 같은 명령이 나간다. 모터는 매번 응답을 돌려주므로 버스가 불필요하게 점유된다.

```c
if (sw_prev[i] == 0u) // 누른 순간에만 1회 전송
{
  sw_prev[i] = 1u;
  dxlGoalPosition(DXL_ID, SW_GOAL[i]);
}
```

직전 상태를 기억해 **눌리지 않음 → 눌림** 으로 바뀌는 순간만 잡았다. day_1 에서 스위치를 매 루프 읽던 것과 달리, 여기서는 "상태"가 아니라 "사건"이 필요했다.

#### 2.7 수신 — IDLE 인터럽트가 알려주는 것

DMA 가 `PCrxBuffer` 를 계속 채우다가 라인이 쉬면 IDLE 인터럽트가 뜬다. 이때 NDTR 레지스터에서 남은 개수를 빼면 이번에 몇 바이트 들어왔는지 나온다.

```c
int len = RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);

if (len > 0 && rxFlag == 0u){
    memcpy(rxPacket, PCrxBuffer, (size_t)len); // 아래에서 DMA 를 되감으므로 먼저 꺼내둔다
    rxLen  = (uint16_t)len;
    rxFlag = 1u;
}

LL_USART_ClearFlag_IDLE(USART3);
LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, RX_BUFFER_SIZE);
LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
```

**복사 위치가 중요하다.** 아래 세 줄이 DMA 쓰기 포인터를 버퍼 맨 앞으로 되감기 때문에, 그 뒤로는 다음 패킷이 `PCrxBuffer[0]` 부터 덮어쓴다. 되감기 전에 꺼내지 않으면 데이터가 사라진다.

인터럽트 안에서는 복사와 플래그 설정만 하고, 파싱은 메인 루프에서 한다. ISR 을 짧게 유지해야 다음 수신을 놓치지 않는다.

#### 2.8 Status Packet 파싱 — 에코를 걸러내야 한다

Status Packet 은 Instruction 자리에 `0x55` 가 들어간다.

```
FF FF FD 00  ID  LEN_L LEN_H  0x55  ERR  [param...]  CRC_L CRC_H
                                    └ 0 이면 정상
```

여기서 문제가 하나 있다. 다이나믹셀 버스는 **반이중(half-duplex)** 이라, 보드 구성에 따라 내가 보낸 바이트가 그대로 RX 로 되돌아온다. 그러면 버퍼가 이렇게 채워진다.

```
[ 내가 보낸 Instruction Packet ][ 모터의 Status Packet ]
  FF FF FD 00 0E 09 00 03 ...    FF FF FD 00 0E 04 00 55 ...
```

버퍼 맨 앞만 보고 판단하면 내 명령을 응답으로 착각하게 된다. 그래서 파서가 버퍼를 **앞에서부터 스캔**하면서 헤더 + ID + `0x55` 가 모두 맞는 지점을 찾도록 했다.

```c
for (i = 0u; (uint32_t)i + 11u <= (uint32_t)n; i++) // 앞에 송신 에코가 섞일 수 있어 스캔한다
```

찾은 뒤에는 CRC 를 다시 계산해서 대조한다. 헤더 네 바이트는 데이터 중간에도 우연히 나올 수 있어서, CRC 검사가 있어야 "진짜 패킷"이라고 말할 수 있다.

같은 이유로 `writepacket()` 끝에 TC(Transmission Complete) 대기를 넣었다. TXE 만 보고 함수를 빠져나오면 마지막 바이트가 아직 시프트 레지스터에 남아 있는 상태라, 반이중 버스에서 송신이 끝나기 전에 방향이 바뀔 수 있다.

```c
while(!LL_USART_IsActiveFlag_TC(USART3)); // 반이중이라 마지막 바이트까지 내보내고 버스를 놓는다
```

---

### 3. 검증

모터가 안 움직일 때 원인이 **배선인지 / ID·보드레이트인지 / 프로토콜인지** 구분할 수단이 필요했다. 응답을 파싱해서 결과를 남기도록 했다.

| 변수 | 의미 |
|---|---|
| `dxlOkCount` | 유효한 Status Packet 수신 횟수 |
| `dxlNgCount` | 뭔가 받았지만 파싱 실패한 횟수 |
| `dxlError` | 모터가 보고한 에러 코드 |
| `rxLen` | 마지막으로 받은 바이트 수 |

`PB0` / `PB1` LED 로도 같은 내용을 표시해서 디버거 없이도 볼 수 있게 했다.

이 세 값의 조합으로 원인이 갈린다.

| 상태 | 해석 |
|---|---|
| `rxLen` = 0, 양쪽 카운트 0 | 응답 자체가 없음 → RX 배선 / 모터 전원 |
| `dxlNgCount` 만 증가, `rxLen` ≈ 13 | 내 송신 에코만 돌아옴 → 모터가 응답 안 함 (ID / 보드레이트) |
| `dxlOkCount` 증가, `dxlError` = `0x02` | 통신은 되지만 주소 체계가 다름 → Protocol 1.0 펌웨어 |
| `dxlOkCount` 증가, `dxlError` = 0 | 정상 |

토크를 켜는 명령이 `main()` 시작 직후 한 번 나가기 때문에, **스위치를 누르기 전에 이미 통신 성립 여부가 판정된다.** 전원을 넣고 `PB0` 이 켜지면 그 시점에 배선·ID·보드레이트·프로토콜이 전부 맞다는 뜻이다.

실제로 나가는 패킷은 다음과 같다.

```
Torque ON     FF FF FD 00 0E 06 00 03 40 00 01 2B 69
PB12 (90°)    FF FF FD 00 0E 09 00 03 74 00 00 04 00 00 71 29
PB13 (180°)   FF FF FD 00 0E 09 00 03 74 00 00 08 00 00 81 29
PB14 (0°)     FF FF FD 00 0E 09 00 03 74 00 00 00 00 00 22 A9
```

CRC 구현은 로보티스 문서의 예제 패킷(Ping ID1 → `0x4E19`)과 대조해서 일치를 확인했다.

---

### 4. 시행착오

IDLE 처리 코드를 `USART3_IRQHandler` 가 아니라 **`ADC_IRQHandler` 에 넣어두고** 한참 헤맸다. 컴파일도 되고 경고도 없지만, USART3 의 IDLE 인터럽트는 `USART3_IRQHandler` 로만 들어오기 때문에 그 코드는 한 번도 실행되지 않는다.

인터럽트 핸들러는 이름으로 벡터 테이블과 연결되는 구조라, **어느 함수에 넣느냐가 곧 어느 인터럽트에 반응하느냐**다. 일반 함수처럼 "코드가 있으면 언젠가 실행되겠지" 하고 생각하면 안 된다는 것을 확인했다.

증상이 "아무 일도 일어나지 않음" 이라 원인을 찾기 어려웠는데, `rxLen` 같은 관측 변수를 먼저 만들어두는 것이 결국 시간을 아끼는 길이었다.

---

### 5. 결론

스위치 3개로 MX106 을 0° / 90° / 180° 로 제어했다.

이번 과제의 핵심은 **길이를 모르는 데이터를 어떻게 받느냐**였다. 보내는 쪽은 내가 길이를 아니까 단순하지만, 받는 쪽은 상대가 몇 바이트를 보낼지 모른다. HAL 의 "N바이트 받으면 알려줘" 방식이 여기서 막히고, 그래서 DMA 가 모아두고 IDLE 이 끝을 알려주는 구조가 필요해진다. 수신 길이를 `RX_BUFFER_SIZE - NDTR` 로 역산한다는 발상이 특히 그랬다.

또 하나는 **응답을 읽는 것이 곧 디버깅 수단**이라는 점이다. 처음에는 요구사항에 없으니 TX 만 구현하려 했는데, 그 상태에서 모터가 안 움직이면 확인할 방법이 전원 뽑았다 꽂는 것밖에 없다. Status Packet 을 파싱해두니 에러 코드 한 바이트로 원인이 좁혀졌다.

---

**Demo Video** : https://drive.google.com/file/d/1hKc7Wf1W0CWOzDJDc9r-tTLi2lORL1Rv/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c), [`Core/Src/stm32f4xx_it.c`](Core/Src/stm32f4xx_it.c)
