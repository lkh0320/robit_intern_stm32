## HW_3

---

이규환 2026406017

---

### 설정 요약

`파라미터 조립 → CAN1 TX (29bit 확장 ID, 1Mbps) → RS00`
`RS00 응답 → CAN1 RX FIFO0 → 모터 ID 확인`

| 항목 | 값 | 이유 |
|---|---|---|
| CAN1 Bit rate | `1Mbps` | RS00 CAN 통신 속도 |
| Prescaler / BS1 / BS2 | `5` / `6TQ` / `2TQ` | 45MHz / 5 / (1+6+2) = 1Mbps |
| ID 형식 | 29bit 확장 ID | RS00 사설 프로토콜이 확장 ID 사용 |
| Host ID | `0xFD` | 명령을 보낸 쪽(STM32)을 구분하는 ID |
| Motor ID | `14` (고정) | 과제 설정값 |
| 제어 모드 | 속도 모드 (run_mode = 2) | CW/CCW 를 속도 부호로 바로 표현 가능 |
| Vin | 24V | 과제 조건 |

**핀 배치**

| 기능 | 핀 |
|---|---|
| CAN1_RX | `PA11` |
| CAN1_TX | `PA12` |

---

### 1. 과제 요구사항

Robstride RS00 액추에이터 CW/CCW 제어 (Vin 24V, 속도 파라미터는 데이터시트 값의 절반 미만)

---

### 2. 구현

#### 2.1 29bit ID 조립

```
bit28~24     bit23~8            bit7~0
통신 타입    데이터 영역 2       모터 ID
             (bit15~8: Host ID)
```

```c
header.ExtId = ((uint32_t)comm_type << 24) | ((uint32_t)RS_HOST_ID << 8) | motor_id;
```

RS00 은 명령 종류를 데이터가 아니라 **ID 안의 통신 타입**으로 구분한다. 그래서 전송 함수 하나에 타입만 바꿔 넘기도록 묶었다.

| 통신 타입 | 의미 |
|---|---|
| 0 | 장치 ID 요청 |
| 3 | 모터 활성화 |
| 4 | 모터 정지 |
| 18 (0x12) | 파라미터 쓰기 |

전송 전에 메일박스가 빌 때까지 기다리되 10ms 가 넘으면 포기하게 했다. 모터가 없거나 버스가 끊겨도 프로그램 전체가 멈추지 않게 하기 위해서다.

#### 2.2 파라미터 쓰기

```
Byte0~1: index   Byte2~3: 0   Byte4~7: 값 (low byte 먼저)
```

```c
memcpy(&data[0], &index, 2);  // Byte0~1: 파라미터 index
memcpy(&data[4], &value, 4);  // Byte4~7: float 값
```

매뉴얼이 little endian 을 요구하는데 STM32 도 little endian 이라, float 를 바이트로 쪼개지 않고 `memcpy` 로 그대로 넣으면 된다.

#### 2.3 속도 모드 초기화 순서 (매뉴얼 4.3.3)

```c
RS_Send(RS_TYPE_STOP, zero);                      // 이전 상태 초기화
RS_WriteU8(RS_IDX_RUN_MODE, 2);                   // 속도 모드
RS_Send(RS_TYPE_ENABLE, zero);                    // 모터 활성화
RS_WriteFloat(RS_IDX_LIMIT_CUR, RS_LIMIT_CUR_A);  // 전류 제한
RS_WriteFloat(RS_IDX_ACC_RAD, RS_ACC_RAD_S2);     // 가속도
```

매뉴얼 순서(run_mode → 활성화 → limit_cur → acc_rad → spd_ref, 매뉴얼 p.52) 앞에 **정지만 추가했다.** 디버깅하면서 보드만 리셋되면 모터는 이전 모드로 켜져 있을 수 있어서, 항상 같은 상태에서 시작하게 하기 위해서다.

#### 2.4 모터 ID 확인

```c
motor_id = (n == 0) ? RS_MOTOR_ID_DEFAULT : n;
RS_Send(RS_TYPE_ID_REQUEST, zero);
...
if (((rx_header.ExtId >> 24) & 0x1F) == 0)
{
  motor_id = (rx_header.ExtId >> 8) & 0xFF;  // 응답한 모터 ID 저장
  return;
}
```

초기화 전에 통신 타입 0(ID 요청)을 ID 14 로 먼저 보내고, 응답이 없으면 1 ~ 126 을 차례로 보내 응답한 모터의 ID 를 `motor_id` 로 쓴다. 모터 ID 설정이 과제 조건(14)과 다를 때도 동작하게 하려는 목적이었다. 한계는 4장에 정리했다.

#### 2.5 CW / CCW 전환

```c
uint32_t phase = ((HAL_GetTick() - t_start) / RS_PHASE_MS) % 2;
RS_WriteFloat(RS_IDX_SPD_REF, (phase == 0) ? RS_SPEED_RAD_S : -RS_SPEED_RAD_S);
```

속도 모드에서는 **목표 속도의 부호가 곧 방향**이다. 5초마다 +5 / -5 rad/s 를 번갈아 보낸다. 가속도 제한(20 rad/s²)이 걸려 있어서 방향이 바뀔 때 0.5초에 걸쳐 부드럽게 반전된다.

#### 2.6 파라미터 선정 (RS00 User Manual)

| 파라미터 | 사용값 | 매뉴얼 | 판단 |
|---|---|---|---|
| 전압 | 24V | 정격 48V, 동작 24 ~ 60V (p.8) | 동작 범위 하한, 저전압 보호 12V (p.28) 보다 높음 |
| 속도 spd_ref | 5 rad/s | -33 ~ 33 rad/s (p.45), 무부하 315rpm @48V (p.8) | 절반(16.5) 미만 |
| 전류 limit_cur | 4A | 0 ~ 16A (p.46), 정격 4.7Apk (p.9) | 절반(8A) 미만 |
| 가속도 acc_rad | 20 rad/s² | 기본값 20 rad/s² (p.46) | 기본값 사용 |

24V 에서는 역기전력 때문에 최고 속도가 전압에 비례해 약 16.5 rad/s 로 줄어든다고 추정할 수 있다(매뉴얼 값이 아닌 계산값). 이 기준으로 봐도 절반(약 8.25)보다 5 rad/s 가 작다.

출처: RS00 User Manual (`RS00User_Manual260713.pdf`), 쪽수는 PDF 기준

---

### 3. 검증

데모 영상에서 RS00 이 5초 주기로 CW / CCW 를 반복하는 것으로 확인했다. 모터가 명령대로 방향을 바꾸는 것 자체가 배선·보드레이트·ID·파라미터 쓰기가 모두 맞다는 근거다.

---

### 4. 시행착오

강의 템플릿 ioc 에 CAN 을 추가했더니 빌드에서 `stm32f4xx_hal_can.h: No such file or directory` 가 49개 나왔다. `stm32f4xx_hal_conf.h` 에는 CAN 모듈이 켜졌는데, **Drivers 폴더에 CAN 드라이버 파일이 복사되지 않은 상태**였다. 프로젝트와 같은 HAL 버전(v1.8.5)의 `stm32f4xx_hal_can.h/.c` 를 넣어 해결했다.

설정(conf)과 실제 파일이 따로 관리된다는 것, 그래서 기존 ioc 에 주변장치를 추가할 때는 새로 켠 주변장치의 드라이버가 들어왔는지 확인해야 한다는 것을 알게 됐다.

**코드 리뷰에서 나온 한계**

- `motor_id` 는 초기값이 14 이고 스캔이 실패해도 14 로 돌아가기 때문에, `motor_id` 값만으로는 모터가 실제로 응답했는지 알 수 없다. 응답을 받았을 때만 1 이 되는 플래그(`rs_found` 등)를 따로 두는 것이 맞다.
- ID 스캔은 응답한 아무 모터의 ID 로 `motor_id` 를 바꾼다. 과제처럼 모터가 하나뿐일 때는 문제없지만, 여러 모터가 연결된 공용 CAN 버스에서는 다른 모터에 명령이 갈 수 있다. 과제 조건이 ID 14 고정이므로 스캔 없이 ID 14 응답만 확인하는 방식이 더 안전하다.

---

### 5. 결론

CAN 으로 RS00 을 속도 모드로 돌려 5초마다 CW / CCW 를 전환했다. UART 와 달리 CAN 은 **ID 자체에 명령 종류와 목적지가 들어가는** 구조라, 패킷을 만드는 일의 절반이 ID 비트를 맞추는 일이었다.

---

**Demo Video** : https://drive.google.com/file/d/1kJI7ICWSahvgfnAXo30dgdxt5QuJi0Bw/view?usp=sharing

소스 : [`Core/Src/main.c`](Core/Src/main.c)
