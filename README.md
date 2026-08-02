# AX_MASTER

STM32F411RE 두 보드로 구성한 ARMIGO 로봇팔의 최종 마스터 펌웨어입니다.
마스터 AX-12A 4축의 위치를 읽고, 키패드와 LCD로 Home, Admin JOG,
Teaching, Auto 시퀀스를 운용하며 Fusion UART 프로토콜로 AX_SLAVE를
제어합니다.

- 운영 브랜치: `main`
- 최종 코드 기준: `ryu@736745c`
- 이전 main 보관: `test`
- 대응 슬레이브: https://github.com/Armigos/AX_SLAVE
- 프로젝트 문서: https://app.notion.com/p/3a6e8d3fa943806caac4f5ef209e2772

## 핵심 기능

- AX-12A Master ID `10, 11, 12, 14` 비동기 위치 읽기
- FreeRTOS 기반 LCD, 키패드, AX-12, Bluetooth, TelePlot 태스크
- BTN16 Home 완료 전 동작을 막는 부팅 안전 인터록
- BTN15 Admin JOG, BTN13 Teaching, BTN14 Auto
- Preset 10개, Preset당 최대 30 Step의 Flash 저장
- 첫 Auto Step의 Sharp 센서 감지 대기와 이후 연속 시퀀스
- PB2 EXTI 기반 E-STOP 및 실제 슬레이브 자세 고정
- USART6 인터럽트 송수신과 최신 JOG 프레임 우선 처리

## 시스템 구조

```text
AX-12A Master x4 (10, 11, 12, 14)
             |
             | USART1 Half-Duplex, 1 Mbps
             v
AX_MASTER / NUCLEO-F411RE
             |
             | USART6, 115200
             v
AX_SLAVE / NUCLEO-F411RE
             |
             | USART1 Half-Duplex, 1 Mbps
             v
AX-12A Slave x4 (1, 2, 3, 5)

ST-LINK VCP <-> USART2, 115200 <-> TelePlot
```

## UART 및 배선

| 인터페이스 | 핀 | 속도 | 용도 |
|---|---|---:|---|
| USART1 | PA9 / D8 | 1,000,000 | AX-12A Half-Duplex DATA |
| USART2 | PA2 TX, PA3 RX | 115,200 | ST-LINK 콘솔 및 TelePlot |
| USART6 | PA11 TX, PA12 RX | 115,200 | AX_SLAVE 또는 HC-05 |
| E-STOP | PB2 / EXTI2 | - | 비상정지 입력 |

유선 보드 간 시험:

| AX_MASTER | AX_SLAVE |
|---|---|
| PA11 / USART6_TX | PC7 / USART6_RX |
| PA12 / USART6_RX | PC6 / USART6_TX |
| GND | GND |

AX-12A는 외부 9~12V로 구동하고 외부 전원 GND와 Nucleo GND를 반드시
공통 연결합니다. AX-12A를 Nucleo 5V 핀으로 구동하지 마십시오.

## 모터 매핑

| Master | Slave |
|---:|---:|
| ID 10 | ID 1 |
| ID 11 | ID 2 |
| ID 12 | ID 3 |
| ID 14 | ID 5 |

마스터 위치 제한과 Home/E-STOP 정렬 속도는
`Core/Inc/ax12_config.h`에서 설정합니다.

## 키패드 운용

| 입력 | 기능 |
|---|---|
| BTN16 | 네 축 Home 512 실행 및 부팅 인터록 해제 |
| BTN15 | Admin JOG 진입, JOG 중 재입력 시 Dashboard |
| BTN13 | Teaching 모드 진입 |
| BTN14 | Auto 모드 진입, Auto 준비 상태에서 재입력 시 실행 |
| BTN1~10 | Teaching/Auto Preset 선택 |
| BTN12 | Teaching 현재 Step 저장 |
| BTN11 | 선택 Preset 전체 삭제 |

1. 부팅 후 BTN16 Home을 먼저 완료합니다.
2. BTN15 JOG로 4축 추종과 방향을 확인합니다.
3. BTN13, Preset 선택, BTN12 순서로 동작 Step을 저장합니다.
4. BTN14, Preset 선택, BTN14 순서로 Auto 시퀀스를 시작합니다.
5. 첫 Auto Step은 AX_SLAVE Sharp 센서 감지 후 움직이며 이후 Step은
   센서를 다시 기다리지 않습니다.

## Fusion UART 프로토콜

```text
AA 55 CMD LEN PAYLOAD... CHECKSUM
```

Checksum은 `CMD + LEN + Payload 전체`의 하위 8비트입니다.

| CMD | 이름 | 역할 |
|---:|---|---|
| `0x01` | SET_GOAL_POS | 개별 축 목표 |
| `0x02` | SET_TORQUE | 슬레이브 Torque 제어 |
| `0x03` | REQ_STATUS | 슬레이브 상태 요청 |
| `0x04` | HOME_POS | 4축 Home |
| `0x05` | SET_ALL_POS | JOG/Teaching 4축 위치 |
| `0x06` | START_AUTO | 첫 Auto Step과 Sharp 대기 시작 |
| `0x07` | RUN_AUTO | 이후 Auto Step 실행 |
| `0x08` | HOLD_CURRENT | 슬레이브 실제 자세 고정 |
| `0x83` | STATUS_REPLY | Position 8B + Load 8B + Auto flags 1B |

JOG 위치는 기본 10ms 주기로 전송합니다. 전송 중 새 JOG 값이 생기면
오래된 대기 프레임 대신 최신 프레임을 유지합니다. Home, Auto,
HOLD_CURRENT, Torque 같은 제어 명령은 JOG보다 우선합니다.

## E-STOP

1. PB2 E-STOP 입력이 EXTI2 인터럽트를 발생시킵니다.
2. 진행 중인 Home, JOG, Auto를 중단합니다.
3. AX_SLAVE에 `HOLD_CURRENT(0x08)`를 보내 실제 현재 자세를 Goal로
   고정합니다.
4. `REQ_STATUS`와 17바이트 `STATUS_REPLY`로 고정 자세를 확인합니다.
5. 마스터 컨트롤러를 해당 자세에 정렬한 뒤 BTN15로 안전하게 JOG를
   재개합니다.

이 과정은 과거 Auto 목표가 E-STOP 해제 후 다시 실행되는 것을 막습니다.

## TelePlot

```text
>master_1_pos:512
>master_2_pos:512
>master_3_pos:512
>master_4_pos:512
```

USART2 ST-LINK Virtual COM Port를 115200, 8N1로 연결합니다.

## 주요 파일

| 파일 | 역할 |
|---|---|
| `AX_MASTER.ioc` | CubeMX 핀, UART, FreeRTOS, NVIC |
| `Core/Inc/ax12_config.h` | ID, 범위, Home/E-STOP 설정 |
| `Core/Src/ax12.c` | AX-12 Protocol 1.0 비동기 드라이버 |
| `Core/Src/freertos.c` | 모드, 키패드, 통신, LCD, E-STOP |
| `Core/Src/teaching_storage.c` | Teaching Flash 저장 |

## 빌드

```bash
cmake --preset Debug
cmake --build --preset Debug
```

결과 파일은 `build/Debug/AX_MASTER.elf`입니다. STM32CubeIDE에서는
`AX_MASTER.ioc`를 열고 Build 후 ST-LINK로 다운로드합니다.

## 최종 시험 순서

1. 양쪽 AX-12 ID와 USART1 1Mbps를 확인합니다.
2. UART6 유선 교차 연결과 공통 GND를 확인합니다.
3. BTN16 Home이 네 축 모두 512에 도착하는지 확인합니다.
4. BTN15 JOG에서 매핑 `10→1, 11→2, 12→3, 14→5`를 확인합니다.
5. Teaching 저장 후 전원을 다시 켜도 Preset이 유지되는지 확인합니다.
6. Auto 첫 Step이 Sharp 감지 전 대기하고 감지 후 실행되는지 확인합니다.
7. Auto 중 E-STOP을 눌러 슬레이브가 현재 자세를 유지하는지 확인합니다.
8. 유선 시험 완료 후 HC-05 무선 링크로 동일 항목을 재시험합니다.
