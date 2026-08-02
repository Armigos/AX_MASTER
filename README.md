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
5. Home 완료 후 LCD2는 `AUTO SENSOR MODE / PRESET 01`로 전환됩니다.
6. 물체가 없으면 `WAITING`, 30cm 이내에서 감지되면 `DETECTED`와
   `5, 4, 3, 2, 1` 카운트다운을 표시합니다.
7. 5초 동안 연속 감지되면 `START`를 표시한 뒤 Preset 1 Auto를
   자동 실행합니다. 감지가 끊기거나 상태 회신이 600ms 이상 없으면
   카운트다운을 초기화합니다.
8. 이후 Step은 Sharp 센서를 다시 기다리지 않습니다.

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

### VS Code 빌드·업로드 경로 주의사항

`.vscode/settings.json`과 `.vscode/tasks.json`에는 STM32CubeIDE에 포함된
Ninja, ARM GCC, STM32 Programmer의 절대경로가 들어 있습니다. 이 경로는
현재 컴퓨터의 설치 위치와 CubeIDE 버전에 맞춘 값이므로 다른 컴퓨터에서
사용할 때는 해당 PC에 설치된 실제 경로로 반드시 변경해야 합니다.

- `cmake.environment.PATH`: `arm-none-eabi-gcc`가 있는 `tools/bin` 폴더
- `CMAKE_MAKE_PROGRAM`: `ninja.exe`의 전체 경로
- `Flash Only.command`: `STM32_Programmer_CLI.exe`의 전체 경로
- `Flash Only`의 ELF 경로: `build/Debug/AX_MASTER.elf`

경로를 변경한 뒤 VS Code에서 `CMake: Delete Cache and Reconfigure`를
실행하고 다시 빌드합니다. `Ctrl+Shift+F`는 기존 ELF를 업로드하는
`Flash Only` 작업이므로, 코드 변경 후에는 먼저 빌드해야 합니다.

### Sharp 감지는 정상인데 LCD가 `WAITING`에 멈추는 경우

GP2Y0A21YK0F의 거리값과 SLAVE `sharp_detected`가 정상이어도 LCD2가
`WAITING`에서 바뀌지 않는 현상이 있었습니다. 다음 TelePlot 값을 양쪽
보드에서 확인해 센서, 통신, Auto 조건을 순서대로 분리 진단했습니다.

```text
SLAVE: sharp_cm, sharp_detected
MASTER: master_sharp_detected, slave_status_flags, slave_status_age_ms
MASTER: auto_home_ready, auto_system_mode, auto_run_state
MASTER: auto_preset1_mask, auto_countdown, auto_start_pending
```

센서와 통신은 정상이었고 `auto_countdown`도 내부적으로 진행됐지만,
기존 글꼴 함수가 문자 픽셀마다 ILI9341 주소 창을 다시 설정하고 SPI를
전송해 LCD 화면 한 번을 그리는 데 수 초가 걸리는 것이 근본 원인이었습니다.
카운트다운은 끝났지만 화면에는 먼저 그린 `WAITING`이 남아 있었습니다.

해결을 위해 `Core/Src/lcd_font.c`에서 한 글자를 하나의 주소 창으로 잡고
RGB565 행 데이터를 연속 SPI 블록으로 전송하도록 변경했습니다. 또한
LCD2의 공유 SPI mutex 대기시간을 늘리고 Auto 센서 화면을 주기적으로
갱신합니다. Home 시작·완료 시 Run 상태도 `STOPPED`로 초기화하여 항상
Preset 1 센서 대기 조건에 진입하도록 했습니다.

정상 동작 순서는 다음과 같습니다.

```text
WAITING → DETECTED → 5 → 4 → 3 → 2 → 1 → START → Preset 1 Auto
```

물체가 없을 때는 `sharp_cm > 30`, `sharp_detected = 0`이어야 합니다.
센서 앞에 바닥판이나 로봇 부품이 있으면 배경을 물체로 감지할 수 있으므로
렌즈를 바닥보다 높이고 정면 30cm 범위를 비운 상태에서 시험합니다.

## 최종 시험 순서

1. 양쪽 AX-12 ID와 USART1 1Mbps를 확인합니다.
2. UART6 유선 교차 연결과 공통 GND를 확인합니다.
3. BTN16 Home이 네 축 모두 512에 도착하는지 확인합니다.
4. BTN15 JOG에서 매핑 `10→1, 11→2, 12→3, 14→5`를 확인합니다.
5. Teaching 저장 후 전원을 다시 켜도 Preset이 유지되는지 확인합니다.
6. Auto 첫 Step이 Sharp 감지 전 대기하고 감지 후 실행되는지 확인합니다.
7. Auto 중 E-STOP을 눌러 슬레이브가 현재 자세를 유지하는지 확인합니다.
8. 유선 시험 완료 후 HC-05 무선 링크로 동일 항목을 재시험합니다.
