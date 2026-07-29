# AX_MASTER

STM32F411RE 기반 로봇팔 마스터 펌웨어입니다. AX-12A 마스터 모터
4개의 현재 위치를 읽고, Fusion UART 프로토콜로 AX_SLAVE에 전달합니다.
LCD, 키패드, Sharp 센서, TelePlot을 FreeRTOS 태스크로 함께 처리합니다.

- GitHub: https://github.com/Armigos/AX_MASTER
- 최적화 브랜치: `feature/uart-interrupt`
- 대응 슬레이브: https://github.com/Armigos/AX_SLAVE

## 시스템 구성

```text
AX-12A Master x4 (ID 10, 11, 12, 14)
             |
             | USART1 Half-Duplex, 1 Mbps
             v
NUCLEO-F411RE / AX_MASTER
             |
             | USART6, 115200
             v
HC-05 Master ))) Bluetooth ((( HC-05 Slave
                                      |
                                      v
                              AX_SLAVE + AX-12A x4

ST-LINK VCP <-- USART2, 115200 --> TelePlot / console
```

## UART 및 핀 설정

| 인터페이스 | STM32 핀 | 속도 | 용도 |
|---|---|---:|---|
| USART1 | PA9 / D8 | 1,000,000 | AX-12A Half-Duplex DATA |
| USART2 | PA2 TX, PA3 RX | 115,200 | ST-LINK 콘솔 및 TelePlot |
| USART6 | PA11 TX, PA12 RX | 115,200 | HC-05 또는 AX_SLAVE 링크 |

UART 형식은 8 data bits, no parity, 1 stop bit입니다.

> `AX_MASTER`의 USART6 핀은 PA11/PA12입니다. AX_SLAVE의
> PC6/PC7과 같지 않으므로 유선 시험 시 핀을 혼동하지 마십시오.

## AX-12 배선

| AX-12A 핀 | 기능 | 연결 |
|---:|---|---|
| 1 | GND | 외부 전원 GND 및 Nucleo GND |
| 2 | VDD | 외부 9~12V |
| 3 | DATA | PA9 / Arduino D8 |

AX-12A를 Nucleo 5V로 구동하지 마십시오. 모터 전원과 Nucleo의 GND는
반드시 공통으로 연결하고, 배선 변경은 전원을 끈 상태에서 진행합니다.

## 마스터 모터

| 논리 모터 | AX-12 ID | 슬레이브 ID |
|---|---:|---:|
| Master 1 | 10 | 1 |
| Master 2 | 11 | 2 |
| Master 3 | 12 | 3 |
| Master 4 | 14 | 5 |

마스터 ID는 `Core/Inc/ax12.h`에서 관리합니다.

## 실시간 동작

1. 부팅 후 ID 10, 11, 12, 14를 Ping합니다.
2. 손으로 움직일 수 있도록 마스터 모터 Torque를 OFF합니다.
3. FreeRTOS의 AX-12 태스크가 네 모터 위치를 순환하며 읽습니다.
4. USART1 송수신은 인터럽트 기반 상태 머신으로 처리합니다.
5. 키패드 버튼 15를 누르면 관리자 JOG 모드와 슬레이브 Torque가 켜집니다.
6. JOG 모드에서는 네 위치를 기본 10 ms 주기로 USART6에 전송합니다.
7. USART6 송신 중 새 값이 오면 대기열을 늘리지 않고 최신 프레임 하나만
   유지해 누적 지연을 방지합니다.
8. USART2 TelePlot은 약 33 ms 주기로 마스터 현재 위치를 출력합니다.

## Fusion UART 프로토콜

모든 패킷은 가변 길이입니다.

```text
AA 55 CMD LEN PAYLOAD... CHECKSUM
```

| 필드 | 크기 | 설명 |
|---|---:|---|
| Header | 2 | `0xAA 0x55` |
| CMD | 1 | 명령 코드 |
| LEN | 1 | Payload 길이, 최대 27 |
| Payload | LEN | 명령 데이터 |
| Checksum | 1 | `CMD + LEN + 모든 Payload`의 하위 8비트 |

| CMD | 이름 | Payload |
|---:|---|---|
| `0x01` | SET_GOAL_POS | `motor_id, pos_l, pos_h` |
| `0x02` | SET_TORQUE | `0` 또는 `1` |
| `0x03` | REQ_STATUS | 없음 |
| `0x04` | HOME_POS | 위치 4개, Little Endian 8바이트 |
| `0x05` | SET_ALL_POS | 위치 4개, Little Endian 8바이트 |
| `0x83` | STATUS_REPLY | 위치, 전압, 부하 총 20바이트 |

버튼 15의 실시간 미러링은 `SET_ALL_POS(0x05)`를 사용합니다. 위치 배열의
순서는 Master 1, 2, 3, 4이며 AX_SLAVE가 같은 인덱스를 ID 1, 2, 3, 5에
매핑합니다.

## 보드 간 유선 테스트

Bluetooth를 제거하고 먼저 UART6를 직접 시험하는 것이 좋습니다.

| AX_MASTER | AX_SLAVE |
|---|---|
| PA11 / USART6_TX | PC7 / USART6_RX |
| PA12 / USART6_RX | PC6 / USART6_TX |
| GND | GND |

상태 회신을 사용하지 않는 단방향 위치 시험은 PA11→PC7과 GND만으로도
가능하지만, 전체 프로토콜 시험은 TX/RX를 모두 교차 연결합니다.

## HC-05 연결

| AX_MASTER | Master HC-05 |
|---|---|
| PA11 / USART6_TX | RXD |
| PA12 / USART6_RX | TXD |
| GND | GND |
| 5V | VCC |

일반적인 레귤레이터 내장 HC-05 브레이크아웃 기준입니다. 데이터 모드는
115200이며, Master HC-05는 `ROLE=1`, `CMODE=0`으로 설정하고 Slave
HC-05 주소를 `BIND`합니다.

## TelePlot

ST-LINK Virtual COM Port를 115200으로 열면 다음 네 값만 그래프로
표시됩니다.

```text
>master_1_pos:512
>master_2_pos:512
>master_3_pos:512
>master_4_pos:512
```

## 주요 파일

| 파일 | 역할 |
|---|---|
| `AX_MASTER.ioc` | CubeMX 핀, UART, NVIC 설정 |
| `Core/Inc/ax12.h` | 마스터 ID와 AX-12 API |
| `Core/Src/ax12.c` | AX-12 Protocol 1.0 및 비동기 위치 읽기 |
| `Core/Src/freertos.c` | 태스크, Fusion 프로토콜, TelePlot, 키패드 |
| `Core/Src/usart.c` | USART1/2/6 초기화 |

`Core/Src/freertos - 사용금지.c`는 현재 빌드에 사용하지 않는 구형
백업 파일입니다.

## 빌드

STM32CubeIDE에서 `AX_MASTER.ioc`를 열어 빌드하거나 CMake preset을
사용합니다.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

결과 파일:

```text
build/Debug/AX_MASTER.elf
```

다른 경로에서 복사한 프로젝트라 CMake cache 오류가 나면 `build`
디렉터리를 삭제한 뒤 다시 configure합니다.

## 테스트 순서

1. 마스터 AX-12A를 한 개씩 연결해 ID와 1 Mbps 응답을 확인합니다.
2. TelePlot에서 네 `master_n_pos`가 손 움직임을 따라가는지 확인합니다.
3. UART6 유선 교차 연결 후 양쪽 보드를 Reset합니다.
4. AX_MASTER 키패드 버튼 15를 눌러 JOG 모드로 진입합니다.
5. 슬레이브 콘솔에서 유효 프레임과 Torque ON 상태를 확인합니다.
6. 슬레이브 모터 1, 2, 3, 5가 각각 10, 11, 12, 14를 따라가는지 봅니다.
7. 유선 시험이 성공한 뒤 HC-05 두 개로 교체합니다.

## 안전 및 문제 해결

- 현재 테스트 소스의 `EMERGENCY_STOP_ENABLED`는 `0`입니다. 실제 로봇
  운전 전 안전 로직과 기구적 비상정지를 반드시 별도로 검증하십시오.
- 네 모터가 모두 읽히지 않으면 외부 전원, 공통 GND, PA9 DATA, 1 Mbps를
  확인합니다.
- 일부 모터만 실패하면 ID 중복, 데이지체인 케이블, 전압 강하를
  확인합니다.
- 슬레이브가 반응하지 않으면 버튼 15, USART6 핀 교차 연결, 양쪽
  115200, 공통 GND를 순서대로 확인합니다.
- 움직임이 밀리면 송신 주기를 무작정 줄이기보다 AX-12 읽기 실패,
  Bluetooth 재전송, 슬레이브 램핑 설정을 함께 확인합니다.
