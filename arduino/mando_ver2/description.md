# mando_ver2 Arduino Code Description

## 개요

이 스케치는 DMC-16 기반 차량의 주행 모터, 조향 모터, RC 수신기 입력, 시리얼 자동주행 명령, LS7366R SPI 휠 엔코더를 제어한다.

`mando_ver1`과 달리 휠 엔코더 A/B 신호를 Arduino 인터럽트 핀에서 직접 세지 않고, LS7366R 전용 카운터 IC를 SPI로 읽는다.

## 주요 기능

- RC 수신기 PWM 입력을 인터럽트로 측정한다.
- Manual / Auto 모드 PWM 신호에 따라 동작 모드를 선택한다.
- 조향 포텐셔미터 값을 각도로 변환해 PID 조향 제어를 수행한다.
- 시리얼 명령으로 자동주행 throttle / steering angle 명령을 받는다.
- LS7366R을 quadrature x4, 4-byte counter 모드로 초기화한다.
- LS7366R 카운터 값을 주기적으로 읽고 encoder fault guard로 비정상적인 큰 점프를 필터링한다.
- 엔코더 카운트, 변화량, 샘플 간격, 조향 포텐셔미터 값을 약 100 Hz로 시리얼 출력한다.
- 시작 시 조향을 0도 근처로 1회 센터링한다.

## 핀 설정

| 기능 | 핀 |
| --- | --- |
| RC 조향 PWM 입력 | 2 |
| RC 가감속 PWM 입력 | 3 |
| Manual 모드 PWM 입력 | 20 |
| Auto 모드 PWM 입력 | 21 |
| LS7366R CS | 53 |
| 구동 모터 1 DIR / PWM | 10 / 11 |
| 구동 모터 2 DIR / PWM | 6 / 7 |
| 조향 모터 DIR / PWM | 8 / 9 |
| 조향 포텐셔미터 | A0 |

SPI 통신은 `LS7366R_SPI_HZ = 2000000UL`, `MSBFIRST`, `SPI_MODE0` 설정을 사용한다.

## 동작 모드

### Break Mode

- 기본 모드이다.
- `Auto_us`와 `Manual_us`가 모두 1600 us 이하일 때 선택된다.
- 모든 구동 모터와 조향 모터를 정지한다.

### Manual Mode

- `Manual_us > 1600`이고 `Auto_us <= 1600`일 때 선택된다.
- RC 가감속 입력으로 전진/후진을 제어한다.
- RC 조향 입력을 목표 조향각으로 변환하고 PID로 조향 모터를 제어한다.
- 수동 모드에서는 throttle 입력 방향이 리모컨 기준에 맞게 전/후진 반전되어 있다.

### Auto Mode

- `Auto_us > 1600`일 때 선택된다.
- 시리얼 명령 `TH`로 주행 throttle을 받고, `SA`로 목표 조향각을 받는다.
- `TH` 또는 `SA` 명령이 500 ms 이상 갱신되지 않으면 해당 명령은 timeout 처리된다.
- throttle timeout 시 주행은 정지한다.
- 조향 명령 timeout 시 RC 조향 입력 기반 PID로 fallback 한다.
- 전/후진 방향 전환 시 `DIR_DEADTIME_US = 200` us 동안 PWM을 끄는 deadtime을 둔다.

## LS7366R 엔코더 처리

- CS 핀은 53번이다.
- `MDR0`는 quadrature x4 모드로 설정한다.
- `MDR1`은 4-byte counter 모드로 설정한다.
- `R` 또는 `r` 명령이 들어오면 LS7366R 카운터와 내부 guard 상태를 함께 초기화한다.
- `LS7366R_REVERSE_COUNT` 값을 바꾸면 엔코더 카운트 방향을 반전할 수 있다.

## Encoder Fault Guard

비정상적인 엔코더 점프를 줄이기 위한 필터가 들어 있다.

| 항목 | 값 |
| --- | --- |
| 활성화 | `ENCODER_FAULT_GUARD_ENABLE = 1` |
| 10 ms당 최대 허용 변화량 | 60 counts |
| 최소 허용 변화량 | 4 counts |
| 최소 dt | 1 ms |
| 최대 dt | 200 ms |
| reset 후 동기화 드롭 샘플 | 1 |

큰 점프가 감지되면 해당 raw delta를 버리고, `encoderFaultRejects`를 증가시킨다.

## 시리얼 명령

시리얼 baud rate는 `115200`이다.

| 명령 | 의미 |
| --- | --- |
| `TH <value>` | 자동주행 throttle 명령. 범위는 -1.0 ~ 1.0 |
| `SA <deg>` | 자동주행 조향각 명령. 범위는 -24 ~ 24 deg |
| `R` 또는 `r` | LS7366R 카운터와 엔코더 guard 초기화 |
| `S` 또는 `s` | `START` 마크 출력 |
| `P`, `p`, `SPACE`, `space` | `SPACE` 마크 출력 |

## 시리얼 출력

reset 직후 동기화 샘플 출력:

```text
ENC_SYNC,<elapsed_ms>,<encoder_count>,POT,<pot_adc>
```

일반 엔코더 스트리밍 출력:

```text
ENC,<elapsed_ms>,<encoder_count>,<delta_count>,<sample_dt_ms>,POT,<pot_adc>
```

디버그 출력은 `ENABLE_VERBOSE_DEBUG`가 `1`이면 약 100 ms마다 출력된다. 현재 코드는 디버그가 꺼져 있다.

## 제어 파라미터

| 항목 | 값 |
| --- | --- |
| 조향 최대각 | +/- 24 deg |
| 조향 PID | KP 0.05, KI 0.0, KD 0.001 |
| PID deadband | 0.10 |
| 조향 오차 deadband | 0.8 deg |
| 조향 PWM gain / scale | 0.80 / 0.60 |
| 최소 조향 PWM | 35 |
| 제어 루프 주기 | 2 ms |
| 엔코더 출력 주기 | 10 ms |

## 주의 사항

- Arduino 스케치 폴더명과 메인 `.ino` 파일명이 모두 `mando_ver2`로 맞춰져 있다.
- 이 코드는 LS7366R SPI 엔코더 하드웨어가 연결된 구성을 전제로 한다. 엔코더 A/B를 Arduino 18/19번 핀에 직접 연결하는 구성은 `mando_ver1`을 사용한다.
- `setup()`에서 `CenterSteeringOnce()`가 실행되어 부팅 직후 조향 모터가 최대 1.2초 동안 움직일 수 있다.
- 외부 프로그램이 엔코더 CSV를 파싱한다면 `mando_ver1`과 출력 포맷이 다르다는 점을 반영해야 한다.
