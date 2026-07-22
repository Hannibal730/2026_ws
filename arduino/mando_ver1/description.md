# mando_ver1 Arduino Code Description

## 개요

이 스케치는 DMC-16 기반 차량의 주행 모터, 조향 모터, RC 수신기 입력, 시리얼 자동주행 명령, 휠 엔코더 스트리밍을 제어한다.

기존 `mando_final` 코드와 동일한 직접 A/B 쿼드러처 엔코더 방식이며, LS7366R SPI 엔코더 카운터를 사용하지 않는다.

## 주요 기능

- RC 수신기 PWM 입력을 인터럽트로 측정한다.
- Manual / Auto 모드 PWM 신호에 따라 동작 모드를 선택한다.
- 조향 포텐셔미터 값을 각도로 변환해 PID 조향 제어를 수행한다.
- 시리얼 명령으로 자동주행 throttle / steering angle 명령을 받는다.
- 엔코더 A/B 채널을 CHANGE 인터럽트로 읽어 쿼드러처 카운트를 계산한다.
- 엔코더 카운트와 조향 포텐셔미터 값을 약 100 Hz로 시리얼 출력한다.
- 시작 시 조향을 0도 근처로 1회 센터링한다.

## 핀 설정

| 기능 | 핀 |
| --- | --- |
| RC 조향 PWM 입력 | 2 |
| RC 가감속 PWM 입력 | 3 |
| 휠 엔코더 A | 18 |
| 휠 엔코더 B | 19 |
| Manual 모드 PWM 입력 | 20 |
| Auto 모드 PWM 입력 | 21 |
| 구동 모터 1 DIR / PWM | 10 / 11 |
| 구동 모터 2 DIR / PWM | 6 / 7 |
| 조향 모터 DIR / PWM | 8 / 9 |
| 조향 포텐셔미터 | A0 |

## 동작 모드

### Break Mode

- 기본 모드이다.
- `Auto_us`와 `Manual_us`가 모두 1600 us 이하일 때 선택된다.
- 모든 구동 모터와 조향 모터를 정지한다.

### Manual Mode

- `Manual_us > 1600`이고 `Auto_us <= 1600`일 때 선택된다.
- RC 가감속 입력으로 전진/후진을 제어한다.
- RC 조향 입력을 목표 조향각으로 변환하고 PID로 조향 모터를 제어한다.
- 코드 주석 기준으로 리모컨 조작 방향에 맞추기 위해 수동 모드 전/후진 방향이 반전되어 있다.

### Auto Mode

- `Auto_us > 1600`일 때 선택된다.
- 시리얼 명령 `TH`로 주행 throttle을 받고, `SA`로 목표 조향각을 받는다.
- `TH` 또는 `SA` 명령이 500 ms 이상 갱신되지 않으면 해당 명령은 timeout 처리된다.
- throttle timeout 시 주행은 정지한다.
- 조향 명령 timeout 시 RC 조향 입력 기반 PID로 fallback 한다.

## 시리얼 명령

시리얼 baud rate는 `115200`이다.

| 명령 | 의미 |
| --- | --- |
| `TH <value>` | 자동주행 throttle 명령. 범위는 -1.0 ~ 1.0 |
| `SA <deg>` | 자동주행 조향각 명령. 범위는 -24 ~ 24 deg |
| `R` 또는 `r` | 엔코더 카운트 초기화 |
| `S` 또는 `s` | `START` 마크 출력 |
| `P`, `p`, `SPACE`, `space` | `SPACE` 마크 출력 |

## 시리얼 출력

엔코더 스트리밍 출력 형식:

```text
<elapsed_ms>,ENC,<encoder_count>,POT,<pot_adc>
```

디버그 출력은 `ENABLE_VERBOSE_DEBUG`가 `1`이면 약 100 ms마다 출력된다. 현재 코드는 디버그가 켜져 있다.

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

- Arduino 스케치 폴더가 `mando_ver1`이면 메인 `.ino` 파일명도 보통 `mando_ver1.ino`로 맞추는 것이 안전하다.
- 이 코드는 `ENCODER_A=18`, `ENCODER_B=19` 직접 인터럽트 입력을 사용한다. LS7366R SPI 엔코더 하드웨어를 쓰는 구성에서는 `mando_ver2` 코드를 사용해야 한다.
- `setup()`에서 `CenterSteeringOnce()`가 실행되어 부팅 직후 조향 모터가 최대 1.2초 동안 움직일 수 있다.
- `ENABLE_VERBOSE_DEBUG`가 켜져 있으면 엔코더 스트리밍 외 디버그 로그가 섞여 나온다. 외부 프로그램이 CSV만 기대한다면 이 값을 `0`으로 바꾸는 것이 좋다.
