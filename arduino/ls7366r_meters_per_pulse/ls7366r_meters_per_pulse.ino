#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <stdint.h>
#include <util/atomic.h>

#define SERIAL_BAUD 115200
#define ENABLE_VERBOSE_DEBUG 1
#define ENCODER_STREAM_INTERVAL_MS 10
#define ENCODER_STREAM_MIN_TX_SPACE 48
#define LOOP_DELAY_MS 2

#define STEERING_PULSE_PIN 2
#define ACCEL_PULSE_PIN 3
#define MANUAL_MODE_PIN 20
#define AUTO_MODE_PIN 21

#define LS7366R_CS_PIN 53
#define LS7366R_EN_PIN 5
#define LS7366R_SPI_HZ 2000000UL
#define LS7366R_WRITE_MDR0 0x88
#define LS7366R_WRITE_MDR1 0x90
#define LS7366R_READ_CNTR 0x60
#define LS7366R_MDR0_QUADRATURE_X4 0x03
#define LS7366R_MDR1_4_BYTE_COUNTER 0x00
#define LS7366R_REVERSE_COUNT 0

#define BREAK_MODE 200
#define MANUAL_MODE 1400
#define AUTO_MODE 1700

#define POT_MAX 1021
#define POT_MIN 0
#define MAX_STEER_TIRE_DEG 24

#define KP 0.05
#define KI 0.0
#define KD 0.001
#define PID_DEADBAND 0.10
#define PID_INTEGRAL_LIMIT 80.0
#define STEER_PWM_GAIN 0.80
#define STEER_PWM_SCALE 0.60
#define STEER_MIN_PWM 35
#define STEER_RATE_LPF_ALPHA 0.25
#define STEER_SENSE_LPF_ALPHA 0.20

#define ACCEL_CENTER_US 1500
#define STEER_CENTER_US 1500
#define ACCEL_DB_US 50
#define STEER_DB_US 25
#define STEER_LEFT_US 1280
#define STEER_RIGHT_US 1792
#define ACCEL_FWD_MAX_US 1804
#define ACCEL_REV_MIN_US 1104
#define RC_THROTTLE_DB_NORM 0.03
#define RC_STEER_DB_NORM 0.1
#define STEER_ERROR_DEADBAND_DEG 0.8

volatile int32_t encoderCount = 0;
long driveLogPrevEncoder = 0;
unsigned long encoderCsvPrevMs = 0;
unsigned long encoderTimeZeroMs = 0;

volatile uint32_t steer_rise_us = 0;
volatile uint32_t accel_rise_us = 0;
volatile uint32_t manual_rise_us = 0;
volatile uint32_t auto_rise_us = 0;
volatile uint16_t Steering_us = 1500;
volatile uint16_t Accel_us = 1500;
volatile uint16_t Manual_us = 1000;
volatile uint16_t Auto_us = 1000;
volatile uint32_t accel_last_us = 0;

int DIR1 = 10;
int PWM1 = 11;
int DIR2 = 6;
int PWM2 = 7;
int DIR3 = 8;
int PWM3 = 9;
int POTPin = A0;

#define MIN_DRIVE_PWM 0
#define MAX_DRIVE_PWM 250

#define SERIAL_BUFFER_SIZE 48
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t bufferIndex = 0;

float throttle_cmd = 0.0f;
float steer_auto_deg = 0.0f;
bool throttleFresh = false;
bool steerFresh = false;
unsigned long lastThrottleMs = 0;
unsigned long lastSteerMs = 0;

#define THROTTLE_TIMEOUT_MS 500
#define STEER_TIMEOUT_MS 500

#define PULSE_MIN 500
#define PULSE_MAX 2500
#define SIGNAL_THRESHOLD 0.05

volatile uint32_t steer_last_us = 0;
volatile uint32_t manual_last_us = 0;
volatile uint32_t auto_last_us = 0;

SPISettings ls7366rSpiSettings(LS7366R_SPI_HZ, MSBFIRST, SPI_MODE0);

void ls7366rWriteRegister(uint8_t command, uint8_t value) {
  SPI.beginTransaction(ls7366rSpiSettings);
  digitalWrite(LS7366R_CS_PIN, LOW);
  SPI.transfer(command);
  SPI.transfer(value);
  digitalWrite(LS7366R_CS_PIN, HIGH);
  SPI.endTransaction();
}

int32_t ls7366rReadCounter() {
  uint32_t raw = 0;

  SPI.beginTransaction(ls7366rSpiSettings);
  digitalWrite(LS7366R_CS_PIN, LOW);
  SPI.transfer(LS7366R_READ_CNTR);
  raw |= (uint32_t)SPI.transfer(0x00) << 24;
  raw |= (uint32_t)SPI.transfer(0x00) << 16;
  raw |= (uint32_t)SPI.transfer(0x00) << 8;
  raw |= (uint32_t)SPI.transfer(0x00);
  digitalWrite(LS7366R_CS_PIN, HIGH);
  SPI.endTransaction();

  int32_t count = (int32_t)raw;
  return LS7366R_REVERSE_COUNT ? -count : count;
}

void ls7366rBegin() {
  pinMode(LS7366R_EN_PIN, OUTPUT);
  digitalWrite(LS7366R_EN_PIN, HIGH);

  pinMode(LS7366R_CS_PIN, OUTPUT);
  digitalWrite(LS7366R_CS_PIN, HIGH);
  SPI.begin();

  ls7366rWriteRegister(LS7366R_WRITE_MDR0, LS7366R_MDR0_QUADRATURE_X4);
  ls7366rWriteRegister(LS7366R_WRITE_MDR1, LS7366R_MDR1_4_BYTE_COUNTER);
}

void Steer(double u) {
  u = constrain(u, -1.0, 1.0);

  if (fabs(u) < PID_DEADBAND) {
    analogWrite(PWM3, 0);
    digitalWrite(DIR3, LOW);
    return;
  }

  int pwm_val = (int)(fabs(u) * 255.0 * STEER_PWM_GAIN * STEER_PWM_SCALE);
  pwm_val = constrain(pwm_val, 0, 255);
  if (pwm_val > 0 && pwm_val < STEER_MIN_PWM) pwm_val = STEER_MIN_PWM;

  if (u > 0) {
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, pwm_val);
  } else {
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, pwm_val);
  }
}

void printEncoderMark(const char *tag) {
  unsigned long now = millis();
  long count;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count = (long)encoderCount;
  }

  Serial.print("MARK,");
  Serial.print(tag);
  Serial.print(",");
  Serial.print(now - encoderTimeZeroMs);
  Serial.print(",");
  Serial.println(count);
}

void parseSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';

      if (strcmp(serialBuffer, "S") == 0 || strcmp(serialBuffer, "s") == 0) {
        printEncoderMark("START");
      } else if (strcmp(serialBuffer, "P") == 0 || strcmp(serialBuffer, "p") == 0 ||
                 strcmp(serialBuffer, "SPACE") == 0 || strcmp(serialBuffer, "space") == 0) {
        printEncoderMark("SPACE");
      } else if (strncmp(serialBuffer, "TH", 2) == 0 || strncmp(serialBuffer, "th", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float v = atof(p);
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        throttle_cmd = v;
        throttleFresh = true;
        lastThrottleMs = millis();
      } else if (strncmp(serialBuffer, "SA", 2) == 0 || strncmp(serialBuffer, "sa", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float a = atof(p);
        if (a > MAX_STEER_TIRE_DEG) a = MAX_STEER_TIRE_DEG;
        if (a < -MAX_STEER_TIRE_DEG) a = -MAX_STEER_TIRE_DEG;
        steer_auto_deg = a;
        steerFresh = true;
        lastSteerMs = millis();
      }

      bufferIndex = 0;
    } else if (c != '\r') {
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0;
      }
    }
  }
}

static inline float Mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline double applyDeadband(double x, double band) {
  return (fabs(x) < band) ? 0.0 : x;
}

static inline float clampFloat(float x, float low, float high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}

static inline double clampDouble(double x, double low, double high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}

static inline float normalizeSteerPulse(long pulse_us) {
  if (pulse_us > STEER_CENTER_US + STEER_DB_US) {
    float v = (float)(pulse_us - (STEER_CENTER_US + STEER_DB_US)) /
              (STEER_RIGHT_US - (STEER_CENTER_US + STEER_DB_US));
    return clampFloat(v, 0.0f, 1.0f);
  }

  if (pulse_us < STEER_CENTER_US - STEER_DB_US) {
    float v = (float)(pulse_us - (STEER_CENTER_US - STEER_DB_US)) /
              ((STEER_CENTER_US - STEER_DB_US) - STEER_LEFT_US);
    return clampFloat(v, -1.0f, 0.0f);
  }

  return 0.0f;
}

const unsigned int DIR_DEADTIME_US = 200;
int last_dir_sign = 0;

void driveWithDeadtime(float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);

  float mag = fabs(cmd);
  int dir_sign = (cmd > 0.0f) ? +1 : (cmd < 0.0f ? -1 : 0);

  if (mag < SIGNAL_THRESHOLD || dir_sign == 0) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    last_dir_sign = 0;
    return;
  }

  if (last_dir_sign != 0 && dir_sign != last_dir_sign) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    delayMicroseconds(DIR_DEADTIME_US);
  }

  if (dir_sign > 0) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
  } else {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
  }

  int in = (int)Mapping(mag, 0.0f, 1.0f, (float)MIN_DRIVE_PWM, (float)MAX_DRIVE_PWM);
  analogWrite(PWM1, in);
  analogWrite(PWM2, in);

  last_dir_sign = dir_sign;
}

void SteeringPulseInt() {
  uint32_t now = micros();
  if (digitalRead(STEERING_PULSE_PIN) == HIGH) {
    steer_rise_us = now;
  } else {
    uint32_t w = now - steer_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Steering_us = (uint16_t)w;
      steer_last_us = now;
    }
  }
}

void AccelPulseInt() {
  uint32_t now = micros();
  if (digitalRead(ACCEL_PULSE_PIN) == HIGH) {
    accel_rise_us = now;
  } else {
    uint32_t w = now - accel_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Accel_us = (uint16_t)w;
      accel_last_us = now;
    }
  }
}

void ManualPulseInt() {
  uint32_t now = micros();
  if (digitalRead(MANUAL_MODE_PIN) == HIGH) {
    manual_rise_us = now;
  } else {
    uint32_t w = now - manual_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Manual_us = (uint16_t)w;
      manual_last_us = now;
    }
  }
}

void AutoPulseInt() {
  uint32_t now = micros();
  if (digitalRead(AUTO_MODE_PIN) == HIGH) {
    auto_rise_us = now;
  } else {
    uint32_t w = now - auto_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Auto_us = (uint16_t)w;
      auto_last_us = now;
    }
  }
}

double PID(double ref, double sense, unsigned long dt_us) {
  static double prev_err = 0.0;
  static double integral = 0.0;

  double dt_s = dt_us * 1.0e-6;
  if (dt_s <= 0.0) dt_s = 1e-6;

  double err = ref - sense;
  if (fabs(err) < STEER_ERROR_DEADBAND_DEG) {
    integral = 0.0;
    prev_err = err;
    return 0.0;
  }

  integral += err * dt_s;
  integral = clampDouble(integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  double P = KP * err;
  double I = KI * integral;
  double D = KD * (err - prev_err) / dt_s;

  prev_err = err;
  return P + I + D;
}

void StopMotor() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
  digitalWrite(DIR3, LOW);
  analogWrite(PWM3, 0);
}

void MoveForward(double throttle) {
  if (throttle > 1.0) throttle = 1.0;
  else if (throttle < 0.0) throttle = 0.0;

  int in = (int)(Mapping(throttle, 0.0, 1.0, MIN_DRIVE_PWM, MAX_DRIVE_PWM));
  if (throttle < 0.01) in = 0;

  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, in);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM2, in);
}

void MoveBackward(double throttle) {
  if (throttle > 1.0) throttle = 1.0;
  else if (throttle < 0.0) throttle = 0.0;

  int in = (int)(Mapping(throttle, 0.0, 1.0, MIN_DRIVE_PWM, MAX_DRIVE_PWM));
  if (throttle < 0.01) in = 0;

  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, in);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, in);
}

void CenterSteeringOnce() {
  const double CENTER_DEG = 0.0;
  const double TOL_DEG = 3;
  const unsigned long TIMEOUT_MS = 1200;

  unsigned long t0 = millis();

  while (millis() - t0 < TIMEOUT_MS) {
    int pot = analogRead(POTPin);
    double deg = Mapping(pot, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);
    double err = CENTER_DEG - deg;

    if (fabs(err) <= TOL_DEG) {
      digitalWrite(DIR3, LOW);
      analogWrite(PWM3, 0);
      break;
    }

    double u = constrain(err / MAX_STEER_TIRE_DEG, -1.0, 1.0);
    u = constrain(u * 0.6, -0.6, 0.6);
    if (fabs(u) < 0.12) u = (u > 0) ? 0.12 : -0.12;
    Steer(u);
    delay(10);
  }

  Steer(0.0);
}

void printEncoderCsv(long count) {
  unsigned long now = millis();

  if (now - encoderCsvPrevMs >= ENCODER_STREAM_INTERVAL_MS) {
    if (Serial.availableForWrite() < ENCODER_STREAM_MIN_TX_SPACE) return;

    unsigned long elapsedMs = now - encoderTimeZeroMs;
    int potVal = analogRead(POTPin);
    encoderCsvPrevMs += ENCODER_STREAM_INTERVAL_MS;
    if (now - encoderCsvPrevMs >= ENCODER_STREAM_INTERVAL_MS) encoderCsvPrevMs = now;

    Serial.print(elapsedMs);
    Serial.print(",ENC,");
    Serial.print(count);
    Serial.print(",POT,");
    Serial.println(potVal);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(STEERING_PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STEERING_PULSE_PIN), SteeringPulseInt, CHANGE);

  pinMode(ACCEL_PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ACCEL_PULSE_PIN), AccelPulseInt, CHANGE);

  pinMode(MANUAL_MODE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MANUAL_MODE_PIN), ManualPulseInt, CHANGE);

  pinMode(AUTO_MODE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(AUTO_MODE_PIN), AutoPulseInt, CHANGE);

  pinMode(POTPin, INPUT);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM3, OUTPUT);

  ls7366rBegin();
  StopMotor();
  digitalWrite(DIR3, LOW);
  analogWrite(PWM3, 0);
  CenterSteeringOnce();

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoderCount = ls7366rReadCounter();
  }
  encoderTimeZeroMs = millis();
  encoderCsvPrevMs = encoderTimeZeroMs;

  Serial.println("ENCODER_START");
  Serial.println("LS7366R SPI Encoder + Remote control start");
}

void loop() {
  parseSerial();

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoderCount = ls7366rReadCounter();
  }

  long encoder_for_stream;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoder_for_stream = (long)encoderCount;
  }
  printEncoderCsv(encoder_for_stream);

  static unsigned long prev_t_us = 0;
  unsigned long t_us = micros();
  if (prev_t_us != 0 && (t_us - prev_t_us) < (unsigned long)LOOP_DELAY_MS * 1000UL) {
    return;
  }
  unsigned long dt_us = (prev_t_us == 0) ? 1000UL : (t_us - prev_t_us);
  prev_t_us = t_us;

  long Steering_us_local;
  long Accel_us_local;
  long Manual_us_local;
  long Auto_us_local;
  long encoder_local;
  uint32_t accel_last_us_local;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    Steering_us_local = Steering_us;
    Accel_us_local = Accel_us;
    Manual_us_local = Manual_us;
    Auto_us_local = Auto_us;
    encoder_local = (long)encoderCount;
    accel_last_us_local = accel_last_us;
  }

  int Mode_val = BREAK_MODE;
  if (Auto_us_local > 1600) Mode_val = AUTO_MODE;
  else if (Manual_us_local > 1600) Mode_val = MANUAL_MODE;

  if (t_us - accel_last_us_local > 30000) Accel_us_local = ACCEL_CENTER_US;

  float Throttle_input = 0.0f;
  if (Accel_us_local > ACCEL_CENTER_US + ACCEL_DB_US) {
    Throttle_input = (float)(Accel_us_local - ACCEL_CENTER_US) /
                     (ACCEL_FWD_MAX_US - ACCEL_CENTER_US);
  } else if (Accel_us_local < ACCEL_CENTER_US - ACCEL_DB_US) {
    Throttle_input = (float)(Accel_us_local - ACCEL_CENTER_US) /
                     (ACCEL_CENTER_US - ACCEL_REV_MIN_US);
  }
  Throttle_input = constrain(Throttle_input, -1.0, 1.0);

  float Steer_rc = normalizeSteerPulse(Steering_us_local);
  Steer_rc = applyDeadband(Steer_rc, RC_STEER_DB_NORM);
  double ref_steer_deg_rc = Mapping(Steer_rc, -1.0, 1.0, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

  int POTval = analogRead(POTPin);
  double raw_deg = Mapping(POTval, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

  static bool steer_sense_init = false;
  static double deg_filtered = 0.0;
  if (!steer_sense_init) {
    steer_sense_init = true;
    deg_filtered = raw_deg;
  } else {
    deg_filtered += STEER_SENSE_LPF_ALPHA * (raw_deg - deg_filtered);
  }
  double deg = deg_filtered;

  static bool steer_rate_init = false;
  static double prev_raw_deg = 0.0;
  static double steer_rate_dps = 0.0;
  double dt_s = dt_us * 1.0e-6;
  if (dt_s <= 0.0) dt_s = 1.0e-6;

  if (!steer_rate_init) {
    steer_rate_init = true;
    prev_raw_deg = raw_deg;
    steer_rate_dps = 0.0;
  } else {
    double rate_raw_dps = (raw_deg - prev_raw_deg) / dt_s;
    steer_rate_dps += STEER_RATE_LPF_ALPHA * (rate_raw_dps - steer_rate_dps);
    prev_raw_deg = raw_deg;
  }

  bool sa_ok = steerFresh && (millis() - lastSteerMs <= STEER_TIMEOUT_MS);
  double ref_steer_deg = (Mode_val == AUTO_MODE && sa_ok) ? (double)steer_auto_deg : ref_steer_deg_rc;

  bool th_ok = throttleFresh && (millis() - lastThrottleMs <= THROTTLE_TIMEOUT_MS);

  double u_rc = 0.0;
  double u_auto = 0.0;

  if (Mode_val == AUTO_MODE && sa_ok) {
    u_auto = PID((double)steer_auto_deg, deg, dt_us);
    u_auto = applyDeadband(u_auto, PID_DEADBAND);
    u_auto = constrain(u_auto, -1.0, 1.0);
  } else {
    u_rc = PID(ref_steer_deg_rc, deg, dt_us);
    u_rc = applyDeadband(u_rc, PID_DEADBAND);
    u_rc = constrain(u_rc, -1.0, 1.0);
  }

  double u_used = (Mode_val == AUTO_MODE && sa_ok) ? u_auto : u_rc;

  if (Mode_val == BREAK_MODE) {
    StopMotor();
  } else if (Mode_val == MANUAL_MODE) {
    if (Throttle_input > 0.05f) MoveBackward(Throttle_input * 0.6f);
    else if (Throttle_input < -0.05f) MoveForward((-Throttle_input) * 0.6f);
    else {
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
    }

    Steer(u_rc);
  } else {
    float th = th_ok ? throttle_cmd : 0.0f;
    driveWithDeadtime(th);

    if (sa_ok) Steer(u_auto);
    else Steer(u_rc);
  }

#if ENABLE_VERBOSE_DEBUG
  static unsigned long lp = 0;
  if (millis() - lp > 100) {
    long d_encoder = encoder_local - driveLogPrevEncoder;
    driveLogPrevEncoder = encoder_local;
    lp = millis();

    Serial.print("MODE:");
    Serial.print(Mode_val);
    Serial.print(" | Tgt:");
    Serial.print(ref_steer_deg, 1);
    Serial.print(" | Cur:");
    Serial.print(deg, 1);
    Serial.print(" | dDeg/s:");
    Serial.print(steer_rate_dps, 1);
    Serial.print(" | PID:");
    Serial.print(u_used, 2);
    Serial.print(" | POT:");
    Serial.print(POTval);
    Serial.print(" | MAN_US:");
    Serial.print(Manual_us_local);
    Serial.print(" | AUTO_US:");
    Serial.print(Auto_us_local);
    Serial.print(" | ENC:");
    Serial.print(encoder_local);
    Serial.print(" | dENC:");
    Serial.print(d_encoder);
    Serial.print(" | TH:");
    Serial.print(throttle_cmd, 2);
    Serial.print(" ok:");
    Serial.print((int)th_ok);
    Serial.print(" | SA:");
    Serial.print(steer_auto_deg, 1);
    Serial.print(" ok:");
    Serial.println((int)sa_ok);
  }
#endif
}
