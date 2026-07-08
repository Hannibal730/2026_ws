#include <Arduino.h>
#include <util/atomic.h>

// Encoder PPR/CPR measurement sketch.
// Upload this alone, reset counters with 'r', rotate exactly one revolution,
// then press 'p' to print the measured counts.

#define SERIAL_BAUD 115200
#define ENCODER_A 18
#define ENCODER_B 19

volatile long quad_count = 0;  // Signed x4 quadrature count.
volatile byte last_ab = 0;

volatile unsigned long a_rising = 0;
volatile unsigned long a_falling = 0;
volatile unsigned long b_rising = 0;
volatile unsigned long b_falling = 0;
volatile unsigned long invalid_transition = 0;

unsigned long last_print_ms = 0;

void resetCounters() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    quad_count = 0;
    a_rising = 0;
    a_falling = 0;
    b_rising = 0;
    b_falling = 0;
    invalid_transition = 0;

    byte a = digitalRead(ENCODER_A);
    byte b = digitalRead(ENCODER_B);
    last_ab = (a << 1) | b;
  }

  Serial.println();
  Serial.println(F("RESET"));
  Serial.println(F("Rotate the encoder/wheel exactly 1 revolution, then send 'p'."));
}

void updateEncoder() {
  byte a = digitalRead(ENCODER_A);
  byte b = digitalRead(ENCODER_B);

  byte previous = last_ab;
  byte current = (a << 1) | b;
  byte changed = previous ^ current;
  byte transition = (previous << 2) | current;

  if (changed & 0b10) {
    if (a) a_rising++;
    else a_falling++;
  }
  if (changed & 0b01) {
    if (b) b_rising++;
    else b_falling++;
  }

  switch (transition) {
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      quad_count++;
      break;

    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      quad_count--;
      break;

    default:
      if (current != previous) invalid_transition++;
      break;
  }

  last_ab = current;
}

void snapshot(long &count, unsigned long &ar, unsigned long &af,
              unsigned long &br, unsigned long &bf, unsigned long &bad) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count = quad_count;
    ar = a_rising;
    af = a_falling;
    br = b_rising;
    bf = b_falling;
    bad = invalid_transition;
  }
}

void printMeasurement() {
  long count;
  unsigned long ar;
  unsigned long af;
  unsigned long br;
  unsigned long bf;
  unsigned long bad;
  snapshot(count, ar, af, br, bf, bad);

  unsigned long abs_count = labs(count);
  float estimated_ppr_from_x4 = abs_count / 4.0f;

  Serial.println();
  Serial.println(F("===== ENCODER MEASUREMENT ====="));
  Serial.print(F("X4 signed count (CPR if 1 rev): "));
  Serial.println(count);
  Serial.print(F("X4 absolute count: "));
  Serial.println(abs_count);
  Serial.print(F("Estimated PPR = abs(X4 count) / 4: "));
  Serial.println(estimated_ppr_from_x4, 3);
  Serial.print(F("A rising edges: "));
  Serial.println(ar);
  Serial.print(F("A falling edges: "));
  Serial.println(af);
  Serial.print(F("B rising edges: "));
  Serial.println(br);
  Serial.print(F("B falling edges: "));
  Serial.println(bf);
  Serial.print(F("Invalid transitions: "));
  Serial.println(bad);
  Serial.println(F("==============================="));
}

void printStatus() {
  long count;
  unsigned long ar;
  unsigned long af;
  unsigned long br;
  unsigned long bf;
  unsigned long bad;
  snapshot(count, ar, af, br, bf, bad);

  Serial.print(F("COUNT_X4="));
  Serial.print(count);
  Serial.print(F(" | A_RISE="));
  Serial.print(ar);
  Serial.print(F(" | B_RISE="));
  Serial.print(br);
  Serial.print(F(" | BAD="));
  Serial.println(bad);
}

void printHelp() {
  Serial.println();
  Serial.println(F("Encoder PPR measurement"));
  Serial.println(F("Commands:"));
  Serial.println(F("  r : reset counters"));
  Serial.println(F("  p : print measurement summary"));
  Serial.println(F("  h : print help"));
  Serial.println();
  Serial.println(F("Procedure:"));
  Serial.println(F("  1) Mark the encoder shaft or wheel start position."));
  Serial.println(F("  2) Send 'r'."));
  Serial.println(F("  3) Rotate exactly 1 revolution."));
  Serial.println(F("  4) Send 'p'."));
  Serial.println(F("If measuring wheel PPR through a differential, rotate both drive wheels together."));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ;  // Wait for native USB boards. Harmless on Mega/Uno.
  }

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  byte a = digitalRead(ENCODER_A);
  byte b = digitalRead(ENCODER_B);
  last_ab = (a << 1) | b;

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);

  printHelp();
  resetCounters();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      resetCounters();
    } else if (c == 'p' || c == 'P') {
      printMeasurement();
    } else if (c == 'h' || c == 'H') {
      printHelp();
    }
  }

  unsigned long now = millis();
  if (now - last_print_ms >= 500) {
    last_print_ms = now;
    printStatus();
  }
}
