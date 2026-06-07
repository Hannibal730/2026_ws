// 엔코더 검정 → Arduino GND
// 엔코더 빨강 → Arduino 5V
// 엔코더 주황 → Arduino 18
// 엔코더 갈색 → Arduino 19

#define ENCODER_A 18
#define ENCODER_B 19

volatile long encoderCount = 0;

void encoderISR_A() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void encoderISR_B() {
  if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(57600);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR_B, CHANGE);

  Serial.println("Encoder test start");
}

void loop() {
  long nowCount;

  noInterrupts();
  nowCount = encoderCount;
  interrupts();

  Serial.print("ENC: ");
  Serial.println(nowCount);

  delay(50);
}