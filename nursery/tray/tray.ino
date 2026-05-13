nconst int r1dirPin  = 2;
const int r1stepPin = 3;

const int r2dirPin  = 4;
const int r2stepPin = 5;

const int r1en = 13;
const int r2en = 12;

void setup() {
  pinMode(r1dirPin, OUTPUT);
  pinMode(r1stepPin, OUTPUT);
  pinMode(r1en, OUTPUT);

  pinMode(r2dirPin, OUTPUT);
  pinMode(r2stepPin, OUTPUT);
  pinMode(r2en, OUTPUT);

  // TB6600 기준: 보통 LOW가 Enable
  digitalWrite(r1en, LOW);
  digitalWrite(r2en, LOW);
}

// r1 구동 함수 pps는 낮을수록 빠름
void r1_move(bool dir, int speedUs) {
  digitalWrite(r1dirPin, dir);

  digitalWrite(r1stepPin, HIGH);
  delayMicroseconds(speedUs);
  digitalWrite(r1stepPin, LOW);
  delayMicroseconds(speedUs);
}

// r2 구동 함수 pps는 낮을수록 빠름
void r2_move(bool dir, int speedUs) {
  digitalWrite(r2dirPin, dir);

  digitalWrite(r2stepPin, HIGH);
  delayMicroseconds(speedUs);
  digitalWrite(r2stepPin, LOW);
  delayMicroseconds(speedUs);
}

void loop() {
  r1_move(HIGH, 500);
  r2_move(HIGH, 500);
}