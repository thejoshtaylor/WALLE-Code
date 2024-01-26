#define FPWM 6
#define BPWM 5
#define FEN 2
#define BEN 4

void setup() {
  pinMode(FPWM, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(FEN, OUTPUT);
  pinMode(BEN, OUTPUT);

  digitalWrite(FEN, HIGH);
  digitalWrite(BEN, HIGH);
}

void loop() {
  analogWrite(FPWM, 255);
  analogWrite(BPWM, 0);
  delay(2000);
  analogWrite(FPWM, 128);
  analogWrite(BPWM, 0);
  delay(2000);
  analogWrite(FPWM, 0);
  analogWrite(BPWM, 0);
  delay(2000);
  analogWrite(FPWM, 0);
  analogWrite(BPWM, 255);
  delay(2000);
  analogWrite(FPWM, 0);
  analogWrite(BPWM, 128);
  delay(2000);
  analogWrite(FPWM, 0);
  analogWrite(BPWM, 0);
  delay(2000);
}
