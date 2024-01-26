#define DIR1 2
#define PWM1 3
#define DIR2 4
#define PWM2 5

void setup() {
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
}

void loop() {
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, 255);
  delay(2000);
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, 128);
  delay(2000);
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, 0);
  delay(2000);
  
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 255);
  delay(2000);
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 128);
  delay(2000);
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  delay(2000);
}
