#define PIN1 6
#define PIN2 5

void setup() {
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
}

void loop() {
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, HIGH);
  delay(2000);
  digitalWrite(PIN1, LOW);
  analogWrite(PIN2, 128);
  delay(2000);
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);
  delay(2000);
  digitalWrite(PIN1, HIGH);
  digitalWrite(PIN2, LOW);
  delay(2000);
  analogWrite(PIN1, 128);
  digitalWrite(PIN2, LOW);
  delay(2000);
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);
  delay(2000);
}
