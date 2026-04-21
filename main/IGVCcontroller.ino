#define STEER_PIN 2
#define THROTTLE_PIN 3

void setup() {
  Serial.begin(115200);
  pinMode(STEER_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
}

void loop() {
  unsigned long steer = pulseIn(STEER_PIN, HIGH, 30000);
  unsigned long throttle = pulseIn(THROTTLE_PIN, HIGH, 30000);

  Serial.print("Steer: ");
  Serial.print(steer);
  Serial.print("  Throttle: ");
  Serial.println(throttle);

  delay(20);
}
