void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    s.toLowerCase();
    if (s == "stop") {
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (s == "clear") {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}