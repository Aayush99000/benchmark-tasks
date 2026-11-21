// Arduino Nano (5 V). Wiring and door switch are specified in FIXTURE.md.
const byte startPin = 2, doorPin = 3, motorPin = 9;
bool running = false, armed = false;
unsigned long started, releasedAt;

void setup() {
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(doorPin, INPUT);
  releasedAt = millis();
}

void loop() {
  unsigned long now = millis();
  bool closed = digitalRead(doorPin) == HIGH;
  bool pressed = digitalRead(startPin) == LOW;
  if (!closed || (running && now - started >= 30000UL)) {
    running = false;
    armed = false;
    releasedAt = now;
  }
  if (closed && !running && !pressed && now - releasedAt >= 30) armed = true;
  if (pressed) {
    if (closed && armed && !running) {
      running = true;
      started = now;
    }
    armed = false;
    releasedAt = now;
  }
  digitalWrite(motorPin, running ? HIGH : LOW);
}
