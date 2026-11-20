#include <Servo.h>
Servo probe;

void setup() {
  Serial.begin(115200);
  probe.attach(7);
  Serial.println(F("Type an angle 0-180 and press Enter"));
}

void loop() {
  if (!Serial.available()) return;
  int a = Serial.parseInt();
  while (Serial.available()) Serial.read();
  if (a >= 0 && a <= 180) {
    probe.write(a);
    Serial.print(F("Angle: "));
    Serial.println(a);
  }
}
