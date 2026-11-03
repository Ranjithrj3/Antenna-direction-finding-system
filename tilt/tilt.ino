#include <ESP32Servo.h>

Servo myServo;

void setup() {
  Serial.begin(115200);

  myServo.setPeriodHertz(50);
  myServo.attach(18, 500, 2400);

  delay(500);
  Serial.println("Servo test started");
}

void loop() {
  myServo.write(30);
  Serial.println("Position: 30 degrees");
  delay(2000);

  myServo.write(150);
  Serial.println("Position: 150 degrees");
  delay(2000);
}