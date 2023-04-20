#include <Arduino.h>

#define dirPin 2 // classifica pino que define a direção do motor
#define stepPin 3
#define stepsPerRevolution 1600
#define stepDelay 100

void setup() {

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  digitalWrite(dirPin, HIGH); // direção do motor sentido horário

  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo
  }

  delay(1000); // espera 1 segundo

  digitalWrite(dirPin, LOW); // direção do motor sentido anti-horário

  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo
  }
}