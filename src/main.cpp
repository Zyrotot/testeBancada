#include <Arduino.h>

#define dirPin 2 // define pino de direção do motor
#define stepPin 3 // define pino de step do motor
#define stepsPerRevolution 1600 // define quantidades de step por volta
#define stepDelay 100 // define delay entre steps

void setup() {
  pinMode(stepPin, OUTPUT); // define o pino de step como saida
  pinMode(dirPin, OUTPUT); // define o pino de direção como saida
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