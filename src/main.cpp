#include <Arduino.h>

#define stepsPerRevolution 1600 // define quantidades de step por volta
#define stepDelay 100 // define delay entre steps

// Motor eixo X

#define XdirPin 27 // define pino de direção do motor
#define XstepPin 26 // define pino de step do motor

// Motor eixo Y

#define YdirPin 32 // define pino de direção do motor
#define YstepPin 33 // define pino de step do motor

// Motor eixo Z

#define ZdirPin 12 // define pino de direção do motor
#define ZstepPin 14 // define pino de step do motor

// Sensores

#define tempSensor 21
#define fluxoSensor 19
#define portaSensor 4
#define pinoExaustor 5

void controlMotor(int dirPin, int stepPin) {
  pinMode(stepPin, OUTPUT); // define o pino de step como saida
  pinMode(dirPin, OUTPUT); // define o pino de direção como saida

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

int readPin(int pin) {
  pinMode(pin, INPUT); // set the pin to input mode
  return digitalRead(pin); // return the read value of the pin
}

void setOutput(int pin) {
  pinMode(pin,OUTPUT);
  digitalWrite(pin, HIGH);
}

void setup() {
  controlMotor(XdirPin, XstepPin);
  controlMotor(YdirPin, YstepPin);
  controlMotor(ZdirPin, ZstepPin);
  Serial.begin(9600);
}

void loop() {

}
