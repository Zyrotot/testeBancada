#include <Arduino.h>
#include <OneWire.h>  
#include <DallasTemperature.h>

#define stepsPerRevolution 1600 // define quantidades de step por volta
#define stepDelay 100 // define delay entre steps

double flow; //Liters of passing water volume
unsigned long pulse_freq;

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
#define sensorNivel 39 

OneWire oneWire(tempSensor);
DallasTemperature sensors(&oneWire);

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

int readAnalog(int pin) {
  return analogRead(pin);
}

void setOutput(int pin) {
  pinMode(pin,OUTPUT);
  digitalWrite(pin, HIGH);  
}

void pulse () // Interrupt function

{
  pulse_freq++;
}

void setup() {
  // setOutput(pinoExaustor);

  pinMode(fluxoSensor, INPUT);
  attachInterrupt(0, pulse, RISING); // Setup Interrupt

  // sensors.begin();
  // controlMotor(XdirPin, XstepPin);
  // controlMotor(YdirPin, YstepPin);
  // controlMotor(ZdirPin, ZstepPin);
  Serial.begin(9600);
}

void loop() {
  // Serial.println(readPin(portaSensor));
  // Serial.println(readPin(sensorNivel));

  flow = .00225 * pulse_freq;
  Serial.print(flow, DEC);
  Serial.println(" L");
  
  // sensors.requestTemperatures();
  // Serial.println(sensors.getTempCByIndex(0));
  delay(500);
}
