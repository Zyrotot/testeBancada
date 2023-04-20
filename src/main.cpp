#include <Arduino.h>
#include <LiquidCrystal.h> // Biblioteca para utilizar o display lcd

// Ligações no Arduino do display
// const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
// LiquidCrystal lcd (rs, en, d4, d5, d6, d7);

#define dirPin 2 // classifica pino que define a direção do motor
#define stepPin 3
#define stepsPerRevolution 1600
#define stepDelay 100

void setup() {

  // lcd.begin(16, 2); // Aqui informa que são 16 caracteres e 2 linhas
  // lcd.setCursor(0, 0); // Colando pra exibir a frase a partir da coluna 0 e linha 0 (opcional)
  // lcd.print("Bancada de Teste");
  // lcd.serCursor(0, 1); // Colando pra exibir a frase a partir da coluna 0 e linha 1
  // lcd.print("Due Laser");

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