#include <Arduino.h> 
#include <OneWire.h>   
#include <DallasTemperature.h> 
 
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
#define portaSensor 4 
#define pinoExaustor 5  
#define sensorNivel 39  
#define fluxoSensor 19 
 
 
// Variáveis para o cálculo do fluxo 
volatile int pulseCount; 
unsigned int flowMilliLitres; 
unsigned long totalMilliLitres; 
 
// Variáveis para o tempo 
unsigned long startTime; 
unsigned long currentTime; 
 
OneWire oneWire(tempSensor); 
DallasTemperature sensors(&oneWire); 
 
int readPin(int pin) { 
  pinMode(pin, INPUT); // set the pin to input mode 
  return digitalRead(pin); // return the read value of the pin 
} 
 
// Interrupção acionada pelo sensor 
void IRAM_ATTR flowInterrupt() 
{ 
  // Incrementar o contador de pulsos 
  pulseCount++; 
} 
 
bool validateDoor(int dirPin1, int stepPin1, int dirPin2, int stepPin2) { 
  pinMode(stepPin1, OUTPUT); // define o pino de step como saida 
  pinMode(dirPin1, OUTPUT);  // define o pino de direção como saida 
 
  pinMode(stepPin2, OUTPUT); // define o pino de step como saida 
  pinMode(dirPin2, OUTPUT);  // define o pino de direção como saida 
 
  if (readPin(portaSensor) == 1) { 
    return false; 
  } 
 
  digitalWrite(dirPin1, HIGH); // direção do motor sentido horário 
 
  for (int i = 0; i < stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin1, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin1, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  if (readPin(portaSensor) == 0) { 
    return false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(dirPin1, LOW); // direção do motor sentido anti-horário 
 
  for (int i = 0; i < stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin1, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin1, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  if (readPin(portaSensor) == 1) { 
    return false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(dirPin2, HIGH); // direção do motor sentido horário 
 
  for (int i = 0; i < stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin2, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin2, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  if (readPin(portaSensor) == 0) { 
    return false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(dirPin2, LOW); // direção do motor sentido anti-horário 
 
  for (int i = 0; i < stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin2, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin2, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  if (readPin(portaSensor) == 0) { 
    return false; 
  } 
 
  delay(500); // espera 0.5 segundos 
  return true; 
} 
 
bool validateLevel(int dirPin, int stepPin) { 
  pinMode(stepPin, OUTPUT); // define o pino de step como saida 
  pinMode(dirPin, OUTPUT);  // define o pino de direção como saida 
 
  if (readPin(sensorNivel) == 1) { 
    return false; 
  } 
 
  digitalWrite(dirPin, HIGH); // direção do motor sentido horário 
 
  for (int i = 0; i <
stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  if (readPin(sensorNivel) == 0) { 
    return false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(dirPin, LOW); // direção do motor sentido anti-horário 
 
  for (int i = 0; i < stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  if (readPin(sensorNivel) == 1) { 
    return false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  return true; 
} 
 
bool validateTemperature() { 
  sensors.begin(); 
  sensors.requestTemperatures(); 
  if (sensors.getTempCByIndex(0) != 0) { 
    return true; 
  } 
 
  return false; 
} 
 
bool validateFlow() { 
  // Configurar a porta do sensor como entrada 
  pinMode(fluxoSensor, INPUT); 
 
  // Definir a função de interrupção para a porta do sensor 
  attachInterrupt(digitalPinToInterrupt(fluxoSensor), flowInterrupt, RISING); 
 
  // Inicializar as variáveis 
  pulseCount = 0; 
  flowMilliLitres = 0; 
  totalMilliLitres = 0; 
  startTime = millis(); 
 
  pinMode(pinoExaustor, OUTPUT); 
  digitalWrite(pinoExaustor, HIGH); 
 
  unsigned long startTime = millis(); 
  unsigned long currentTime; 
  unsigned long elapsedTime; 
 
  while (millis() - startTime < 5100) { 
    // Disable the interrupt 
    detachInterrupt(digitalPinToInterrupt(fluxoSensor)); 
 
    // Read the pulse count and reset it 
    unsigned int pulseCountCopy = pulseCount; 
    pulseCount = 0; 
 
    // Calculate the elapsed time since the last reading 
    currentTime = millis(); 
    elapsedTime = currentTime - startTime; 
 
    // Calculate the flow volume in milliliters 
    flowMilliLitres = (pulseCountCopy * 60UL * 1000UL) / (elapsedTime * 7.5); 
 
    // Increment the total milliliters 
    totalMilliLitres += flowMilliLitres; 
 
    // Enable the interrupt again 
    attachInterrupt(digitalPinToInterrupt(fluxoSensor), flowInterrupt, RISING); 
 
    delay(500); 
  } 
 
  digitalWrite(pinoExaustor, LOW); 
 
  if (totalMilliLitres > 0) { 
    return true; 
  } else { 
    return false; 
  } 
} 
 
void setup() { 
  Serial.begin(9600);
} 
 
void loop() { 
   
}
