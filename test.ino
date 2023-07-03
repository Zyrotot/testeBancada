#include <Arduino.h> 
#include <OneWire.h>   
#include <DallasTemperature.h> 
 
#define stepsPerRevolution 3200 // define quantidades de step por volta 
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
#define sensorFDC 36 
#define fluxoSensor 19 
 
// Saidas
#define laserPin 15
#define safetyPin 2
#define floodPin 13

// Variáveis para o cálculo do fluxo 
volatile int pulseCount; 
unsigned int flowMilliLitres; 
unsigned long totalMilliLitres; 
 
// Variáveis para o tempo 
unsigned long startTime; 
unsigned long currentTime; 

// Variavel para receber o comando serial

int command = 11;
bool commandReceived = false;

// ---------------

OneWire oneWire(tempSensor); 
DallasTemperature sensors(&oneWire); 

// ---------------

int readPin(int pin) { 
  pinMode(pin, INPUT); // set the pin to input mode 
  return digitalRead(pin); // return the read value of the pin 
} 

// ---------------

void IRAM_ATTR flowInterrupt() 
{ 
  // Incrementar o contador de pulsos 
  pulseCount++; 
} 

// ---------------

bool validateDoorX(int dirPin, int stepPin) { 
  bool result = true;

  pinMode(stepPin, OUTPUT); // define o pino de step como saida 
  pinMode(dirPin, OUTPUT);  // define o pino de direção como saida 

  if (readPin(portaSensor) == 1) { 
    result = false; 
  } 
 
  digitalWrite(dirPin, HIGH); // direção do motor sentido horário 
 
  for (int i = 0; i < stepsPerRevolution/4; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay*4); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay*4); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 

  if (readPin(portaSensor) == 0) { 
    result = false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(dirPin, LOW); // direção do motor sentido anti-horário 
 
  for (int i = 0; i < stepsPerRevolution/4; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay*4); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay*4); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(500); // espera 0.5 segundos 

  if (readPin(portaSensor) == 1) { 
    result = false; 
  } 
 
  delay(500); // espera 0.5 segundos 
  
  return result;
} 

// ---------------

bool validateDoorY(int dirPin, int stepPin) { 
  bool result = true;

  pinMode(stepPin, OUTPUT); // define o pino de step como saida 
  pinMode(dirPin, OUTPUT);  // define o pino de direção como saida 

  if (readPin(portaSensor) == 1) { 
    result = false; 
  } 
 
  digitalWrite(dirPin, HIGH); // direção do motor sentido horário 
 
  for (int i = 0; i < stepsPerRevolution; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(1000); // espera 0.5 segundos 

  if (readPin(portaSensor) == 0) { 
    result = false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(dirPin, LOW); // direção do motor sentido anti-horário 
 
  for (int i = 0; i < stepsPerRevolution; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(1000); // espera 0.5 segundos 

  if (readPin(portaSensor) == 1) { 
    result = false; 
  } 
 
  delay(500); // espera 0.5 segundos 
  
  return result;
} 
 
// ---------------

bool validateLevel(int dirPin, int stepPin) {
  bool result = true;

  pinMode(stepPin, OUTPUT); // define o pino de step como saida 
  pinMode(dirPin, OUTPUT);  // define o pino de direção como saida 
 
  if (readPin(sensorNivel) == 0) { 
    result = false; 
  } 
 
  digitalWrite(dirPin, LOW); // direção do motor sentido horário 
 
  for (int i = 0; i <stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(1000); // espera 0.5 segundos 
 
  if (readPin(sensorNivel) == 1) { 
    result = false; 
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(dirPin, HIGH); // direção do motor sentido anti-horário 
 
  for (int i = 0; i < stepsPerRevolution / 2; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(stepDelay); // espera 100 microssegundos = 0,0001 segundo 
  } 
 
  delay(1000); // espera 0.5 segundos 
 
  if (readPin(sensorNivel) == 0) { 
    result = false; 
  } 
  
  return result; 
} 
 
// ---------------

bool validateTemperature() { 
  sensors.begin(); 
  sensors.requestTemperatures(); 
  if (sensors.getTempCByIndex(0) != 0) { 
    return true; 
  } 
 
  return false; 
} 

// ---------------

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

  // Check if any pulses were received
  if (pulseCount == 0) {
    return false;
  }

  if (totalMilliLitres > 0) {
    return true;
  } else {
    return false;
  }
} 

// ---------------

bool validateLaser() {
  pinMode(safetyPin, OUTPUT);
  pinMode(sensorFDC, INPUT);

  digitalWrite(safetyPin, HIGH);
  digitalWrite(laserPin, HIGH);

  delay(2000); // espera 0.5 segundos 
  if (readPin(sensorFDC) == 1) {
    digitalWrite(safetyPin, LOW);
    return false;
  } 
 
  delay(2000); // espera 0.5 segundos 
 
  digitalWrite(safetyPin, LOW);
  digitalWrite(laserPin, LOW);
  if (readPin(sensorFDC) == 0) {
    return false;
  }

  delay(5000); // espera 0.5 segundos 
  return true; 
} 

// --------------

bool validateFlood(int actuatorPin, int sensorPin) { 
  pinMode(floodPin, OUTPUT);
  pinMode(sensorFDC, INPUT);

  digitalWrite(floodPin, HIGH);

  delay(500); // espera 0.5 segundos 
  Serial.println(readPin(sensorFDC));
  if (readPin(sensorFDC) == 1) {
    digitalWrite(floodPin, LOW);
    return false;
  } 
 
  delay(500); // espera 0.5 segundos 
 
  digitalWrite(floodPin, LOW);
  Serial.println(readPin(sensorFDC));
  if (readPin(sensorFDC) == 0) {
    return false;
  }

  delay(5000); // espera 0.5 segundos 
  return true; 
}

// --------------
// Function to run all the tests
void runAllTests() {
  Serial.println("Running all tests...");

  testFlowModule();
  delay(500);

  testLevelModule();
  delay(500);

  testDoorModule();
  delay(500);

  testPumpModule();
  delay(500);

  testLaserModule();
  delay(500);

  testTemperatureModule();
  delay(500);

  Serial.println("All tests completed.");
}

// Function to test the flow module
void testFlowModule() {
  Serial.println("Testing flow module...");

  if (validateFlow()) {
    Serial.println("Sensor de fluxo: OK");
    Serial.println("Exaustor: OK");
  } else {
    Serial.println("Sensor de fluxo: Erro");
    Serial.println("Exaustor: Erro");
  }

  Serial.println("Flow module test completed.");
}

// Function to test the level module
void testLevelModule() {
  Serial.println("Testing level module...");

  if (validateLevel(ZdirPin, ZstepPin)){
    Serial.println("Motor Z: OK");
    Serial.println("Sensor nivel: OK");
  } else {
    Serial.println("Motor Z: Erro");
    Serial.println("Sensor nivel: Erro");
  }

  Serial.println("Level module test completed.");
}

// Function to test the X motor module
void testDoorModule() {
  Serial.println("Testing door module...");

  bool doorY = validateDoorY(YdirPin, YstepPin);
  bool doorX = validateDoorX(XdirPin, XstepPin);
  if (doorX && doorY) {
    Serial.println("Motor X: OK");
    Serial.println("Motor Y: OK");
    Serial.println("Sensor porta: OK");
  } else if (doorX && !doorY) {
    Serial.println("Motor X: OK");
    Serial.println("Motor Y: Erro");
    Serial.println("Sensor porta: OK");
  } else if (!doorX && doorY) {
    Serial.println("Motor X: Erro");
    Serial.println("Motor Y: OK");
    Serial.println("Sensor porta: OK");
  } else {
    Serial.println("Motor X: Erro");
    Serial.println("Motor Y: Erro");
    Serial.println("Sensor porta: Erro");
  }

  Serial.println("Door module test completed.");
}

// Function to test the pump module
void testPumpModule() {
  Serial.println("Testing pump module...");

  bool flood = validateFlood();
  if (flood){
    Serial.println("Saida bomba: OK");
  } else {
    Serial.println("Saida bomba: Erro");
  }

  Serial.println("Pump module test completed.");
}

// Function to test the laser module
void testLaserModule() {
  Serial.println("Testing laser module...");

  bool laser = validateLaser();
  if (laser){
    Serial.println("Saida laser: OK");
  } else {
    Serial.println("Saida laser: Erro");
  }

  Serial.println("Laser module test completed.");
}

// Function to test the temperature module
void testTemperatureModule() {
  Serial.println("Testing temperature module...");

  if (validateTemperature()){
    Serial.println("Sensor temperatura: OK");
  } else {
    Serial.println("Sensor temperatura: Erro");
  }

  Serial.println("Temperature module test completed.");
}

// ---------------

void setup() { 
  Serial.begin(9600);
} 

// ---------------

void loop() {
  if (Serial.available() > 0 && !commandReceived) {
    command = Serial.parseInt(); // Read the command from serial
    commandReceived = true; // Set the flag to indicate that a command has been received
  }

  if (commandReceived) {
    switch (command) {
      case 1:
        testFlowModule();
        break;
      case 2:
        testLevelModule();
        break;
      case 3:
        testDoorModule();
        break;
      case 4:
        testPumpModule();
        break;
      case 5:
        testLaserModule();
        break;
      case 6:
        testTemperatureModule();
        break;
      case 7:
        runAllTests();
        break;
      default:
        Serial.println("Coloque um numero de 1 a 7");
        break;
    }

    commandReceived = false; // Reset the flag to allow receiving new commands
  }

  delay(1000); // Wait for 1 second before looping
}
