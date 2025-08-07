#include <Arduino.h>
#include <EEPROM.h>

// === CONFIGURACIÓN ===
#define REGLA 0
//#define DEBUG           // ← Comentá esta línea para desactivar logs por Serial

// === PINES ===
#define BOTTOM1 12      // ↓ Disminuye kp
#define BOTTOM2 4       // ↑ Aumenta kp
#define PINA1 3
#define PINB1 9
#define PINA2 10
#define PINB2 11
#define LED_INICIO 6
#define LED_CALIBRACION 7

// === EEPROM ===
#define EEPROM_ADDR_KP 0 // Dirección de EEPROM para guardar kp

// === BOTONES ===
unsigned long debounceDelay = 100;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
bool lastState1 = HIGH;
bool lastState2 = HIGH;

// === SENSORES ===
const int sensores[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int valor_blanco[8];
int valor_negro[8];
int valor_umbrales[8];
bool valor_binario[8];

// === POSICIÓN ===
int pos = 0;
int poslast = 0;
int sumap = 0;
int suma = 0;

// === PID ===
float kp = 0.1;
float ki = 0.0;
float kd = 2.0;
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float setpoint = 350;
int correccion = 0;
int baseSpeed = 200;

// === MOTOR ===
class Motor {
  private:
    int pinA;
    int pinB;

  public:
    Motor(int _pinA, int _pinB) : pinA(_pinA), pinB(_pinB) {}

    void begin() {
      pinMode(pinA, OUTPUT);
      pinMode(pinB, OUTPUT);
    }

    void setSpeed(int speed) {
      speed = constrain(speed, -255, 255);
      if (speed > 0) {
        analogWrite(pinA, speed);
        analogWrite(pinB, 0);
      } else if (speed < 0) {
        analogWrite(pinA, 0);
        analogWrite(pinB, -speed);
      } else {
        analogWrite(pinA, 0);
        analogWrite(pinB, 0);
      }
    }
};

Motor M1 = Motor(PINA1, PINB1);
Motor M2 = Motor(PINA2, PINB2);

// === PID ===
int calcularPID(int lectura) {
  error = setpoint - lectura;
  integral += error;
  derivative = error - lastError;
  lastError = error;
  return kp * error + ki * integral + kd * derivative;
}

// === CALIBRACIÓN DE SENSORES ===
void calibrarSensores() {
  digitalWrite(LED_CALIBRACION, HIGH);
  while (digitalRead(BOTTOM1));
  delay(10);
  for (int x = 0; x < 8; x++) {
    valor_blanco[x] = analogRead(sensores[x]);
  }

  delay(200);
  while (!digitalRead(BOTTOM1));
  delay(10);
  while (digitalRead(BOTTOM1));

  for (int x = 0; x < 8; x++) {
    valor_negro[x] = analogRead(sensores[x]);
  }

  for (int x = 0; x < 8; x++) {
    valor_umbrales[x] = (valor_blanco[x] + valor_negro[x]) / 2;
  }

  delay(200);
  digitalWrite(LED_CALIBRACION, LOW);
  while (!digitalRead(BOTTOM1));
  delay(10);
  while (digitalRead(BOTTOM1));
}

// === SETUP ===
void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  pinMode(BOTTOM1, INPUT_PULLUP);
  pinMode(BOTTOM2, INPUT_PULLUP);
  pinMode(LED_INICIO, OUTPUT);
  pinMode(LED_CALIBRACION, OUTPUT);

  M1.begin();
  M2.begin();
  // Leer kp desde EEPROM
  EEPROM.get(EEPROM_ADDR_KP, kp);
  if (isnan(kp) || kp < 0.05 || kp > 5.0) {
    kp = 0.1;
  }

  calibrarSensores();

  digitalWrite(LED_INICIO, HIGH);

  #ifdef DEBUG
  Serial.print("Sistema iniciado. kp = ");
  Serial.println(kp);
  #endif
}

// === LOOP ===
void loop() {
  // Leer sensores
  sumap = 0;
  suma = 0;
  for (int x = 0; x < 8; x++) {
    int valor = analogRead(sensores[x]);
    valor_binario[x] = valor > valor_umbrales[x] ? REGLA : !REGLA;
    int peso = x * 100;
    sumap += valor_binario[x] * peso;
    suma += valor_binario[x];
  }

  // Calcular posición
  if (suma > 0) {
    pos = sumap / suma;
  } else {
    pos = -1;
  }

  if (pos == -1) {
    if (poslast <= 100) pos = 0;
    else if (poslast >= 700) pos = 700;
    else {
      pos = 350;
      integral = 0;
      lastError = 0;
    }
  }
  poslast = pos;

  // PID
  correccion = calcularPID(pos);

  // Control de motores
  int velocidadIzquierda = baseSpeed + correccion;
  int velocidadDerecha  = baseSpeed - correccion;

  M2.setSpeed(constrain(velocidadIzquierda, -255, 255));
  M1.setSpeed(constrain(velocidadDerecha, -255, 255));

  // === BOTÓN BOTTOM2: Aumentar kp ===
  bool currentState2 = digitalRead(BOTTOM2);
  if (currentState2 == LOW && lastState2 == HIGH && millis() - lastDebounceTime2 > debounceDelay) {
    kp += 0.05;
    EEPROM.put(EEPROM_ADDR_KP, kp);
    lastDebounceTime2 = millis();
    #ifdef DEBUG
    Serial.print("Aumentó kp: ");
    Serial.println(kp);
    #endif
  }
  lastState2 = currentState2;

  // === BOTÓN BOTTOM1: Disminuir kp ===
  bool currentState1 = digitalRead(BOTTOM1);
  if (currentState1 == LOW && lastState1 == HIGH && millis() - lastDebounceTime1 > debounceDelay) {
    kp = max(0.05, kp - 0.05);
    EEPROM.put(EEPROM_ADDR_KP, kp);
    lastDebounceTime1 = millis();
    #ifdef DEBUG
    Serial.print("Disminuyó kp: ");
    Serial.println(kp);
    #endif
  }
  lastState1 = currentState1;

  // Debug
  #ifdef DEBUG
  Serial.print("pos: "); Serial.print(pos);
  Serial.print(" | error: "); Serial.print(error);
  Serial.print(" | salida: "); Serial.print(correccion);
  Serial.print(" | kp: "); Serial.println(kp);
  #endif

  delay(10);
}
