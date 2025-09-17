#include <Arduino.h>
#include <EEPROM.h>

// === CONFIGURACIÓN ===
#define REGLA 0
//#define DEBUG           // ← Comentá esta línea para desactivar logs por Serial
#define USE_FAULT_PROTECT   // ← Comentá esta línea para desactivar la protección por nFAULT

// === PINES ===
#define BOTTOM1 12      // ↓ Disminuye kp
#define BOTTOM2 4       // ↑ Aumenta kp
#define PINA1 3
#define PINB1 9
#define PINA2 10
#define PINB2 11
#define LED_INICIO 6
#define LED_CALIBRACION 7

#ifdef USE_FAULT_PROTECT
#define FAULT_PIN 2     // Pin conectado a nFAULT del DRV8833
#endif

// === EEPROM ===
#define EEPROM_ADDR_KP 0 // Dirección de EEPROM para guardar kp

// === PID ===
#define TIME 1
// === BOTONES ===
unsigned long debounceDelay = 100;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastTime = 0;
bool lastState1 = HIGH;
bool lastState2 = HIGH;

#ifdef USE_FAULT_PROTECT
volatile bool faultActiveISR = false; // Flag de interrupción
#endif

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
float kp = 0.7;
float ki = 0.1;
float kd = 2.0;
float error = 0;
float error2 = 0; 
float error3 = 0;
float error4 = 0;
float error5 = 0; 
float error6 = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float setpoint = 350;
int correccion = 0;
int baseSpeed = 120;

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
      // Inicializar en modo frenado activo (HIGH-HIGH)
      digitalWrite(pinA, HIGH);
      digitalWrite(pinB, HIGH);
    }

    void setSpeed(int speed) {
      speed = constrain(speed, -255, 255);
      
      if (speed > 0) {
        // Forward PWM, slow decay: 1 (HIGH) + PWM
        digitalWrite(pinA, HIGH);
        analogWrite(pinB, 255 - speed); // Invertimos el PWM para slow decay
      } else if (speed < 0) {
        // Reverse PWM, slow decay: PWM + 1 (HIGH)
        analogWrite(pinA, 255 + speed); // Invertimos el PWM para slow decay (speed es negativo)
        digitalWrite(pinB, HIGH);
      } else {
        // Frenado activo: ambos HIGH
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
      }
    }
};

Motor M1 = Motor(PINA1, PINB1);
Motor M2 = Motor(PINB2, PINA2);

// === PID ===

float calcularKp(int p){
if (p >= 650 && p <= 50)return kp * 5;
else return kp;
}

int calcularPID(int lectura) {
  error = setpoint - lectura;
  integral = error + error2 + error3 + error4 + error5 + error6;
  derivative = error - lastError;
  lastError = error;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error;
  return (calcularKp(lectura) * error + ki * integral + kd * derivative);
}

void girro(int p, int velz, int veld){
  if(p >= 650){
    M2.setSpeed(0);
    M1.setSpeed(200);
  }
  else if(p <= 50){
    M2.setSpeed(200);
    M1.setSpeed(0);
  }
  else{
    M2.setSpeed(constrain(velz, -255, 255));
    M1.setSpeed(constrain(veld, -255, 255));
  }
}


// === CALIBRACIÓN DE SENSORES ===
void calibrarSensores() {
  digitalWrite(LED_CALIBRACION, HIGH);
  while (digitalRead(BOTTOM1));
  delay(10);
  for (int x = 0; x < 8; x++) {
    int valor_promedio = 0;
    for (int i = 0; i < 10; i++){
      valor_promedio += analogRead(sensores[x]); 
    }
    valor_blanco[x] = valor_promedio/10;
  }

  delay(200);
  while (!digitalRead(BOTTOM1));
  delay(10);
  while (digitalRead(BOTTOM1));

  for (int x = 0; x < 8; x++) {
    int valor_promedio = 0;
    for (int i = 0; i < 10; i++){
      valor_promedio += analogRead(sensores[x]); 
    }
    valor_negro[x] = valor_promedio/10;
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

#ifdef USE_FAULT_PROTECT
void faultISR() {
  faultActiveISR = true;
}
#endif

// === SETUP ===
void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  pinMode(BOTTOM1, INPUT_PULLUP);
  pinMode(BOTTOM2, INPUT_PULLUP);
  pinMode(LED_INICIO, OUTPUT);
  pinMode(LED_CALIBRACION, OUTPUT);

  #ifdef USE_FAULT_PROTECT
  pinMode(FAULT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAULT_PIN), faultISR, FALLING);
  #endif

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
    // --- Cambiar Timer1 (pines 9 y 10) a ~980 Hz ---
  TCCR1B = (TCCR1B & 0b11111000) | 0x03; // prescaler = 64 → 16MHz / (64*256) = 976.56 Hz aprox.

  // --- Cambiar Timer2 (pines 3 y 11) a ~980 Hz ---
  TCCR2B = (TCCR2B & 0b11111000) | 0x04; // prescaler = 64 → 16MHz / (64*256) = 976.56 Hz aprox.

  #ifdef DEBUG
  Serial.println("Timers configurados a ~980Hz");
  #endif

  lastTime = millis();
}

// === LOOP ===
void loop() {

  #ifdef USE_FAULT_PROTECT
  static bool faultBlinkState = false;
  static unsigned long lastBlinkTime = 0;

  if (faultActiveISR) {
    // Parar motores
    M1.setSpeed(0);
    M2.setSpeed(0);

    // Parpadeo LEDs cada 200ms
    if (millis() - lastBlinkTime >= 200) {
      lastBlinkTime = millis();
      faultBlinkState = !faultBlinkState;
      digitalWrite(LED_INICIO, faultBlinkState);
      digitalWrite(LED_CALIBRACION, faultBlinkState);
    }

    // Salir del loop normal
    if (digitalRead(FAULT_PIN) == HIGH) {
      // Falla se liberó
      faultActiveISR = false;
      digitalWrite(LED_INICIO, HIGH);
      digitalWrite(LED_CALIBRACION, LOW);
    } else {
      return; // Mantener parada
    }
  }
  #endif

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
  
  pos = sumap / suma;


  if (pos == -1) {
    if (poslast <= 200) pos = 0;
    else if (poslast >= 500) pos = 700;
//    else {
//      pos = 350;
//      integral = 0;
//      lastError = 0;
//    }
  }
  poslast = pos;

  // PID
  if(millis() > TIME + lastTime){
    correccion = calcularPID(pos);
    lastTime = millis();
  }

  // Control de motores
  int velocidadIzquierda = baseSpeed + correccion;
  int velocidadDerecha  = baseSpeed - correccion;
  //girro(pos, velocidadIzquierda, velocidadDerecha);
  
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
