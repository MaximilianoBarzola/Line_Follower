#include <Arduino.h>
#include <EEPROM.h>
#include <avr/interrupt.h>

// ================== CONFIGURACIÓN GENERAL ==================
#define NUM_CH 8              // Usar 6 para Arduino UNO (solo A0..A5)
#define REGLA 1               // 1 = línea blanca, 0 = línea negra (ajústalo a tu regla)

//#define DEBUG               // ← descomenta para logs por Serial
#define USE_FAULT_PROTECT     // Protección por nFAULT del DRV8833

// ================== PINES ==================
#define BOTTOM1 12     // ↓ Disminuye kp
#define BOTTOM2 4      // ↑ Aumenta kp
#define PINA1 3
#define PINB1 9
#define PINA2 10
#define PINB2 11
#define LED_INICIO 6
#define LED_CALIBRACION 7

#ifdef USE_FAULT_PROTECT
#define FAULT_PIN 2     // nFAULT del DRV8833 (activo en LOW)
#endif

// ================== EEPROM ==================
#define EEPROM_ADDR_KP 10 // Dirección para guardar kp

// ================== BOTONES / TIMING ==================
unsigned long debounceDelay = 100;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;

// ================== SENSORES ==================
const int sensores_hw[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; // referencia; el ADC lee por registros
int valor_blanco[8];
int valor_negro[8];
int valor_umbrales[8];
bool valor_binario[8];

volatile uint8_t adc_vals[NUM_CH];   // resultados 8-bit (0..255)
volatile uint8_t canal_actual = 0;
volatile uint8_t descartar = 1;      // descartar 1 conversión tras cambio
volatile bool pidTick = false;       // true al terminar un barrido completo

#ifdef USE_FAULT_PROTECT
volatile bool faultActiveISR = false;
#endif

// ================== POSICIÓN ==================
int pos = 0;
int poslast = 0;
int sumap = 0;
int suma = 0;

// ================== PID (Q8.8 entero) ==================
#define Q   256      // 1.0 = 256 (formato Q8.8)
int16_t KP_Q = (int16_t)(0.70f * Q); // kp inicial en Q
int16_t KI_Q = (int16_t)(0.00f * Q); // ki inicial (por segundo) en Q
int16_t KD_Q = (int16_t)(0.10f * Q); // kd inicial (por segundo) en Q

// Para UI con botones mantenemos floats y convertimos a Q cuando cambian
float kp = 0.70f;
float ki = 0.00f;
float kd = 0.10f;

static int32_t integ_q = 0;          // integral acumulada (Q8.8)
static int16_t prevMeas = 350;       // última medida de posición (entero)
static int32_t d_filt = 0;           // derivada filtrada (Q8.8)

const int16_t OUT_MAX = 255;
const int16_t OUT_MIN = -255;

// Filtro derivada: y = y + α (x - y) ; α en Q8.8 (0<α<1). 0.2 recomendado
#define ALPHA_D_Q  (int16_t)(0.2f * Q)

uint32_t lastPidMicros = 0;          // timestamp del último PID
int setpoint = 350;                  // centro de 0..700 (8 sensores*100)
int correccion = 0;                  // salida del PID
int baseSpeed = 120;                 // velocidad base

// ================== MOTORES ==================
class Motor {
  private:
    int pinA;
    int pinB;

  public:
    Motor(int _pinA, int _pinB) : pinA(_pinA), pinB(_pinB) {}

    void begin() {
      pinMode(pinA, OUTPUT);
      pinMode(pinB, OUTPUT);
      // Frenado activo por defecto
      digitalWrite(pinA, HIGH);
      digitalWrite(pinB, HIGH);
    }

    void setSpeed(int speed) {
      speed = constrain(speed, -255, 255);
      if (speed > 0) {
        // Adelante: 1 + PWM (slow decay)
        digitalWrite(pinA, HIGH);
        analogWrite(pinB, 255 - speed);
      } else if (speed < 0) {
        // Atrás: PWM + 1 (slow decay)
        analogWrite(pinA, 255 + speed); // speed es negativo
        digitalWrite(pinB, HIGH);
      } else {
        // Frenado activo
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
      }
    }
};

Motor M1 = Motor(PINA1, PINB1);
Motor M2 = Motor(PINB2, PINA2);

// ================== ADC (free-running, 8-bit) ==================
void adc_init_freerun_8bit(void) {
  // Referencia = AVcc, ADLAR=1 (8 bits en ADCH), canal inicial = A0
  ADMUX = (1 << REFS0) | (1 << ADLAR);

  // Prescaler = /16 → F_ADC ≈ 1 MHz (8 bits ok)
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2);

  // Auto Trigger Source = Free Running (ADTS=000)
  ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

  // Deshabilitar buffers digitales usados (ruido ↓)
  DIDR0 = 0;
  if (NUM_CH > 0) DIDR0 |= (1 << ADC0D);
  if (NUM_CH > 1) DIDR0 |= (1 << ADC1D);
  if (NUM_CH > 2) DIDR0 |= (1 << ADC2D);
  if (NUM_CH > 3) DIDR0 |= (1 << ADC3D);
  if (NUM_CH > 4) DIDR0 |= (1 << ADC4D);
  if (NUM_CH > 5) DIDR0 |= (1 << ADC5D);
#if defined(ADC6D) && defined(ADC7D)
  if (NUM_CH > 6) DIDR0 |= (1 << ADC6D);
  if (NUM_CH > 7) DIDR0 |= (1 << ADC7D);
#endif

  // Comenzar conversión
  ADCSRA |= (1 << ADSC);

  sei(); // Habilitar interrupciones globales
}

// ISR: una conversión terminó
ISR(ADC_vect) {
  uint8_t v = ADCH; // 8 bits

  if (descartar) {
    descartar = 0;       // descarto sólo la 1ª tras el cambio de canal
    return;
  }

  adc_vals[canal_actual] = v;

  // ¿terminamos el barrido?
  bool endOfScan = (canal_actual == (NUM_CH - 1));

  // Siguiente canal
  canal_actual++;
  if (canal_actual >= NUM_CH) canal_actual = 0;

  // Programar canal y marcar descarte 1 muestra
  ADMUX = (ADMUX & 0xF0) | (canal_actual & 0x0F);
  descartar = 1;

  // Flag para correr el PID en el loop (dt exacto)
  if (endOfScan) {
    pidTick = true;
  }
}

#ifdef USE_FAULT_PROTECT
void faultISR() {
  faultActiveISR = true;
}
#endif

// ================== PID: utilidades ==================
static inline void updateGainsQ() {
  KP_Q = (int16_t)(kp * Q);
  KI_Q = (int16_t)(ki * Q);
  KD_Q = (int16_t)(kd * Q);
}

// PID en Q8.8, con dt (Ts_q) en Q8.8 (segundos)
int pid_update_q8_dt(int16_t setp, int16_t meas, int16_t Ts_q) {
  // error (entero)
  int16_t e = setp - meas;

  // Integral: integ_q += ki*e*Ts
  int32_t e_q = (int32_t)e * Q;             // e en Q
  int32_t kiTs_q = ((int32_t)KI_Q * Ts_q) / Q; // KI*Ts en Q
  integ_q += (kiTs_q * e_q) / Q;

  // Anti-windup
  int32_t integ_max = (int32_t)OUT_MAX * Q;
  int32_t integ_min = (int32_t)OUT_MIN * Q;
  if (integ_q > integ_max) integ_q = integ_max;
  if (integ_q < integ_min) integ_q = integ_min;

  // Derivada sobre la medida (mejor anti-ruido)
  int16_t d_meas = meas - prevMeas;
  prevMeas = meas;

  // d_q ≈ -(d_meas/Ts)*Q  → -(d_meas * Q * Q) / Ts_q
  if (Ts_q < 1) Ts_q = 1; // evitar /0 si dt muy chico
  int32_t d_q = -((int32_t)d_meas * Q * Q) / Ts_q;

  // Filtro de derivada (1er orden)
  d_filt += ((int32_t)ALPHA_D_Q * (d_q - d_filt)) / Q;

  // P y D
  int32_t p_q = ((int32_t)KP_Q * e_q) / Q;
  int32_t dterm_q = ((int32_t)KD_Q * d_filt) / Q;

  // Salida total
  int32_t u_q = p_q + integ_q + dterm_q;
  int32_t u = u_q / Q;

  if (u > OUT_MAX) u = OUT_MAX;
  if (u < OUT_MIN) u = OUT_MIN;

  return (int)u;
}

// ================== CALIBRACIÓN ==================
void calibrarSensores() {
  digitalWrite(LED_CALIBRACION, HIGH);

  // Esperar pulsación para iniciar blanco
  while (digitalRead(BOTTOM1));
  delay(10);

  for (int x = 0; x < NUM_CH; x++) {
    int acc = 0;
    for (int i = 0; i < 12; i++) {  // un poco más de muestras
      acc += adc_vals[x];
      delay(2);
    }
    valor_blanco[x] = acc / 12;
  }

  delay(200);
  // Esperar liberar y volver a pulsar para negro
  while (!digitalRead(BOTTOM1));
  delay(10);
  while (digitalRead(BOTTOM1));

  for (int x = 0; x < NUM_CH; x++) {
    int acc = 0;
    for (int i = 0; i < 12; i++) {
      acc += adc_vals[x];
      delay(2);
    }
    valor_negro[x] = acc / 12;
  }

  for (int x = 0; x < NUM_CH; x++) {
    valor_umbrales[x] = (valor_blanco[x] + valor_negro[x]) / 2;
  }

  digitalWrite(LED_CALIBRACION, LOW);

  // Esperar soltar
  while (!digitalRead(BOTTOM1));
  delay(10);
  while (digitalRead(BOTTOM1));
}

// ================== SETUP ==================
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  adc_init_freerun_8bit();

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

  // Recuperar kp de EEPROM (solo kp; si querés guarda ki/kd también)
  EEPROM.get(EEPROM_ADDR_KP, kp);
  if (isnan(kp) || kp < 0.05f || kp > 5.0f) {
    kp = 0.70f;
  }
  updateGainsQ(); // sincroniza KP_Q/KI_Q/KD_Q con floats

  calibrarSensores();

  digitalWrite(LED_INICIO, HIGH);

  // Subir frecuencia de PWM a ~980 Hz en Timer1 (pines 9,10) y Timer2 (3,11)
  TCCR1B = (TCCR1B & 0b11111000) | 0x03; // /64 → ~976.6 Hz
  TCCR2B = (TCCR2B & 0b11111000) | 0x04; // /64 → ~976.6 Hz

#ifdef DEBUG
  Serial.println("Timers PWM ~980 Hz configurados");
#endif

  lastPidMicros = micros();
}

// ================== LOOP ==================
void loop() {

#ifdef USE_FAULT_PROTECT
  static bool faultBlinkState = false;
  static unsigned long lastBlinkTime = 0;

  if (faultActiveISR) {
    M1.setSpeed(0);
    M2.setSpeed(0);

    if (millis() - lastBlinkTime >= 200) {
      lastBlinkTime = millis();
      faultBlinkState = !faultBlinkState;
      digitalWrite(LED_INICIO, faultBlinkState);
      digitalWrite(LED_CALIBRACION, faultBlinkState);
    }

    if (digitalRead(FAULT_PIN) == HIGH) {
      faultActiveISR = false;
      digitalWrite(LED_INICIO, HIGH);
      digitalWrite(LED_CALIBRACION, LOW);
    } else {
      return; // mantener parada mientras la falla siga activa
    }
  }
#endif

  // --- Lectura de sensores y cálculo de posición ---
  sumap = 0;
  suma = 0;
  for (int x = 0; x < NUM_CH; x++) {
    valor_binario[x] = (adc_vals[x] > valor_umbrales[x]) ? REGLA : !REGLA;
    sumap += valor_binario[x] * (x * 100);
    suma  += valor_binario[x];
  }

  if (suma == 0) {
    // Sin detección: usa última pista para no dividir por 0
    pos = (poslast <= 200) ? 0 : ((poslast >= 500) ? 700 : 350);
  } else {
    pos = sumap / suma;
  }
  poslast = pos;

  // --- PID: 1 vez por barrido completo ---
if (pidTick) {
  pidTick = false;

  // dt real desde el último PID (en Q8.8)
  uint32_t now = micros();
  uint32_t dt_us = now - lastPidMicros;
  lastPidMicros = now;

  // Ts_q = dt[s] * Q = (dt_us * Q) / 1e6
  uint32_t Ts_calc = (dt_us * (uint32_t)Q) / 1000000UL;
  if (Ts_calc < 1) Ts_calc = 1;   // asegura mínimo de 1
  int16_t Ts_q = (int16_t)Ts_calc;

  // Ejecutar PID
  correccion = pid_update_q8_dt((int16_t)setpoint, (int16_t)pos, Ts_q);
}
  // --- Control de motores ---
  int velocidadIzquierda = baseSpeed + correccion;
  int velocidadDerecha   = baseSpeed - correccion;
  M2.setSpeed(constrain(velocidadIzquierda, -255, 255));
  M1.setSpeed(constrain(velocidadDerecha,  -255, 255));

  // --- Botón BOTTOM2: aumenta kp ---
  bool s2 = digitalRead(BOTTOM2);
  static bool last2 = HIGH;
  if (s2 == LOW && last2 == HIGH && (millis() - lastDebounceTime2 > debounceDelay)) {
    kp += 0.05f;
    kp = min(kp, 5.0f);
    EEPROM.put(EEPROM_ADDR_KP, kp);
    updateGainsQ();
    lastDebounceTime2 = millis();
#ifdef DEBUG
    Serial.print("kp aumentó: "); Serial.println(kp, 3);
#endif
  }
  last2 = s2;

  // --- Botón BOTTOM1: disminuye kp ---
  bool s1 = digitalRead(BOTTOM1);
  static bool last1 = HIGH;
  if (s1 == LOW && last1 == HIGH && (millis() - lastDebounceTime1 > debounceDelay)) {
    kp -= 0.05f;
    kp = max(kp, 0.05f);
    EEPROM.put(EEPROM_ADDR_KP, kp);
    updateGainsQ();
    lastDebounceTime1 = millis();
#ifdef DEBUG
    Serial.print("kp disminuyó: "); Serial.println(kp, 3);
#endif
  }
  last1 = s1;

#ifdef DEBUG
  static uint32_t tdbg = 0;
  if (millis() - tdbg > 200) {
    tdbg = millis();
    Serial.print("pos="); Serial.print(pos);
    Serial.print("  corr="); Serial.print(correccion);
    Serial.print("  kp="); Serial.print(kp, 2);
    Serial.print("  adc[0..7]=");
    for (int i = 0; i < NUM_CH; i++) { Serial.print(' '); Serial.print(adc_vals[i]); }
    Serial.println();
  }
#endif

  delay(10); // aliviana el loop (no afecta al PID, que va por pidTick)
}



/*#include <Arduino.h>
#include <EEPROM.h>
#include <avr/interrupt.h>

#define NUM_CH 8   // usar 6 para Arduino UNO (solo A0..A5)

// === CONFIGURACIÓN ===
#define REGLA 1
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
#define EEPROM_ADDR_KP 10 // Dirección de EEPROM para guardar kp

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
float kd = 4.0;
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
int baseSpeed = 130;


volatile uint8_t adc_vals[NUM_CH];   // vector de resultados (0..255)
volatile uint8_t canal_actual = 0;
volatile uint8_t descartar = 1;  


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

void adc_init_freerun_8bit(void) {
  // Vref = AVcc, resultado alineado a la izquierda, canal inicial = A0
  ADMUX = (1 << REFS0) | (1 << ADLAR);

  // Prescaler = 16 → ADPS2:0 = 100 → /16
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2);

  // Free Running (ADTS=000)
  ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

  // Deshabilitar buffers digitales (reduce ruido)
  DIDR0 = 0;
  if (NUM_CH > 0) DIDR0 |= (1 << ADC0D);
  if (NUM_CH > 1) DIDR0 |= (1 << ADC1D);
  if (NUM_CH > 2) DIDR0 |= (1 << ADC2D);
  if (NUM_CH > 3) DIDR0 |= (1 << ADC3D);
  if (NUM_CH > 4) DIDR0 |= (1 << ADC4D);
  if (NUM_CH > 5) DIDR0 |= (1 << ADC5D);
#if defined(ADC6D) && defined(ADC7D)
  if (NUM_CH > 6) DIDR0 |= (1 << ADC6D);
  if (NUM_CH > 7) DIDR0 |= (1 << ADC7D);
#endif

  // Iniciar la primera conversión
  ADCSRA |= (1 << ADSC);

  sei(); // habilitar interrupciones globales
}

ISR(ADC_vect) {
  uint8_t valor = ADCH; // lectura de 8 bits

  if (descartar) {
    descartar = 0; // no guardar esta muestra
    return;
  }

  // Guardar valor del canal actual
  adc_vals[canal_actual] = valor;

  // Avanzar al siguiente canal
  canal_actual++;
  if (canal_actual >= NUM_CH) canal_actual = 0;

  // Configurar canal nuevo y descartar próxima conversión
  ADMUX = (ADMUX & 0xF0) | (canal_actual & 0x0F);
  descartar = 1;
}

// === PID ===

float calcularKp(int p){
if (p >= 600 && p <= 100)return kp * 5;
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
      valor_promedio += adc_vals[x]; 
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
      valor_promedio += adc_vals[x]; 
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

  adc_init_freerun_8bit();

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
    valor_binario[x] = adc_vals[x] > valor_umbrales[x] ? REGLA : !REGLA;
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
*/