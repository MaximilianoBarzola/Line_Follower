#include <Arduino.h>
#include <PIDController.hpp>
#define UMBRAL 700
#define BOTTOM1 4
#define BOTTOM2 12
#define PINA1 3
#define PINB1 9
#define PINA2 10
#define PINB2 11
#define REGLA 0

class Motor {
  private:
    int pinA;
    int pinB;
    

  public:
    Motor(int _pinA, int _pinB)
      : pinA(_pinA), pinB(_pinB){}

    void begin() {
      pinMode(pinA, OUTPUT);
      pinMode(pinB, OUTPUT);
    }
    void invert(){
      int tem = pinA;
      pinA = pinB;
      pinB = tem;
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





const int sensores[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; 
int valor_blanco[8];
int valor_negro[8];
int valor_umbrales[8];
bool valor_binario[8];
int pos = 0;
int sumap = 0;
int suma = 0;
int poslast;

// PID
float kp = 0.1;
float ki = 0.0;
float kd = 0.0;

int setpoint = 350; // Centro ideal del borde
int correccion = 0;
int baseSpeed = 50; // Velocidad base de los motores

PID::PIDParameters<double> parameters(kp, ki, kd);
PID::PIDController<double> pidController(parameters);

Motor M1 = Motor(PINA1, PINB1);
Motor M2 = Motor(PINA2, PINB2);

void setup() {
  Serial.begin(9600);
  M1.begin();
  M2.begin();
  pinMode(BOTTOM1, INPUT_PULLUP);
  pinMode(BOTTOM2, INPUT_PULLUP);
  pidController.Input = pos;
  pidController.Setpoint = setpoint;
  //M2.invert();

  pidController.TurnOn();
  while(digitalRead(BOTTOM1));
  for(int x = 0; x < 8; x++){
      valor_blanco[x] = analogRead(sensores[x]);
  }
  delay(10);
  while(!digitalRead(BOTTOM1));
  delay(10);
  while(digitalRead(BOTTOM1));
  for(int x = 0; x < 8; x++){
    valor_negro[x] = analogRead(sensores[x]);
  }
  delay(10);
  for(int x = 0; x < 8; x++){
    valor_umbrales[x] = (valor_blanco[x] + valor_negro[x])/2;
  }
  while(!digitalRead(BOTTOM1));
  delay(10);
  while(digitalRead(BOTTOM1));
  Serial.println("Hola");
  //M1.setSpeed(255);
  //M2.setSpeed(255);
}

void loop() {
  for(int x = 0; x < 8; x++){
    int valor = analogRead(sensores[x]);
    valor_binario[x] = valor > valor_umbrales[x] ? REGLA : !REGLA;
    int peso = x * 100;
    sumap += valor_binario[x] * peso;
    suma += valor_binario[x];
    pos=(sumap/suma);
    
  }
  if(poslast<=100 && pos==-1){
    pos=0;
  }
  if(poslast>=700 && pos==-1){
    pos=700;
  }
  poslast=pos;
  pidController.Input = pos;
  pidController.Update();
  
  correccion = pidController.Output;
  
  int velocidadIzquierda = baseSpeed + correccion;
  int velocidadDerecha  = baseSpeed - correccion;
  
  // Limitar velocidad para evitar sobrepaso de PWM
  velocidadIzquierda = constrain(velocidadIzquierda, -255, 255);
  velocidadDerecha  = constrain(velocidadDerecha, -255, 255);
  
  M2.setSpeed(velocidadIzquierda);
  M1.setSpeed(velocidadDerecha);
  
  sumap = 0;
  suma = 0;
  Serial.print(pos);
  Serial.print(" ");
  Serial.println(pidController.Output);



/*
for(int i = 0; i < 8; i++){
  Serial.print(valor_umbrales[i]);
  Serial.print(" ");
}
Serial.println(pos);
pos = 0;
Serial.println("");
*/ 

}
