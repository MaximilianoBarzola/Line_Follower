#include <Arduino.h>
#define UMBRAL 700
#define BOTTOM 12

const int sensores[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; 
int valor_blanco[8];
int valor_negro[8];
int valor_umbrales[8];
bool valor_binario[8];
int pos = 0;



void setup() {
  Serial.begin(9600);
  pinMode(BOTTOM, INPUT_PULLUP);
  while(digitalRead(BOTTOM));
  for(int x = 0; x < 8; x++){
      valor_blanco[x] = analogRead(sensores[x]);
  }
  delay(10);
  while(!digitalRead(BOTTOM));
  delay(10);
  while(digitalRead(BOTTOM));
  for(int x = 0; x < 8; x++){
    valor_negro[x] = analogRead(sensores[x]);
  }
  delay(10);
  for(int x = 0; x < 8; x++){
    valor_umbrales[x] = (valor_blanco[x] + valor_negro[x])/2;
  }
}

void loop() {
  for(int x = 0; x < 8; x++){
    int valor = analogRead(sensores[x]);
    if(valor > valor_umbrales[x]){
      valor_binario[x] = 1;
    }
    else{
      valor_binario[x] = 0;
    }
  }
  for(int y = 0; y < 8; y++){
    pos = pos + valor_binario[y] * 100;
  }
 
  for(int i = 0; i < 8; i++){
    Serial.print(valor_umbrales[i]);
    Serial.print(" ");
  }
  Serial.println(pos);
  pos = 0;
  Serial.println("");
  delay(100);
  /*
  Serial.print(analogRead(A0));
  Serial.print(" ");
  Serial.print(analogRead(A1));
  Serial.print(" ");
  Serial.print(analogRead(A2));
  Serial.print(" ");
  Serial.print(analogRead(A3));
  Serial.print(" ");
  Serial.print(analogRead(A4));
  Serial.print(" ");
  Serial.print(analogRead(A5));
  Serial.print(" ");
  Serial.print(analogRead(A6));
  Serial.print(" ");
  Serial.println(analogRead(A7));
  delay(100);
  */
}
