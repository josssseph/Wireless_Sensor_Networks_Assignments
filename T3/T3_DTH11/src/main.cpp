#include <Arduino.h>

// Define el pin GPIO32 como el pin de entrada del sensor
int sensorPin = 32; 

int sensorValue = 0;

void setup() {
  // Comunicación serial a 115200 baudios
  Serial.begin(115200); 
}

void loop() {
  // Lee el valor analógico del pin (0 a 4095 en ESP32, por defecto)
  sensorValue = analogRead(sensorPin); 

  Serial.println(sensorValue); 
  
  // Espera 1000 milisegundos antes de la siguiente lectura
  delay(1000); 
}