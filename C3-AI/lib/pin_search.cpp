#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 1 // Pin que usarás para SDA
#define SCL_PIN 10 // Pin que usarás para SCL

void setup() {
  Serial.begin(115200); // Inicializa la comunicación serie a 115200 baudios
  Wire.begin(SDA_PIN, SCL_PIN); // Inicializa la comunicación I2C
  
  Serial.println("Escaneando dispositivos I2C...");
}

void loop() {
  byte error, address;
  int count = 0;

  Serial.println("Dispositivos encontrados:");

  // Escanea todas las direcciones I2C de 1 a 127
  for (address = 1; address < 127; address++) {
    // Inicia la transmisión
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    // Si no hay error, significa que hay un dispositivo en esa dirección
    if (error == 0) {
      Serial.print("Dispositivo encontrado en la dirección 0x");
      Serial.println(address, HEX);
      count++;
      delay(500); // Espera medio segundo entre escaneos
    } 
    else if (error == 4) {
      Serial.print("Error en la dirección 0x");
      Serial.println(address, HEX);
    }
  }
  
  if (count == 0) {
    Serial.println("No se encontraron dispositivos I2C.");
  } else {
    Serial.print("Total de dispositivos encontrados: ");
    Serial.println(count);
  }

  delay(5000); // Espera 5 segundos antes de escanear de nuevo
}
