
// Libraries
//////////////////////////////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ArduinoIoTCloud.h>

// #include <WiFi.h>
// #include "ESPAsyncWebServer.h"
/////////////////////////////////////////////////

#include "thingProperties.h"

// Variables

// BMP680
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme(&Wire); // I2C

// PINES
#define SDA_PIN 1 // Pin que usarás para SDA
#define SCL_PIN 10 // Pin que usarás para SCL
#define LDR_PIN 3
#define HUM_GND_PIN 2
#define PUMP_PIN 10


float temperature;
float humidity;
float pressure;
float altitude;
float gasResistance;

/////////////////////////////////////////////////////////

unsigned long lastTime = 0;  
unsigned long timerDelay = 30000;  // send readings timer
int inc = 0;
int status = 1;



void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);

  // pins initialization
  pinMode(LDR_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  // setDebugMessageLevel(2);
  // ArduinoCloud.printDebugInfo();

  // Init BME680 sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms


}

void loop() {
  
  ArduinoCloud.update();
  //get Readings
  calcLDR();
  calcHumGround();
  if ((millis() - lastTime) > timerDelay) {
      getBME680Readings();
      Serial.printf("Temperature = %.2f ºC \n", temperature);
      Serial.printf("Humidity = %.2f % \n", humidity);
      Serial.printf("Pressure = %.2f hPa \n", pressure);
      Serial.printf("Gas Resistance = %.2f KOhm \n", gasResistance);
      Serial.println();
    lastTime = millis();
  }

  if (Serial.available() > 0) {
    // read the incoming byte:
    status = Serial.read() -'0';

    // say what you got:
    Serial.print("I received: ");
    Serial.println(status);
    if (status == 0) {
      manejoBomba(false);
    }
    if (status == 1) {
      pumpMilliMeters(100);
    }
  }

  
  // delay(2000);
}

void calcLDR() {
  lDR = analogRead(LDR_PIN);  // Leemos el valor del pinLDR y lo guardamos en la variable creada.
  // lDR++;
  Serial.println(lDR);
}

void calcHumGround() {
  lDR = analogRead(LDR_PIN);  // Leemos el valor del pinLDR y lo guardamos en la variable creada.
  // lDR++;
  Serial.println(lDR);
}

// void waterLevel() {
//   float x = analogRead(LDR_PIN);
//   Serial.print("agua: ");
//   Serial.println(x);
// }

// void calcTempLM35() {
//   lDR = analogRead(LDR_PIN);  // Leemos el valor del pinLDR y lo guardamos en la variable creada.
//   // lDR++;
//   Serial.print("lDR: ");
//   Serial.println(lDR);
//   //Obtenemos la temperatura con la siguiente formula:
//   temp = (lDR * (500.0 / 1023.0));
//   //Imprimimos por monitor serie la temperatura en celcius
//   Serial.print("temp: ");
//   Serial.println(temp);
// }

/**
 * Get lectures from BME680 sensor
 */
void getBME680Readings(){
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  temperature = bme.temperature;
  pressure = bme.pressure / 100.0;
  humidity = bme.humidity;
  gasResistance = bme.gas_resistance / 1000.0;
}

void manejoBomba(bool status) {
  if (status) {
    digitalWrite(PUMP_PIN, HIGH);
  } else {
    digitalWrite(PUMP_PIN, LOW);
  }
}

void pumpMilliMeters(int mm) {
  digitalWrite(PUMP_PIN, HIGH);
  float timeToFinish = mm/33.3*1000;
  delay(timeToFinish);
  digitalWrite(PUMP_PIN, LOW);
}

