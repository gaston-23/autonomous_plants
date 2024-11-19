
// Libraries
//////////////////////////////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
// #include <ArduinoIoTCloud.h>

// #include <WiFi.h>
// #include "ESPAsyncWebServer.h"
/////////////////////////////////////////////////

#include "thingProperties.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN        8
#define NUMPIXELS 16

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500
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
#define LDR_PIN 5
#define HUM_GND_PIN 2
#define PUMP_PIN 3



/////////////////////////////////////////////////////////

unsigned long lastTime = 0;  
unsigned long timerDelay = 30000;  // send readings timer
int inc = 0;
int status = 1;
// int ml = 0;
long lastPumpTime = 0;


void calcLDR() {
  // float ldr_raw = analogRead(LDR_PIN);  // Leemos el valor del pinLDR y lo guardamos en la variable creada.
  lDR = analogRead(LDR_PIN) / 40.96;  // Leemos el valor del pinLDR y lo guardamos en la variable creada.
  // lDR = ldr_raw * (5 /1023);
  
  // Serial.print("LDR: ");
  // Serial.println(lDR);
}

void calcHumGround() {
  hG = analogRead(HUM_GND_PIN) / 40.96;  // Leemos el valor del pinLDR y lo guardamos en la variable creada.
  // Serial.print("Hum suelo: ");
  // Serial.println(hG);
}

/**
 * Get lectures from BME680 sensor
 */
void getBME680Readings(){
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    // Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    // Serial.println(F("Failed to complete reading :("));
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

void onMlChange(){
  if(ml > 0) {
    pumpMilliMeters(ml);
    ml = 0;
    lastPumpTime = millis();
  }

}

void wifiConectado(){
  for(int i=0; i<3; i++) {
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();
    delay(DELAYVAL);
  }
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
}

void errorConexion(){
  for(int i=0; i<3; i++) {
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
    pixels.show();
    delay(DELAYVAL);
  }
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
}

OnNetworkEventCallback wifiConectadoCallback = wifiConectado;
OnNetworkEventCallback errorConexionCallback = errorConexion;

void setup() {
  // Initialize serial and wait for port to open:
  // Serial.begin(115200);
  pixels.begin();
  pixels.clear();

  // pins initialization
  pinMode(LDR_PIN, INPUT);
  pinMode(HUM_GND_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  minHumidity = 50;
  lastPumpTime = millis();
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  // ArduinoCloud.getConnection()->addConnectCallback(wifiConectadoCallback);
  // ArduinoCloud.getConnection()->addDisconnectCallback(errorConexionCallback);
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
    // Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif


}

void loop() {
  
  ArduinoCloud.update();
  // for(int i=0; i<NUMPIXELS; i++) {

  //   pixels.setPixelColor(i, pixels.Color(150, 0, 0));
  //   pixels.show();
    // Serial.print("------------------> ");
    // Serial.println(i);
  //   delay(DELAYVAL);
  // }
  //get Readings
  calcLDR();
  calcHumGround();
  if ((millis() - lastTime) > timerDelay) {
      getBME680Readings();
      // Serial.printf("Temperature = %.2f ºC \n", temperature);
      // Serial.printf("Humidity = %.2f % \n", humidity);
      // Serial.printf("Pressure = %.2f hPa \n", pressure);
      // Serial.printf("Gas Resistance = %.2f KOhm \n", gasResistance);
      // Serial.println();
    lastTime = millis();
  }

  if(hG < minHumidity && lastPumpTime + 1000*5 < millis()) { // 1 hora de restraso para evitar que se active muy seguido el riego
    int quantityInMl = (minHumidity - hG);
    // Serial.print("Valor: ");
    // Serial.println(quantityInMl);
    // manejoBomba(true);
    // delay(1000);
    // manejoBomba(false);
    pumpMilliMeters(quantityInMl);
    lastPumpTime = millis();
  } 

  if (Serial.available() > 0) {
    // read the incoming byte:
    status = Serial.read() -'0';

    // say what you got:
    // Serial.print("I received: ");
    // Serial.println(status);
    if (status == 0) {
      manejoBomba(false);
    }
    if (status == 1) {
      pumpMilliMeters(100);
      lastPumpTime = millis();
    }
  }

  
  delay(3000);
}


// void waterLevel() {
//   float x = analogRead(LDR_PIN);
  // Serial.print("agua: ");
  // Serial.println(x);
// }

// void calcTempLM35() {
//   lDR = analogRead(LDR_PIN);  // Leemos el valor del pinLDR y lo guardamos en la variable creada.
//   // lDR++;
  // Serial.print("lDR: ");
  // Serial.println(lDR);
//   //Obtenemos la temperatura con la siguiente formula:
//   temp = (lDR * (500.0 / 1023.0));
//   //Imprimimos por monitor serie la temperatura en celcius
  // Serial.print("temp: ");
  // Serial.println(temp);
// }

