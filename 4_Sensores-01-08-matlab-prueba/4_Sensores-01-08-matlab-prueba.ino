#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_Si7021.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>

/////SHT31//////////
Adafruit_SHT31 sht31 = Adafruit_SHT31();
//////DHT21//////////////////////////
Adafruit_Si7021 sensor = Adafruit_Si7021();
//////BME280//////////////////////////
Adafruit_BME280 bme; // I2C
///////INA-219//////////////////////////
Adafruit_INA219 ina219;
//////////////////////////////////////////

long tiempo=0;

void setup() {
  Serial.begin(115200);
  //////SHT31////////
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  /////SHT21/////////////////
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }
  /////BME280////////////////////
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  //////INA-219//////////////////
  ina219.begin();
  ina219.setCalibration_16V_400mA();
  pinMode(7,INPUT_PULLUP);
}

void loop() {
  ////SHT31/////////////////////
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  Serial.print(t,3); Serial.print("\t");
  Serial.print(h,3); Serial.print("\t");

  ////////SHT21/////////////////////////////
  float t1=sensor.readTemperature();
  float h1=sensor.readHumidity();
  
  Serial.print(t1,3); Serial.print("\t");
  Serial.print(h1,3);Serial.print("\t");
  
  /////////BME280//////////////////////////////
  float t2=bme.readTemperature();
  float h2=bme.readHumidity();
  float p2=bme.readPressure();///1000.0F;
  
  Serial.print(t2,3);Serial.print("\t");
  Serial.print(h2,3);Serial.print("\t");
  Serial.print(p2,3);Serial.print("\t");
  /////////INA-219////////////////////////////
  float busvoltage = ina219.getBusVoltage_V();
  float presion = (busvoltage - 2.5) * (1000);
  Serial.print(busvoltage,3);Serial.print("\t"); 
  Serial.print(presion,3);Serial.print("\t"); 
  //////////Calculo velocidad////////////////
  float velocidad = sqrt((2*abs(presion))/1.183);
  Serial.print(velocidad,3);Serial.print("\t");
  ///////////////////////////////////////////
  float velocidad1 = velocidad*100;
  float velocidad2 = velocidad1;

  //////////////////////////////////////////
  //
  tiempo=millis();
  Serial.print(tiempo);Serial.print("\t");
  boolean paro = digitalRead(7);
  Serial.println(paro);
  delay(1000);
}
