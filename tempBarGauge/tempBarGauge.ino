/*************************************************** 
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED 24 Bargraph Backpack
  ----> http://www.adafruit.com/products/721

  These displays use I2C to communicate, 2 pins are required to 
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>
#include "Adafruit_LEDBackpack.h"

//Bar gauge setup
int numBars = 24;
Adafruit_24bargraph bar = Adafruit_24bargraph();

//calibration knobs setup
int scalePot = A0;
int shiftPot = A1;
double midTemp = 23.0;
double tempRange = 3.0;

//RTD setup
/*// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0!
#define RREF 430.0
*/

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 max = Adafruit_MAX31856(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 max = Adafruit_MAX31856(10);



void setup() {
  Serial.begin(115200);

   analogReference(EXTERNAL);
   pinMode(8,OUTPUT);

  //init temp sensor
  //max.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  max.begin();
  max.setThermocoupleType(MAX31856_TCTYPE_K);
  max.setTempFaultThreshholds(0.0, 3000.0);
  
  //init bar
  bar.begin(0x70);  // pass in the address
  for (uint8_t b=0; b<numBars; b++ ){
    if ((b % 3) == 0)  bar.setBar(b, LED_RED);
    if ((b % 3) == 1)  bar.setBar(b, LED_YELLOW);
    if ((b % 3) == 2)  bar.setBar(b, LED_GREEN);
  }
  bar.writeDisplay();
  delay(2000);
}


void loop() {
  //read temperature

  double tempC = 0.0;
  //double tempC = readTemperatureRTD();
  if(digitalRead(6))
    tempC = readTemperatureTC();
  else
    tempC = readTempThermistor();

  Serial.print("Temperature: "); Serial.println(tempC);

  //read calibration knobs
  tempRange = readTempRange();
  midTemp = readMidTemp();
  Serial.print("tempRange: "); Serial.println(tempRange);
  Serial.print("midTemp: "); Serial.println(midTemp);
  
 double degPerBar = tempRange/numBars;
 double startTemp = midTemp-0.5*tempRange;
 for (uint8_t b=0; b<numBars; b++) {
  if(startTemp+b*degPerBar<=tempC)
  {
    bar.setBar(b, LED_RED);
  }
  else
  {
    bar.setBar(b, LED_OFF); 
  }
 }
 bar.writeDisplay();
 delay(50);
}

//double readTemperatureRTD()
//{
//  uint16_t rtd = max.readRTD();
//  float ratio = rtd;
//  ratio /= 32768;
//  return max.temperature(100, RREF);
//}
double readTemperatureTC()
{
  // Check and print any faults
  uint8_t fault = max.readFault();
  if (fault) {
    if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
    if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
    if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
    if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
    if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
    if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
    if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
    if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
  }
  return max.readThermocoupleTemperature();
}
double readTempThermistor()
{
  double reading = 0.0;
  int nMeasurements = 5;
  int measureAuxPin = 8;
  digitalWrite(measureAuxPin,HIGH);
  for(int i=0;i<nMeasurements;++i)
  {
    reading += analogRead(A2);
    delay(5);
  }
  digitalWrite(measureAuxPin,LOW);
  reading = reading/nMeasurements; 
  return voltageToTemp(reading);
}
double voltageToTemp(double analogVoltageReading)
{
  // resistance at 25 degrees C
  const int THERMISTORNOMINAL = 10000;      
  // temp. for nominal resistance (almost always 25 C)
  const int TEMPERATURENOMINAL = 25;  
  // The beta coefficient of the thermistor (usually 3000-4000)
  const int BCOEFFICIENT = 3950;
  // the value of the 'other' resistor
  const int SERIESRESISTOR = 10000;  
  
  // convert the value to resistance
  double resistance = 1023.0 / analogVoltageReading  - 1;
  resistance = SERIESRESISTOR / resistance;
  Serial.print("Thermistor resistance "); 
  Serial.println(resistance);
 
  double steinhart;
  steinhart = resistance / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
  Serial.print("Temperature "); 
  Serial.print(steinhart);
  Serial.println(" *C");

  return steinhart;
}
double readTempRange()
{
  return mapf(analogRead(A0), 0, 1023, 0.5, 8);
}
double readMidTemp()
{
  return mapf(analogRead(A1), 0, 1023, 15, 40);
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

