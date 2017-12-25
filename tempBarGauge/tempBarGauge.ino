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

class LongBarGraph
{
  private:
  int m_firsAddr = 0x70;
  int m_nLEDPerBar = 24;
  boolean flipDirection = true;
  int m_nBars;
  
  Adafruit_24bargraph* m_bars;
  
  public:
  LongBarGraph(int nBars)
  {
    m_nBars = nBars;
    m_bars = new Adafruit_24bargraph[m_nBars];
    for(int i = 0; i < nBars; ++i)
    {
      m_bars[i] = Adafruit_24bargraph();  
    }
  }
  //==============================
  init()
  {
    for(int i = 0; i < m_nBars; ++i)
    {
      m_bars[i].begin(m_firsAddr+m_nBars-1-i);
    }
    
  }
  //==============================
  setBar(int b, int c)
  {
    if(flipDirection)
    {
      b = (m_nLEDPerBar*m_nBars)-b-1;
    }
    m_bars[(int)floor(b/m_nLEDPerBar)].setBar(b%m_nLEDPerBar, c);
  }
  //
  writeDisplay()
  {
    for(int i = 0; i < m_nBars; ++i)
    {
      m_bars[i].writeDisplay();
    }
  }
};

//Bar gauge setup
/////original
//int numLEDs = 24;
//Adafruit_24bargraph bar = Adafruit_24bargraph();
////should work
//int numLEDs = 24;
//LongBarGraph bar = LongBarGraph(1)
////double length
int numLEDs = 48;
LongBarGraph bar = LongBarGraph(2);

//calibration knobs setup
int scalePot = A0;
int initButton = 3;
double midTemp = 23.0;
double tempRange = 3.0;

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 max = Adafruit_MAX31856(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 max = Adafruit_MAX31856(10);

void setup() {
  Serial.begin(115200);

  max.begin();
  max.setThermocoupleType(MAX31856_TCTYPE_K);
  max.setTempFaultThreshholds(0.0, 3000.0);
  
  //init bar
  //bar.begin(0x70);  // pass in the address
  bar.init(); 
  for (uint8_t b=0; b<numLEDs; b++ ){
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
  tempC = readTemperatureTC();

  Serial.print("Temperature: "); Serial.println(tempC);

  //read calibration knob
  tempRange = readTempRange();
  //read init button
  if(digitalRead(initButton))
  {
    midTemp = tempC;
  }
  
  Serial.print("tempRange: "); Serial.println(tempRange);
  Serial.print("midTemp: "); Serial.println(midTemp);
  
 double degPerBar = tempRange/numLEDs;
 double startTemp = midTemp-0.5*tempRange;
 for (uint8_t b=0; b<numLEDs; b++) {
  if(startTemp+b*degPerBar<tempC)
  {
    bar.setBar(b, LED_YELLOW);
  }
  else
  {
    bar.setBar(b, LED_OFF); 
  }
 }
 bar.writeDisplay();
 delay(20);
}

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
double readTempRange()
{
  return mapf(analogRead(scalePot), 0, 1023, 0.5, 8);
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

