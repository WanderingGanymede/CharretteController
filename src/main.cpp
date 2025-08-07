#include "config.h"
#include "VescComm.h"

const int CALIBRATE_BUTTON_PIN = 4; // Use pin 4 for calibration button
#include <Arduino.h>
#include "HX711_ADC.h"
#include "LoadCell.h"
// put function declarations here:
int myFunction(int, int);

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

float minLoadCellValue = 0;
float maxLoadCellValue = 0;
// pins:
const int HX711_dout = 2; // mcu > HX711 dout pin
const int HX711_sck = 3;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

void registerMinMax(HX711_ADC &cell)
{
  Serial.println("***");
  Serial.println("Pull as hard as you can on the load cell for 5 seconds!");
  unsigned long startTime = millis();
  maxLoadCellValue = cell.getData();
  while (millis() - startTime < 5000)
  {
    cell.update();
    float val = cell.getData();
    if (val > maxLoadCellValue)
      maxLoadCellValue = val;
  }
  Serial.print("Max value registered: ");
  Serial.println(maxLoadCellValue);

  delay(1000);
  Serial.println("Now push as hard as you can on the load cell for 5 seconds!");
  startTime = millis();
  minLoadCellValue = cell.getData();
  while (millis() - startTime < 5000)
  {
    cell.update();
    float val = cell.getData();
    if (val < minLoadCellValue)
      minLoadCellValue = val;
  }
  Serial.print("Min value registered: ");
  Serial.println(minLoadCellValue);
  Serial.println("***");
}

void setup()
{
  Serial.begin(9600);
  // Start Serial1 for VESC UART
  // setupVesc(115200, &Serial1); // Pass Serial1 to VESC UART setup
  delay(10);
  Serial.println();
  Serial.println("Starting Charrette...");

  LoadCell.begin();
  // LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP); // Setup calibration button pin
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  }
  else
  {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update())
    ;
  calibrate(LoadCell, true, calVal_eepromAdress); // start calibration procedure
  registerMinMax(LoadCell);                       // register min/max values after calibration
}

void loop()
{

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; // increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update())
    newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady)
  {
    if (millis() > t + serialPrintInterval)
    {
      float i = LoadCell.getData();
      // Serial.print("Load_cell output val: ");
      // Serial.println(i);
      //  Map value to 0-1
      float mapped = 0;
      if (maxLoadCellValue != minLoadCellValue)
      {
        mapped = (i - minLoadCellValue) / (maxLoadCellValue - minLoadCellValue);
        if (mapped < 0)
          mapped = 0;
        if (mapped > 1)
          mapped = 1;
      }
      Serial.print("Mapped value (0-1): ");
      Serial.println(mapped, 3);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal
  if (Serial.available() > 0)
  {
    char inByte = Serial.read();
    if (inByte == 't')
      LoadCell.tareNoDelay(); // tare
    else if (inByte == 'r')
      calibrate(LoadCell, true, calVal_eepromAdress); // calibrate
    else if (inByte == 'c')
      changeSavedCalFactor(LoadCell, true, calVal_eepromAdress); // edit calibration value manually
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true)
  {
    Serial.println("Tare complete");
  }
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}