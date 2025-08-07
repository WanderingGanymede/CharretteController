#include "HX711_ADC.h"

#include <EEPROM.h>
#include "config.h" // For CALIBRATE_BUTTON_PIN definition

void calibrate(HX711_ADC cell, boolean save_to_eprom, int calVal_eepromAdress)
{
    Serial.println("***");
    Serial.println("Start calibration:");
    Serial.println("Place the load cell on a level stable surface.");
    Serial.println("Remove any load applied to the load cell.");
    Serial.println("Press the calibration button to set the tare offset.");

    boolean _resume = false;
    while (_resume == false)
    {
        cell.update();
        if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) // Button pressed (assuming pull-up)
        {
            cell.tareNoDelay();
        }
        if (cell.getTareStatus() == true)
        {
            Serial.println("Tare complete");
            _resume = true;
        }
    }

    Serial.println("Now, place your known mass on the loadcell.");
    Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

    float known_mass = 0;
    _resume = false;
    while (_resume == false)
    {
        cell.update();
        if (Serial.available() > 0)
        {
            known_mass = Serial.parseFloat();
            if (known_mass != 0)
            {
                Serial.print("Known mass is: ");
                Serial.println(known_mass);
                _resume = true;
            }
        }
    }

    cell.refreshDataSet();                                          // refresh the dataset to be sure that the known mass is measured correct
    float newCalibrationValue = cell.getNewCalibration(known_mass); // get the new calibration value

    Serial.print("New calibration value has been set to: ");
    Serial.print(newCalibrationValue);
    Serial.println(", use this as calibration value (calFactor) in your project sketch.");


    _resume = false;

    if (save_to_eprom)
    {
#if defined(ESP8266) || defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266) || defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
    }
    else
    {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
    }

    Serial.println("End calibration");
    Serial.println("***");
    Serial.println("To re-calibrate, send 'r' from serial monitor.");
    Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
    Serial.println("***");
}

void changeSavedCalFactor(HX711_ADC cell, boolean save_to_eprom, int calVal_eepromAdress)
{
    Serial.println("***");
    Serial.println("Start calibration:");
    Serial.println("Place the load cell an a level stable surface.");
    Serial.println("Remove any load applied to the load cell.");
    Serial.println("Send 't' from serial monitor to set the tare offset.");
    float oldCalibrationValue = cell.getCalFactor();
    boolean _resume = false;
    Serial.println("***");
    Serial.print("Current value is: ");
    Serial.println(oldCalibrationValue);
    Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
    float newCalibrationValue;
    while (_resume == false)
    {
        if (Serial.available() > 0)
        {
            newCalibrationValue = Serial.parseFloat();
            if (newCalibrationValue != 0)
            {
                Serial.print("New calibration value is: ");
                Serial.println(newCalibrationValue);
                cell.setCalFactor(newCalibrationValue);
                _resume = true;
            }
        }
    }
    _resume = false;
    Serial.print("Save this value to EEPROM adress ");
    Serial.print(calVal_eepromAdress);
    Serial.println("? y/n");
    if (save_to_eprom)
    {
#if defined(ESP8266) || defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266) || defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
    }
    else
    {
        Serial.println("Value not saved to EEPROM");
    }
    Serial.println("End change calibration value");
    Serial.println("***");
}