
#ifndef LOADCELL_H
#define LOADCELL_H

#include "HX711_ADC.h"
#include <EEPROM.h>

void calibrate(HX711_ADC cell, boolean save_to_eprom, int calVal_eepromAdress);
void changeSavedCalFactor(HX711_ADC cell, boolean save_to_eprom, int calVal_eepromAdress);
#endif // LOADCELL_H
