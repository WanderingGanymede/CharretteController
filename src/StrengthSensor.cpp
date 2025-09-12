#include "StrengthSensor.h"

StrengthSensor::StrengthSensor(uint8_t dataPin, uint8_t sckPin, double offset, float calibration, float thresholdEq) : _cell(dataPin, sckPin)
{
  _dataPin = dataPin;
  _sckPin = sckPin;
  _offset = offset;
  _calibrationValue = calibration;
  _threshold_sensor_equilibrium = thresholdEq;
}

void StrengthSensor::begin()
{
  _cell.begin();
}

bool StrengthSensor::start(bool tare)
{
  _cell.start(2000, tare);
  if (_cell.getTareTimeoutFlag())
    return 0;
  else
  {
    _cell.setCalFactor(_calibrationValue);
    return 1;
  }
}

double StrengthSensor::getRaw()
{
  return _cell.getData();
}

void StrengthSensor::setOffset(double offset)
{
  _offset = offset;
}

void StrengthSensor::update(bool *newDataReady, double *value)
{
  if (_cell.update())
  {
    *newDataReady = true;
    // inversion du signe pour que tirer donne une valeur positive
    *value = (_cell.getData() - _offset) * (-1.0);
    if (abs(*value) < _threshold_sensor_equilibrium)
      *value = 0.0;
    else if (*value > 0.0)
      *value -= _threshold_sensor_equilibrium;
    else
      *value += _threshold_sensor_equilibrium;
  }
}
/*

  if(_cell.update()){
    *newDataReady = true;
  if(abs(*value)<_threshold_sensor_equilibrium)
    *value = 0.0;

  else{
    _tmpValue = _cell.getData() - _offset;
    if (_tmpValue<0)
      *value = _tmpValue + _threshold_sensor_equilibrium;
    else
      *value = _tmpValue -_threshold_sensor_equilibrium;
  }
  }
*/

void StrengthSensor::setSamplesInUse(int samples)
{
  _cell.setSamplesInUse(samples);
}

void StrengthSensor::setThresholdSensor(float new_threshold)
{
  _threshold_sensor_equilibrium = new_threshold;
}
