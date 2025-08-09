#include "Vesc.h"
// Vesc.cpp - encapsulated VESC interface using VescUart directly, no VescComm dependency
#include "Vesc.h"
#include <VescUart.h>

Vesc::Vesc() : vescUart() {}

void Vesc::setSerialPort(HardwareSerial *serial)
{
    vescUart.setSerialPort(serial);
}

bool Vesc::getValues()
{
    return vescUart.getVescValues();
}

void Vesc::setCurrent(float current)
{
    vescUart.setCurrent(current);
}

void Vesc::setDuty(float duty)
{
    vescUart.setDuty(duty);
}

void Vesc::setBrake(float brake)
{
    vescUart.setBrakeCurrent(brake);
}

float Vesc::getRPM()
{
    return vescUart.data.rpm;
}

float Vesc::getVoltage()
{
    return vescUart.data.inpVoltage;
}

float Vesc::getCurrent()
{
    return vescUart.data.avgMotorCurrent;
}

float Vesc::getDuty()
{
    return vescUart.data.dutyCycleNow;
}

float Vesc::getTempMosfet()
{
    return vescUart.data.tempMosfet;
}

float Vesc::getTempMotor()
{
    return vescUart.data.tempMotor;
}

float Vesc::getAmpHours()
{
    return vescUart.data.ampHours;
}

float Vesc::getAmpHoursCharged()
{
    return vescUart.data.ampHoursCharged;
}

float Vesc::getWattHours()
{
    return vescUart.data.wattHours;
}

float Vesc::getWattHoursCharged()
{
    return vescUart.data.wattHoursCharged;
}

long Vesc::getTachometer()
{
    return vescUart.data.tachometer;
}

long Vesc::getTachometerAbs()
{
    return vescUart.data.tachometerAbs;
}
