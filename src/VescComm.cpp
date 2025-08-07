#include "VescComm.h"

VescUart vesc;

void setupVesc(long baudrate, Stream *serialPort)
{
    
    vesc.setSerialPort(serialPort);
}

bool getVescValues()
{
    return vesc.getVescValues();
}

void setVescDuty(float duty)
{
    vesc.setDuty(duty);
}

void setVescCurrent(float current)
{
    vesc.setCurrent(current);
}

void setVescBrake(float brake)
{
    vesc.setBrakeCurrent(brake);
}

float getVescRPM()
{
    return vesc.data.rpm;
}

float getVescVoltage()
{
    return vesc.data.inpVoltage;
}

float getVescCurrent()
{
    return vesc.data.avgMotorCurrent;
}

float getVescDuty()
{
    return vesc.data.dutyCycleNow;
}

float getVescTempMosfet()
{
    return vesc.data.tempMosfet;
}

float getVescTempMotor()
{
    return vesc.data.tempMotor;
}

float getVescAmpHours()
{
    return vesc.data.ampHours;
}

float getVescAmpHoursCharged()
{
    return vesc.data.ampHoursCharged;
}

float getVescWattHours()
{
    return vesc.data.wattHours;
}

float getVescWattHoursCharged()
{
    return vesc.data.wattHoursCharged;
}

long getVescTachometer()
{
    return vesc.data.tachometer;
}

long getVescTachometerAbs()
{
    return vesc.data.tachometerAbs;
}
