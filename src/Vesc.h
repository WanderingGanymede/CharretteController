
#pragma once
#include <Arduino.h>
#include <VescUart.h>

class Vesc
{
public:
    Vesc();
    void setSerialPort(HardwareSerial *serial);
    bool getValues();
    void setCurrent(float current);
    void setDuty(float duty);
    void setBrake(float brake);
    float getRPM();
    float getVoltage();
    float getCurrent();
    float getDuty();
    float getTempMosfet();
    float getTempMotor();
    float getAmpHours();
    float getAmpHoursCharged();
    float getWattHours();
    float getWattHoursCharged();
    long getTachometer();
    long getTachometerAbs();

private:
    VescUart vescUart;
};
