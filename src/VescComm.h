#ifndef VESCCOMM_H
#define VESCCOMM_H

#include <VescUart.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

extern VescUart vesc;

void setupVesc(long baudrate, Stream *serialPort);
bool getVescValues();
void setVescDuty(float duty);
void setVescCurrent(float current);
void setVescBrake(float brake);

// VESC telemetry getters
float getVescRPM();
float getVescVoltage();
float getVescCurrent();
float getVescDuty();
float getVescTempMosfet();
float getVescTempMotor();
float getVescAmpHours();
float getVescAmpHoursCharged();
float getVescWattHours();
float getVescWattHoursCharged();
long getVescTachometer();
long getVescTachometerAbs();

#endif // VESCCOMM_H
