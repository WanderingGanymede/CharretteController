#ifndef INPUT_CONTROL_H
#define INPUT_CONTROL_H

#include <Arduino.h>
#include <Chrono.h>

// Toggle pins and state
extern int toggle4Pin, toggle3Pin, toggle2Pin, toggle1Pin;
extern Chrono toggle1Chrono, toggle2Chrono, toggle3Chrono, toggle4Chrono;
extern bool toggle1Debouncing, toggle2Debouncing, toggle3Debouncing, toggle4Debouncing;
extern int toggle1State, toggle2State, toggle3State, toggle4State;
extern int toggle1NewState, toggle2NewState, toggle3NewState, toggle4NewState;

// Button (momentary) pins and state
extern int buttonA, buttonB, buttonC;
extern bool buttonAState, buttonBState, buttonCState;
extern bool buttonAPrevState, buttonBPrevState, buttonCPrevState;

void setupInputControl();
void InputControlHandler();

// ADC input handling for Pi Pico
extern int adc0Value;
extern int adc1Value;
void readADCInputs();
int getADC0();
int getADC1();
float getBrakesMapped(float in_min = 280, float in_max = 785, float out_min = 0, float out_max = 1);
#endif
