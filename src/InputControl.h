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

#endif
