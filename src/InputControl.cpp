#include "InputControl.h"
#include <Arduino.h>
#include <Chrono.h>
#include "main.h"
// --- Toggle pins and state ---
int toggle4Pin = 19;
int toggle3Pin = 20;
int toggle2Pin = 21;
int toggle1Pin = 22;
Chrono toggle1Chrono, toggle2Chrono, toggle3Chrono, toggle4Chrono;
bool toggle1Debouncing = 0;
bool toggle2Debouncing = 0;
bool toggle3Debouncing = 0;
bool toggle4Debouncing = 0;
int toggle1State = 0;
int toggle2State = 0;
int toggle3State = 0;
int toggle4State = 0;
int toggle1NewState = 0;
int toggle2NewState = 0;
int toggle3NewState = 0;
int toggle4NewState = 0;

// --- Button (momentary) pins and state ---
int buttonA = 23;
int buttonB = 24;
int buttonC = 25;
bool buttonAState = 0, buttonBState = 0, buttonCState = 0;
bool buttonAPrevState = 0, buttonBPrevState = 0, buttonCPrevState = 0;

// Forward declarations for user actions
void setMode(int mode);
void buttonAPressed()
{
    Serial.println("Button A pressed");
    // Implement your action for button A press here
}
void buttonAReleased()
{
    Serial.println("Button A released");
    // Implement your action for button A release here
}
void buttonBPressed()
{
    Serial.println("Button B pressed");
    // Implement your action for button A press here
}
void buttonBReleased()
{
    Serial.println("Button B released");
    // Implement your action for button A release here
}
void buttonCPressed()
{
    Serial.println("Button c pressed");
    // Implement your action for button c press here
}
void buttonCReleased()
{
    Serial.println("Button C released");
    // Implement your action for button A release here
}
void buttonBPressed();
void buttonBReleased();
void buttonCPressed();
void buttonCReleased();

const int debounceTime = 75;

void setupInputControl()
{

    pinMode(toggle1Pin, INPUT_PULLUP);
    pinMode(toggle2Pin, INPUT_PULLUP);
    pinMode(toggle3Pin, INPUT_PULLUP);
    pinMode(toggle4Pin, INPUT_PULLUP);
    pinMode(buttonA, INPUT_PULLUP);
    pinMode(buttonB, INPUT_PULLUP);
    pinMode(buttonC, INPUT_PULLUP);
}

void InputControlHandler()
{
    // Toggle 2 logic
    if (digitalRead(toggle2Pin) != toggle2State && !toggle2Debouncing)
    {
        toggle2Chrono.restart();
        toggle2NewState = digitalRead(toggle2Pin);
        toggle2Debouncing = 1;
        Serial.print("Button 2 pressed starting debounce check, new state: ");
        Serial.println(toggle2NewState);
    }
    if (toggle2Chrono.elapsed() > debounceTime && digitalRead(toggle2Pin) == toggle2NewState)
    {
        toggle2State = toggle2NewState;
        toggle2Chrono.restart();
        toggle2Chrono.stop();
        toggle2Debouncing = 0;
        Serial.print("Button 2 state changed to: ");
        Serial.println(toggle2State);
        setMode(toggle2State);
    }
    // Toggle 1 logic
    if (digitalRead(toggle1Pin) != toggle1State && !toggle1Debouncing)
    {
        toggle1Chrono.restart();
        toggle1NewState = digitalRead(toggle1Pin);
        toggle1Debouncing = 1;
        Serial.print("Button 1 pressed starting debounce check, new state: ");
        Serial.println(toggle1NewState);
    }
    if (toggle1Chrono.elapsed() > debounceTime && digitalRead(toggle1Pin) == toggle1NewState)
    {
        toggle1State = toggle1NewState;
        toggle1Chrono.restart();
        toggle1Chrono.stop();
        toggle1Debouncing = 0;
        Serial.print("Button 1 state changed to: ");
        Serial.println(toggle1State);
        // TODO: handle toggle 1 action here
    }
    // Toggle 3 logic
    if (digitalRead(toggle3Pin) != toggle3State && !toggle3Debouncing)
    {
        toggle3Chrono.restart();
        toggle3NewState = digitalRead(toggle3Pin);
        toggle3Debouncing = 1;
        Serial.print("Button 3 pressed starting debounce check, new state: ");
        Serial.println(toggle3NewState);
    }
    if (toggle3Chrono.elapsed() > debounceTime && digitalRead(toggle3Pin) == toggle3NewState)
    {
        toggle3State = toggle3NewState;
        toggle3Chrono.restart();
        toggle3Chrono.stop();
        toggle3Debouncing = 0;
        Serial.print("Button 3 state changed to: ");
        Serial.println(toggle3State);
        // TODO: handle toggle 3 action here
    }
    // Toggle 4 logic
    if (digitalRead(toggle4Pin) != toggle4State && !toggle4Debouncing)
    {
        toggle4Chrono.restart();
        toggle4NewState = digitalRead(toggle4Pin);
        toggle4Debouncing = 1;
        Serial.print("Button 4 pressed starting debounce check, new state: ");
        Serial.println(toggle4NewState);
    }
    if (toggle4Chrono.elapsed() > debounceTime && digitalRead(toggle4Pin) == toggle4NewState)
    {
        toggle4State = toggle4NewState;
        toggle4Chrono.restart();
        toggle4Chrono.stop();
        toggle4Debouncing = 0;
        Serial.print("Button 4 state changed to: ");
        Serial.println(toggle4State);
        // TODO: handle toggle 4 action here
    }
    // --- Momentary button logic ---
    buttonAState = !digitalRead(buttonA);
    buttonBState = !digitalRead(buttonB);
    buttonCState = !digitalRead(buttonC);
    if (buttonAState && !buttonAPrevState)
        buttonAPressed();
    if (!buttonAState && buttonAPrevState)
        buttonAReleased();
    buttonAPrevState = buttonAState;
    if (buttonBState && !buttonBPrevState)
        buttonBPressed();
    if (!buttonBState && buttonBPrevState)
        buttonBReleased();
    buttonBPrevState = buttonBState;
    if (buttonCState && !buttonCPrevState)
        buttonCPressed();
    if (!buttonCState && buttonCPrevState)
        buttonCReleased();
    buttonCPrevState = buttonCState;
}
