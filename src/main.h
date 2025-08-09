#pragma once

// Display debug info on the OLED screen
void displayDebugInfo();

// Show 4 parameters at the screen corners
// (top left, top right, bottom left, bottom right)
// Implemented in BikeDisplay.cpp as a member function
// Usage: bikeDisplay.displayFourParams(...)

// Existing function declarations
void updateVescTelemetry();
int InitializeStrengthSensor();
int initializeVesc();
void test2();
void interruptCtrlAlive();
void setMode(int mode);
void miseAJourVitesse();
void setPIDMode(bool walkOrNot);
bool transition01();
bool transition07();
bool transition12();
bool transition15();
bool transition23();
bool transition32();
bool transition34();
bool transition42();
bool transition45();
bool transition52();
bool transition51();
bool transition70();
bool transition0();
bool transition5();
