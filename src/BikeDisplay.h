#ifndef BIKE_DISPLAY_H
#define BIKE_DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class BikeDisplay
{
public:
    BikeDisplay();
    void begin();
    void clear();
    void update();

    // Arrow drawing functions
    void showMainLeftArrow();
    void showMainRightArrow();
    void showSecondaryLeftArrow();
    void showSecondaryRightArrow();
    void showAllArrows();

    // Distance display functions
    void displayMainDistance(int distance);
    void displaySecondaryDistance(int distance);

    // Display a text message centered on the screen
    void displayMessage(const String &msg);
    // Display up to 4 parameters at the screen corners
    void displayFourParams(const String &param1, const String &param2, const String &param3, const String &param4);

private:
    Adafruit_SSD1306 display;

    // Canvas dimensions
    static const uint8_t SEC_TEX_W = 80;
    static const uint8_t SEC_TEX_H = 13;
    static const uint8_t MAIN_TEX_W = 80;
    static const uint8_t MAIN_TEX_H = 20;

    GFXcanvas1 canvasSecText;
    GFXcanvas1 canvasMainText;

    // Bitmaps (ensure these are defined in your bmps.h)
    const unsigned char *bmp_MainLeft;
    const unsigned char *bmp_MainRight;
    const unsigned char *bmp_SecondaryLeft;
    const unsigned char *bmp_SecondaryRight;

    // Dimensions from bmps.h
    uint8_t main_arrow_width;
    uint8_t main_arrow_height;
    uint8_t sec_arrow_width;
    uint8_t sec_arrow_height;

    String formatDistance(int meters);
    void drawCenteredCanvas(GFXcanvas1 &canvas, const String &text, int yPosition);
};

#endif