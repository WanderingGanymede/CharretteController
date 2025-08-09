#include "BikeDisplay.h"
#include "bmps.h" // Your bitmap definitions
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

BikeDisplay::BikeDisplay() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET),
                             canvasSecText(SEC_TEX_W, SEC_TEX_H),
                             canvasMainText(MAIN_TEX_W, MAIN_TEX_H)
{
    // Initialize bitmap pointers
    bmp_MainLeft = ::bmp_MainLeft;
    bmp_MainRight = ::bmp_MainRight;
    bmp_SecondaryLeft = ::bmp_SecondaryLeft;
    bmp_SecondaryRight = ::bmp_SecondaryRight;

    // Initialize bitmap dimensions
    main_arrow_width = ::main_arrow_width;
    main_arrow_height = ::main_arrow_height;
    sec_arrow_width = ::sec_arrow_width;
    sec_arrow_height = ::sec_arrow_height;
}

void BikeDisplay::begin()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    canvasMainText.setFont(&FreeSansBold12pt7b);
    canvasSecText.setFont(&FreeSansBold9pt7b);

    display.display();
    delay(200);
    clear();
}
// Display an arbitrary text message centered on the screen
void BikeDisplay::displayMessage(const String &msg)
{
    clear();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    int16_t x, y;
    uint16_t w, h;
    display.setFont(); // Use default font for compatibility
    display.getTextBounds(msg, 0, 0, &x, &y, &w, &h);
    int16_t xPos = (display.width() - w) / 2;
    int16_t yPos = (display.height() - h) / 2;
    display.setCursor(xPos, yPos);
    display.println(msg);
    update();
}
void BikeDisplay::clear()
{
    display.clearDisplay();
}

void BikeDisplay::update()
{
    display.display();
}

String BikeDisplay::formatDistance(int meters)
{
    if (meters >= 1000)
    {
        return String(meters / 1000.0, 1) + "k";
    }
    return String(meters) + "m";
}

void BikeDisplay::drawCenteredCanvas(GFXcanvas1 &canvas, const String &text, int yPosition)
{
    int16_t x, y;
    uint16_t w, h;

    canvas.fillScreen(0);
    canvas.setCursor(0, canvas.height() - 1);
    canvas.getTextBounds(text, 0, 0, &x, &y, &w, &h);
    canvas.println(text);

    int displayX = (display.width() - w) / 2;
    display.drawBitmap(displayX, yPosition, canvas.getBuffer(), canvas.width(), canvas.height(), SSD1306_WHITE);
}

// Arrow functions
void BikeDisplay::showMainLeftArrow()
{
    display.drawBitmap(0, 0, bmp_MainLeft, main_arrow_width, main_arrow_height, SSD1306_WHITE);
}

void BikeDisplay::showMainRightArrow()
{
    display.drawBitmap(display.width() - main_arrow_width, 0, bmp_MainRight, main_arrow_width, main_arrow_height, SSD1306_WHITE);
}

void BikeDisplay::showSecondaryLeftArrow()
{
    display.drawBitmap(11, 5, bmp_SecondaryLeft, sec_arrow_width, sec_arrow_height, SSD1306_WHITE);
}

void BikeDisplay::showSecondaryRightArrow()
{
    display.drawBitmap(106, 5, bmp_SecondaryRight, sec_arrow_width, sec_arrow_height, SSD1306_WHITE);
}

void BikeDisplay::showAllArrows()
{
    clear();
    showMainLeftArrow();
    showMainRightArrow();
    showSecondaryLeftArrow();
    showSecondaryRightArrow();
    update();
}

// Distance display functions
void BikeDisplay::displayMainDistance(int distance)
{
    drawCenteredCanvas(canvasMainText, formatDistance(distance), SEC_TEX_H - 1);
    update();
}

void BikeDisplay::displaySecondaryDistance(int distance)
{
    drawCenteredCanvas(canvasSecText, formatDistance(distance), 1);
    update();
}

// Display up to 4 parameters at the screen corners
void BikeDisplay::displayFourParams(const String &param1, const String &param2, const String &param3, const String &param4)
{
    clear();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setFont(); // Use default font for compatibility

    // Top left (param1)
    display.setCursor(0, 0 + 8); // y=8 for baseline alignment
    display.print(param1);

    // Top right (param2)
    int16_t x2, y2;
    uint16_t w2, h2;
    display.getTextBounds(param2, 0, 0, &x2, &y2, &w2, &h2);
    display.setCursor(display.width() - w2, 0 + 8);
    display.print(param2);

    // Bottom left (param3)
    int16_t yBottom = display.height() - 2; // y for bottom row
    display.setCursor(0, yBottom);
    display.print(param3);

    // Bottom right (param4)
    int16_t x4, y4;
    uint16_t w4, h4;
    display.getTextBounds(param4, 0, 0, &x4, &y4, &w4, &h4);
    display.setCursor(display.width() - w4, yBottom);
    display.print(param4);

    update();
}