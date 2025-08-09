# Display Library Dependencies

This project uses the Adafruit SSD1306 and Adafruit GFX libraries for the BikeDisplay class. Please ensure these libraries are installed in your PlatformIO environment:

- Adafruit SSD1306
- Adafruit GFX Library

You can install them with:

```
pio lib install "Adafruit SSD1306"
pio lib install "Adafruit GFX Library"
```

Or add them to your `platformio.ini` under `[env]` with:

```
libraries =
    Adafruit SSD1306
    Adafruit GFX Library
```
