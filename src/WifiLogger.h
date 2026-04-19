#pragma once

#include <WiFiEspAT.h>

/**
 * WifiLogger
 *
 * Creates a Wi-Fi SoftAP on the attached ESP8285 (via Serial1 / AT commands)
 * and exposes a single TCP server to which a laptop can connect to receive
 * CSV-formatted telemetry lines in real time.
 *
 * Usage (in main.cpp):
 *   // --- globals ---
 *   WifiLogger wifiLogger("PicoTrailer", "12345678");
 *
 *   // --- setup() ---
 *   Serial1.setRX(serial1RX);
 *   Serial1.setTX(serial1TX);
 *   Serial1.begin(115200);
 *   wifiLogger.begin(Serial1);
 *
 *   // --- loop() ---
 *   wifiLogger.update();                  // near the top, every loop
 *   wifiLogger.log(csvLine);              // inside the 200 ms telemetry block
 *
 * On the laptop, connect to the "PicoTrailer" network and run:
 *   python3 -c "import socket,sys; s=socket.create_connection(('192.168.4.1',8888)); [print(l,end='') for l in s.makefile()]"
 */
class WifiLogger
{
public:
    // CSV column names — sent automatically to every new client that connects.
    static const char* const CSV_HEADER;

    /**
     * @param ssid      SSID of the SoftAP the ESP will broadcast.
     * @param password  WPA2 password (min 8 chars). Pass nullptr for open AP.
     * @param port      TCP port to listen on (default 8888).
     */
    WifiLogger(const char* ssid, const char* password, uint16_t port = 8888);


    /**
     * Initialise the ESP module, bring up the SoftAP, and start the TCP server.
     *
     * `espSerial` must already be configured (setRX/setTX) and started
     * (begin(baud)) by the caller before this is called — this mirrors the
     * pattern used for Serial2 / VESC elsewhere in the project.
     *
     * Call once in setup(). Returns true on success.
     */
    bool begin(Stream& espSerial);

    /**
     * Must be called every loop().
     * Accepts incoming connections and cleans up disconnected clients.
     * Non-blocking.
     */
    void update();

    /**
     * Send one pre-formatted CSV line to the connected client.
     * Completely non-blocking: silently drops the line if no client is
     * connected or the logger failed to initialise.
     *
     * @param line  A single CSV row (no trailing newline needed).
     */
    void log(const String& line);

    /** True when a TCP client is currently connected. */
    bool isClientConnected();

private:
    const char* _ssid;
    const char* _password;
    uint16_t    _port;
    WiFiServer  _server;
    WiFiClient  _client;
    bool        _ready;   // set to true only after a successful begin()
};