#pragma once

#include <Arduino.h>

/**
 * WifiLogger
 *
 * Creates a Wi-Fi SoftAP on the attached ESP8285 using raw AT commands
 * (no WiFiEspAT library) and exposes a TCP server to which a laptop can
 * connect to receive CSV-formatted telemetry lines in real time.
 *
 * Mirrors the approach proven in examplemain.cpp.
 *
 * Usage (in main.cpp):
 *   // --- globals ---
 *   WifiLogger wifiLogger("PicoTrailer", "12345678");
 *
 *   // --- setup() ---
 *   Serial1.setRX(<rx_pin>);
 *   Serial1.setTX(<tx_pin>);
 *   Serial1.begin(115200);
 *   wifiLogger.begin(Serial1);
 *
 *   // --- loop() ---
 *   wifiLogger.update();       // near the top, every loop — parses ESP events
 *   wifiLogger.log(csvLine);   // inside the 200 ms telemetry block
 *
 * On the laptop, connect to the SoftAP then run:
 *   python3 -c "import socket; s=socket.create_connection(('192.168.4.1',8888)); [print(l,end='') for l in s.makefile()]"
 */
class WifiLogger
{
public:
    // CSV column names — sent automatically to every new client that connects.
    static const char* const CSV_HEADER;

    /**
     * @param ssid      SSID of the SoftAP the ESP will broadcast.
     * @param password  WPA2 password (min 8 chars).
     * @param port      TCP port to listen on (default 8888).
     */
    WifiLogger(const char* ssid, const char* password, uint16_t port = 8888);

    /**
     * Send the full AT init sequence to the ESP and bring up the SoftAP +
     * TCP server. `espSerial` must already be started (setRX/setTX/begin)
     * by the caller before this is called.
     * Returns true on success.
     */
    bool begin(Stream& espSerial);

    /**
     * Must be called every loop().
     * Reads unsolicited ESP output and updates connection state:
     *   "X,CONNECT" → marks a client as connected
     *   "X,CLOSED"  → marks the client as disconnected
     * Non-blocking.
     */
    void update();

    /**
     * Send one pre-formatted CSV line to the connected client via AT+CIPSEND.
     * No-op if no client is connected or begin() failed.
     * Waits at most 500 ms for the '>' prompt — if it doesn't arrive the
     * client is assumed to have disconnected.
     */
    void log(const String& line);

    /** True when a TCP client is currently connected. */
    bool isClientConnected();

private:
    // Send an AT command and wait for an expected response string.
    // Prints the command and result to Serial for debugging.
    bool sendAT(const char* cmd, const char* expect = "OK", unsigned long timeout = 2000);

    // Block until `target` appears in the ESP stream or timeout expires.
    bool waitFor(const char* target, unsigned long timeout);

    const char* _ssid;
    const char* _password;
    uint16_t    _port;
    Stream*     _serial;          // pointer to the ESP AT serial port
    bool        _ready;           // true after a successful begin()
    bool        _clientConnected; // toggled by parsing "X,CONNECT" / "X,CLOSED"
    int         _connectionId;    // the CIPMUX connection id (0–4)
    String      _lineBuffer;      // accumulates chars between '\n' in update()
};