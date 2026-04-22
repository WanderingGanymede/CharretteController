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
 *   wifiLogger.update();
 *   if (wifiLogger.newClientConnected())
 *     wifiLogger.log(String(WifiLogger::CSV_HEADER));
 *   wifiLogger.log(csvLine);
 *
 * On the laptop, connect to the SoftAP then run:
 *   python3 client.py
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
     *   "X,CONNECT"        → marks a client as connected
     *   "X,CLOSED"         → marks the client as disconnected
     *   "+IPD,X,N:<data>"  → stores incoming TCP data as a pending command
     * Non-blocking. Never calls log() internally.
     */
    void update();

    /**
     * Returns true exactly once after a new client connects.
     * The caller must send the CSV header in response.
     * This exists so log() is never called from inside update(), which
     * would block the serial stream for up to 2 s and swallow +IPD bytes.
     */
    bool newClientConnected();

    /**
     * Returns true if a complete command line was received from the TCP
     * client since the last call to getCommand().
     */
    bool hasCommand();

    /**
     * Returns the last received command string and clears the pending flag.
     * Call only after hasCommand() returns true.
     */
    String getCommand();

    /**
     * Send one pre-formatted line to the connected client via AT+CIPSEND.
     * Drains stale bytes first, waits up to 500 ms for '>', then waits for
     * SEND OK before returning — safe for burst sends (e.g. GET ALL).
     * Sets _clientConnected = false if '>' never arrives.
     * No-op if no client is connected or begin() failed.
     */
    void log(const String& line);

    /**
     * High-frequency streaming variant of log().
     * Drains stale bytes first, waits up to 300 ms for '>', sends, then
     * waits up to 150 ms for SEND OK to leave the ESP in a clean state.
     * Does NOT set _clientConnected = false on a '>' timeout — a missed
     * sample is harmless, but a false disconnect kills streaming permanently.
     * No-op if no client is connected or begin() failed.
     */
    void logStream(const String& line);

    /** True when a TCP client is currently connected. */
    bool isClientConnected();

private:
    // Send an AT command and wait for an expected response string.
    bool sendAT(const char* cmd, const char* expect = "OK", unsigned long timeout = 2000);

    // Block until `target` appears in the ESP stream or timeout expires.
    bool waitFor(const char* target, unsigned long timeout);

    const char* _ssid;
    const char* _password;
    uint16_t    _port;
    Stream*     _serial;             // pointer to the ESP AT serial port
    bool        _ready;              // true after a successful begin()
    bool        _clientConnected;    // toggled by parsing "X,CONNECT" / "X,CLOSED"
    int         _connectionId;       // the CIPMUX connection id (0-4)
    String      _lineBuffer;         // accumulates chars between '\n' in update()

    bool        _newClientConnected; // set by update(), cleared by newClientConnected()
    bool        _hasCommand;         // true when a complete command line is waiting
    String      _pendingCommand;     // the last received command text
};