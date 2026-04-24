#pragma once

#include <Arduino.h>
#include "pico/critical_section.h"

/**
 * AsyncWifiLogger
 *
 * Non‑blocking WiFi logger for ESP8285 (AT firmware) running on a separate
 * RP2040 core. All AT communication happens in update() which must be called
 * repeatedly from core 1. Core 0 queues messages via sendAsync().
 *
 * No waitFor() loops block the core; the AT+CIPSEND sequence is a state machine
 * that advances on each update() call.
 */
class AsyncWifiLogger
{
public:
    // CSV column names — sent automatically to every new client that connects.
    static const char* const CSV_HEADER;

    /**
     * @param ssid      SSID of the SoftAP the ESP will broadcast.
     * @param password  WPA2 password (min 8 chars).
     * @param port      TCP port to listen on (default 8888).
     */
    AsyncWifiLogger(const char* ssid, const char* password, uint16_t port = 8888);

    /**
     * Must be called on core 1. Sends the full AT init sequence.
     * `espSerial` must already be started (setRX/setTX/begin) by the caller.
     * Returns true on success.
     */
    bool begin(Stream& espSerial);

    /**
     * Must be called repeatedly on core 1 (e.g., in loop1()).
     * - Reads and parses unsolicited ESP messages.
     * - Advances the AT+CIPSEND state machine for queued outgoing data.
     * - Updates connection state flags.
     * Never blocks for more than a few microseconds.
     */
    void update();

    /**
     * Called from core 0. Queues a line for transmission.
     * Non‑blocking: returns immediately. If the queue is full, the line is dropped.
     */
    void sendAsync(const String& line);

    /** True once after each new client connection (core‑safe). */
    bool newClientConnected();

    /** True if a complete command line was received (core‑safe). */
    bool hasCommand();

    /** Returns the last command and clears the flag (core‑safe). */
    String getCommand();

    /** True when a TCP client is currently connected (core‑safe). */
    bool isClientConnected();

private:
    // AT+CIPSEND state machine states
    enum SendState {
        STATE_IDLE,
        STATE_WAIT_PROMPT,
        STATE_SEND_DATA,
        STATE_WAIT_OK
    };

    // Thread‑safe queue for strings (single producer, single consumer)
    struct StringQueue {
        static const int QSIZE = 32;
        String buffer[QSIZE];
        volatile int head = 0;
        volatile int tail = 0;
        critical_section_t crit;

        StringQueue();
        bool push(const String& s);   // true if success
        bool pop(String& s);
        void clear();
    };

    // Helper AT commands (non‑blocking)
    void sendAT(const char* cmd);
    bool checkResponse(const char* expected, unsigned long timeout);

    // State machine for sending one queued line
    void processSendState();

    // Read available bytes from ESP and parse lines
    void readAndParseESP();

    const char* _ssid;
    const char* _password;
    uint16_t    _port;
    Stream*     _serial;             // ESP AT serial (owned by core 1)
    bool        _ready;              // true after begin() succeeded


    // Connection state
    bool        _clientConnected;
    int         _connectionId;
    bool        _newClientConnected;
    critical_section_t _connCrit;     // protects connection flags and command

    // Command handling
    bool        _hasCommand;
    String      _pendingCommand;
    bool        _promptSeen;
    // Send queue and state machine
    StringQueue _sendQueue;
    SendState   _sendState;
    String      _currentSendLine;
    unsigned long _stateStartTime;


    bool _waitingForOkSend;
    bool _okSendReceived;

    // Line buffer for incoming ESP messages
    String      _lineBuffer;
};
