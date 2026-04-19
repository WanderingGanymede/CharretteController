#include "WifiLogger.h"

// ---------------------------------------------------------------------------
// CSV header — column order must match the log() call in main.cpp
// ---------------------------------------------------------------------------
const char* const WifiLogger::CSV_HEADER =
    "timestamp_ms,state,weight_kg,speed_kmh,motor_A,"
    "brake_mapped,brake_A,rpm,voltage_V,current_A,duty,temp_mosfet_C";

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
WifiLogger::WifiLogger(const char* ssid, const char* password, uint16_t port)
    : _ssid(ssid), _password(password), _port(port),
      _serial(nullptr), _ready(false), _clientConnected(false), _connectionId(0)
{
}

// ---------------------------------------------------------------------------
// waitFor()  — read ESP output until target string appears or timeout
// ---------------------------------------------------------------------------
bool WifiLogger::waitFor(const char* target, unsigned long timeout)
{
    String resp = "";
    unsigned long start = millis();
    while (millis() - start < timeout)
    {
        while (_serial->available())
        {
            char c = (char)_serial->read();
            resp += c;
            if (resp.indexOf(target) != -1)
                return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// sendAT()  — flush, send command, wait for expected response, log result
// ---------------------------------------------------------------------------
bool WifiLogger::sendAT(const char* cmd, const char* expect, unsigned long timeout)
{
    // Flush any leftover bytes before sending
    while (_serial->available())
        _serial->read();

    Serial.print(F("[WifiLogger] >> "));
    Serial.println(cmd);

    _serial->println(cmd);

    bool ok = waitFor(expect, timeout);

    Serial.println(ok ? F("[WifiLogger]    OK") : F("[WifiLogger]    TIMEOUT/FAIL"));
    return ok;
}

// ---------------------------------------------------------------------------
// begin()  — mirrors the setup() sequence from examplemain.cpp
// ---------------------------------------------------------------------------
bool WifiLogger::begin(Stream& espSerial)
{
    _serial = &espSerial;
    _ready  = false;
    _clientConnected = false;

    // Step 1: disable echo — send raw, don't wait for OK (echo may still be
    //         on at this point so the response is unpredictable)
    Serial.println(F("[WifiLogger] Step 1: ATE0 (disable echo)..."));
    _serial->println("ATE0");
    delay(500);
    Serial.println(F("[WifiLogger] ATE0 sent"));

    // Step 2: confirm the module is alive
    Serial.println(F("[WifiLogger] Step 2: AT..."));
    if (!sendAT("AT"))
    {
        Serial.println(F("[WifiLogger] ERROR: no response to AT. Check wiring and baud rate."));
        return false;
    }

    // Step 3: SoftAP-only mode
    Serial.println(F("[WifiLogger] Step 3: AT+CWMODE=2 (SoftAP)..."));
    if (!sendAT("AT+CWMODE=2", "OK", 3000))
        Serial.println(F("[WifiLogger] WARNING: CWMODE=2 failed, continuing anyway"));
    delay(500);

    // Step 4: configure the AP  (SSID, password, channel 6, WPA2)
    Serial.println(F("[WifiLogger] Step 4: AT+CWSAP..."));
    String apCmd = String(F("AT+CWSAP=\"")) + _ssid
                 + F("\",\"") + _password
                 + F("\",6,3");
    if (!sendAT(apCmd.c_str(), "OK", 5000))
        Serial.println(F("[WifiLogger] WARNING: CWSAP failed"));
    delay(500);

    // Step 5: allow multiple simultaneous connections (required for CIPSERVER)
    Serial.println(F("[WifiLogger] Step 5: AT+CIPMUX=1..."));
    if (!sendAT("AT+CIPMUX=1"))
        Serial.println(F("[WifiLogger] WARNING: CIPMUX=1 failed"));
    delay(500);

    // Step 6: start TCP server on requested port
    Serial.println(F("[WifiLogger] Step 6: AT+CIPSERVER..."));
    String serverCmd = String(F("AT+CIPSERVER=1,")) + _port;
    if (!sendAT(serverCmd.c_str()))
        Serial.println(F("[WifiLogger] WARNING: CIPSERVER failed"));
    delay(500);

    _ready = true;
    Serial.print(F("[WifiLogger] Done. Connect to '"));
    Serial.print(_ssid);
    Serial.print(F("' (192.168.4.1:"));
    Serial.print(_port);
    Serial.println(F(")"));
    Serial.println(F("[WifiLogger] python3 -c \"import socket; s=socket.create_connection(('192.168.4.1',8888)); [print(l,end='') for l in s.makefile()]\""));

    return true;
}

// ---------------------------------------------------------------------------
// update()  — parse unsolicited ESP messages to track connect / disconnect
//             Call every loop(). Non-blocking.
// ---------------------------------------------------------------------------
void WifiLogger::update()
{
    if (!_ready || !_serial)
        return;

    // Accumulate chars into _lineBuffer; process on each newline
    while (_serial->available())
    {
        char c = (char)_serial->read();

        if (c == '\n')
        {
            _lineBuffer.trim();

            if (_lineBuffer.length() > 0)
            {
                // "0,CONNECT" — a client connected on connection id 0
                if (_lineBuffer.indexOf("CONNECT") != -1 &&
                    _lineBuffer.indexOf("DISCONNECT") == -1)
                {
                    // Extract the connection id from the first character
                    if (isDigit(_lineBuffer.charAt(0)))
                        _connectionId = _lineBuffer.charAt(0) - '0';

                    _clientConnected = true;
                    Serial.print(F("[WifiLogger] Client connected, id="));
                    Serial.println(_connectionId);

                    // Send CSV header to the new client
                    log(String(CSV_HEADER));
                }
                // "0,CLOSED" — the client disconnected
                else if (_lineBuffer.indexOf("CLOSED") != -1)
                {
                    _clientConnected = false;
                    Serial.println(F("[WifiLogger] Client disconnected"));
                }
            }

            _lineBuffer = "";
        }
        else if (c != '\r')
        {
            _lineBuffer += c;
        }
    }
}

// ---------------------------------------------------------------------------
// log()  — send one CSV line via AT+CIPSEND, mirroring examplemain.cpp loop()
// ---------------------------------------------------------------------------
void WifiLogger::log(const String& line)
{
    if (!_ready || !_clientConnected || !_serial)
        return;

    // AT+CIPSEND requires \r\n in the byte count
    String toSend = line + "\r\n";
    uint16_t len  = toSend.length();

    _serial->print(F("AT+CIPSEND="));
    _serial->print(_connectionId);
    _serial->print(",");
    _serial->println(len);

    // Wait for the '>' prompt before sending payload (max 500 ms)
    unsigned long start = millis();
    bool gotPrompt = false;
    while (millis() - start < 500)
    {
        while (_serial->available())
        {
            if (_serial->read() == '>')
            {
                gotPrompt = true;
                break;
            }
        }
        if (gotPrompt) break;
    }

    if (gotPrompt)
    {
        _serial->print(toSend);
    }
    else
    {
        // No prompt — assume the client dropped
        _clientConnected = false;
        Serial.println(F("[WifiLogger] log(): no '>' prompt, client assumed disconnected"));
    }
}

// ---------------------------------------------------------------------------
// isClientConnected()
// ---------------------------------------------------------------------------
bool WifiLogger::isClientConnected()
{
    return _clientConnected;
}