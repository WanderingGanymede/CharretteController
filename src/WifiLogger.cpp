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
      _serial(nullptr), _ready(false), _clientConnected(false), _connectionId(-1),
      _newClientConnected(false), _hasCommand(false), _pendingCommand("")
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
// sendAT()  — flush, send command, wait for expected response
// ---------------------------------------------------------------------------
bool WifiLogger::sendAT(const char* cmd, const char* expect, unsigned long timeout)
{
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
    _serial          = &espSerial;
    _ready           = false;
    _clientConnected = false;

    // Step 1: disable echo
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

    // Step 4: configure the AP (SSID, password, channel 6, WPA2)
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
    Serial.print(F("[WifiLogger] Done. Connect laptop to '"));
    Serial.print(_ssid);
    Serial.println(F("' then run:"));
    Serial.println(F("[WifiLogger]   python3 client.py  or  python3 graph.py"));
    Serial.print(F("[WifiLogger] Server: 192.168.4.1:"));
    Serial.println(_port);

    return true;
}

// ---------------------------------------------------------------------------
// update()  — parse unsolicited ESP messages, non-blocking.
//             Never calls log() — that would block the stream for up to 2 s
//             and swallow incoming +IPD bytes.
// ---------------------------------------------------------------------------
void WifiLogger::update()
{
    if (!_ready || !_serial)
        return;

    while (_serial->available())
    {
        char c = (char)_serial->read();

        if (c == '\n')
        {
            _lineBuffer.trim();

            if (_lineBuffer.length() > 0)
            {
                // "+IPD,<id>,<len>:<data>" — incoming TCP data from the client
                if (_lineBuffer.startsWith("+IPD,"))
                {
                    Serial.print(F("[WifiLogger] IPD raw: "));
                    Serial.println(_lineBuffer);

                    int colonIdx = _lineBuffer.indexOf(':');
                    if (colonIdx != -1)
                    {
                        String data = _lineBuffer.substring(colonIdx + 1);
                        data.trim();
                        if (data.length() > 0)
                        {
                            _pendingCommand = data;
                            _hasCommand     = true;
                            Serial.print(F("[WifiLogger] CMD queued: '"));
                            Serial.print(_pendingCommand);
                            Serial.println(F("'"));
                        }
                        else
                        {
                            Serial.println(F("[WifiLogger] IPD: empty payload, ignoring"));
                        }
                    }
                    else
                    {
                        Serial.println(F("[WifiLogger] IPD: no colon found, malformed?"));
                    }
                }
                // "0,CONNECT"
                else if (_lineBuffer.indexOf("CONNECT") != -1 &&
                         _lineBuffer.indexOf("DISCONNECT") == -1)
                {
                    int newId = -1;
                    if (isDigit(_lineBuffer.charAt(0)))
                        newId = _lineBuffer.charAt(0) - '0';

                    // Close the previous connection if a different one was active
                    if (_clientConnected && _connectionId >= 0 && newId != _connectionId)
                    {
                        Serial.print(F("[WifiLogger] New client (id="));
                        Serial.print(newId);
                        Serial.print(F(") replacing existing (id="));
                        Serial.print(_connectionId);
                        Serial.println(F(") — closing old connection"));
                        String closeCmd = String(F("AT+CIPCLOSE=")) + _connectionId;
                        _serial->println(closeCmd);
                        delay(150);
                    }

                    _connectionId       = newId;
                    _clientConnected    = true;
                    _newClientConnected = true;   // caller sends header via newClientConnected()
                    Serial.print(F("[WifiLogger] Client connected, id="));
                    Serial.println(_connectionId);
                }
                // "0,CLOSED"
                else if (_lineBuffer.indexOf("CLOSED") != -1)
                {
                    int closedId = -1;
                    if (isDigit(_lineBuffer.charAt(0)))
                        closedId = _lineBuffer.charAt(0) - '0';

                    if (closedId == _connectionId)
                    {
                        _clientConnected = false;
                        _connectionId    = -1;
                        Serial.println(F("[WifiLogger] Active client disconnected"));
                    }
                    else
                    {
                        Serial.print(F("[WifiLogger] Connection "));
                        Serial.print(closedId);
                        Serial.println(F(" closed (not active, ignoring)"));
                    }
                }
                else
                {
                    Serial.print(F("[WifiLogger] ESP: "));
                    Serial.println(_lineBuffer);
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
// log()  — reliable send for commands, headers, GET ALL responses.
//          Drains stale bytes first so the ESP is always in a clean state.
// ---------------------------------------------------------------------------
void WifiLogger::log(const String& line)
{
    if (!_ready || !_clientConnected || !_serial)
        return;

    // Drain any bytes left from previous logStream() calls (e.g. SEND OK)
    // so we don't confuse the AT state machine.
    while (_serial->available())
        _serial->read();

    String toSend = line + "\r\n";
    uint16_t len  = toSend.length();

    _serial->print(F("AT+CIPSEND="));
    _serial->print(_connectionId);
    _serial->print(",");
    _serial->println(len);

    // Wait for '>' prompt (max 500 ms)
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
        // Wait for SEND OK before returning so the next call finds a clean state.
        if (!waitFor("SEND OK", 2000))
            Serial.println(F("[WifiLogger] log(): SEND OK timeout"));
    }
    else
    {
        _clientConnected = false;
        Serial.println(F("[WifiLogger] log(): no '>' in 500 ms — client disconnected"));
    }
}

// ---------------------------------------------------------------------------
// logStream()  — high-frequency streaming send (e.g. every sensor sample).
//               Drains stale bytes first, waits up to 300 ms for '>',
//               then waits up to 150 ms for SEND OK.
//               Does NOT set _clientConnected = false on timeout — a missed
//               sample is harmless, a false disconnect kills all streaming.
// ---------------------------------------------------------------------------
void WifiLogger::logStream(const String& line)
{
    if (!_ready || !_clientConnected || !_serial)
        return;

    // Drain any stale bytes (SEND OK from the previous call)
    while (_serial->available())
        _serial->read();

    String toSend = line + "\r\n";
    uint16_t len  = toSend.length();

    _serial->print(F("AT+CIPSEND="));
    _serial->print(_connectionId);
    _serial->print(",");
    _serial->println(len);

    // Wait for '>' — allow up to 300 ms
    unsigned long start = millis();
    bool gotPrompt = false;
    while (millis() - start < 300)
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
        // Short SEND OK wait to leave the ESP in a clean state.
        // Don't disconnect on timeout — just move on to the next sample.
        waitFor("SEND OK", 150);
    }
    else
    {
        // Skip this sample rather than killing the stream permanently.
        Serial.println(F("[WifiLogger] logStream(): no '>' in 300 ms, skipping sample"));
    }
}

// ---------------------------------------------------------------------------
// isClientConnected()
// ---------------------------------------------------------------------------
bool WifiLogger::isClientConnected()
{
    return _clientConnected;
}

// ---------------------------------------------------------------------------
// newClientConnected() — returns true once after each new connection.
//                        The caller sends the CSV header via log().
// ---------------------------------------------------------------------------
bool WifiLogger::newClientConnected()
{
    if (_newClientConnected)
    {
        _newClientConnected = false;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// hasCommand()
// ---------------------------------------------------------------------------
bool WifiLogger::hasCommand()
{
    return _hasCommand;
}

// ---------------------------------------------------------------------------
// getCommand() — return the pending command and clear the flag
// ---------------------------------------------------------------------------
String WifiLogger::getCommand()
{
    _hasCommand     = false;
    String cmd      = _pendingCommand;
    _pendingCommand = "";
    return cmd;
}