#include "AsyncWifiLogger.h"
#include "main.h"
#include "pico/multicore.h"
static mutex_t serial_mtx;

#define LOG_ASYNC(msg) do { mutex_enter_blocking(&serial_mtx); Serial.print(msg); mutex_exit(&serial_mtx); } while(0)
#define LOGLN_ASYNC(msg) do { mutex_enter_blocking(&serial_mtx); Serial.println(msg); mutex_exit(&serial_mtx); } while(0)
const char* const AsyncWifiLogger::CSV_HEADER =
    "timestamp_ms,state,weight_kg,speed_kmh,motor_A,"
    "brake_mapped,brake_A,rpm,voltage_V,current_A,duty,temp_mosfet_C";

// ---------------------------------------------------------------------------
// StringQueue implementation
// ---------------------------------------------------------------------------
AsyncWifiLogger::StringQueue::StringQueue()
{
    critical_section_init(&crit);
}

void AsyncWifiLogger::StringQueue::clear() {
    critical_section_enter_blocking(&crit);
    head = tail = 0;
    critical_section_exit(&crit);
}        // true if an item was popped
bool AsyncWifiLogger::StringQueue::push(const String& s)
{
    critical_section_enter_blocking(&crit);
    int next = (head + 1) % QSIZE;
    if (next == tail) {
        critical_section_exit(&crit);
        return false;
    }
    buffer[head] = s;
    head = next;
    critical_section_exit(&crit);
    return true;
}

bool AsyncWifiLogger::StringQueue::pop(String& s)
{
    critical_section_enter_blocking(&crit);
    if (head == tail) {
        critical_section_exit(&crit);
        return false;
    }
    s = buffer[tail];
    tail = (tail + 1) % QSIZE;
    critical_section_exit(&crit);
    return true;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
AsyncWifiLogger::AsyncWifiLogger(const char* ssid, const char* password, uint16_t port)
    : _ssid(ssid), _password(password), _port(port),
      _serial(nullptr), _ready(false),
      _clientConnected(false), _connectionId(-1), _newClientConnected(false),
      _hasCommand(false), _pendingCommand(""),
      _sendState(STATE_IDLE), _currentSendLine(""), _stateStartTime(0),
      _lineBuffer(""), _promptSeen(false)
{
    critical_section_init(&_connCrit);
}

// ---------------------------------------------------------------------------
// sendAT()  — non‑blocking, just writes the command to the ESP
// ---------------------------------------------------------------------------
void AsyncWifiLogger::sendAT(const char* cmd)
{
    while (_serial->available())
        _serial->read();
    _serial->println(cmd);
    Serial.print(F("[AsyncWifi] >> "));
    Serial.println(cmd);
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------
bool AsyncWifiLogger::begin(Stream& espSerial)
{
    _serial = &espSerial;
    _ready = false;

    Serial.println(F("[AsyncWifi] Step 1: ATE0"));
    _serial->println("ATE0");
    delay(500);
    while (_serial->available()) _serial->read();

    Serial.println(F("[AsyncWifi] Step 2: AT"));
    _serial->println("AT");
    delay(500);
    while (_serial->available()) _serial->read();

    Serial.println(F("[AsyncWifi] Step 3: AT+CWMODE=2"));
    _serial->println("AT+CWMODE=2");
    delay(1000);
    while (_serial->available()) _serial->read();

    Serial.println(F("[AsyncWifi] Step 4: AT+CWSAP"));
    String apCmd = String(F("AT+CWSAP=\"")) + _ssid
                 + F("\",\"") + _password
                 + F("\",6,3");
    _serial->println(apCmd);
    delay(2000);
    while (_serial->available()) _serial->read();

    Serial.println(F("[AsyncWifi] Step 5: AT+CIPMUX=1"));
    _serial->println("AT+CIPMUX=1");
    delay(500);
    while (_serial->available()) _serial->read();

    Serial.println(F("[AsyncWifi] Step 6: AT+CIPSERVER"));
    String serverCmd = String(F("AT+CIPSERVER=1,")) + _port;
    _serial->println(serverCmd);
    delay(500);
    while (_serial->available()) _serial->read();

    _ready = true;
    Serial.print(F("[AsyncWifi] AP started. Connect to '"));
    Serial.print(_ssid);
    Serial.println(F("' then run client."));
    return true;
}

// ---------------------------------------------------------------------------
// readAndParseESP()  — called from update() on core 1
// ---------------------------------------------------------------------------
void AsyncWifiLogger::readAndParseESP()
{
    //Serial.print("read and parse");
    if (!_serial) return;

    while (_serial->available()) {
        char c = (char)_serial->read();

             // If we are in WAIT_PROMPT state, we need to detect '>' immediately,
        // even if it's not followed by a newline.
        if (_sendState == STATE_WAIT_PROMPT && c == '>') {
            _promptSeen = true;
            // Don't add to _lineBuffer, just set flag.
            continue;
        }

        if (c == '\n') {
            _lineBuffer.trim();
            if (_lineBuffer.length() > 0) {
                // Debug: uncomment to see every ESP line
                 //Serial.print(F("[ESP] ")); Serial.println(_lineBuffer);

                // Check for connection events
                if (_lineBuffer.startsWith("+IPD,")) {
                    int colonIdx = _lineBuffer.indexOf(':');
                    if (colonIdx != -1) {
                        String data = _lineBuffer.substring(colonIdx + 1);
                        data.trim();
                        if (data.length() > 0) {
                            critical_section_enter_blocking(&_connCrit);
                            _pendingCommand = data;
                            _hasCommand = true;
                            critical_section_exit(&_connCrit);
                            Serial.print(F("[AsyncWifi] CMD queued: '"));
                            Serial.print(data);
                            Serial.println(F("'"));
                        }
                    }
                }
                else if (_lineBuffer.indexOf("CONNECT") != -1 && _lineBuffer.indexOf("DISCONNECT") == -1) {
                    int newId = -1;
                    if (isDigit(_lineBuffer.charAt(0)))
                        newId = _lineBuffer.charAt(0) - '0';

                    if (_clientConnected && _connectionId >= 0 && newId != _connectionId) {
                        String closeCmd = String(F("AT+CIPCLOSE=")) + _connectionId;
                        _serial->println(closeCmd);
                    }

                    critical_section_enter_blocking(&_connCrit);
                    _connectionId = newId;
                    _clientConnected = true;
                    _newClientConnected = true;
                    critical_section_exit(&_connCrit);
                    Serial.print(F("[AsyncWifi] Client connected, id="));
                    Serial.println(_connectionId);
                }
                else if (_lineBuffer.indexOf("CLOSED") != -1) {
                    int closedId = -1;
                    if (isDigit(_lineBuffer.charAt(0)))
                        closedId = _lineBuffer.charAt(0) - '0';
                    if (closedId == _connectionId) {
                        critical_section_enter_blocking(&_connCrit);
                        _clientConnected = false;
                        _connectionId = -1;
                        critical_section_exit(&_connCrit);
                        _sendQueue.clear();
                        Serial.println(F("[AsyncWifi] Client disconnected"));
                    }
                }
                // For SEND OK detection in WAIT_OK state
                else if (_sendState == STATE_WAIT_OK && _lineBuffer.indexOf("SEND OK") != -1) {
                    _okSendReceived =true;
                    //Serial.println("oksendrecevied set to true");
                }
            }
            _lineBuffer = "";
        } else if (c != '\r') {
            _lineBuffer += c;
        }
    }
}

// ---------------------------------------------------------------------------
// processSendState()  — advance the AT+CIPSEND state machine
// ---------------------------------------------------------------------------
void AsyncWifiLogger::processSendState()
{
    switch (_sendState) {
        case STATE_IDLE:
            // If a line is queued and client is connected, start the sequence
            if (_clientConnected && _connectionId >= 0 && _sendQueue.pop(_currentSendLine)) {
                //Serial.println("[ASYNC] popping send Queue");
                //Serial.println(_sendQueue.size())
                // Drain any stale bytes before starting
                while (_serial->available()) _serial->read();
                _serial->print(F("AT+CIPSEND="));
                _serial->print(_connectionId);
                _serial->print(",");
                // +2 for \r\n we'll add when sending
                _serial->println(_currentSendLine.length() + 2);
                _stateStartTime = millis();
                _sendState = STATE_WAIT_PROMPT;
                _promptSeen = false;
                _lineBuffer = "";
               // Serial.print(F("[AsyncWifi] AT+CIPSEND queued, len="));
                //Serial.println(_currentSendLine.length() + 2);
            }
            break;

        case STATE_WAIT_PROMPT:
            // Check if we've seen the '>' character (set by readAndParseESP)
            if (_promptSeen) {
                // Send the actual data
                _serial->print(_currentSendLine);
                _serial->print("\r\n");
                _stateStartTime = millis();
                _sendState = STATE_WAIT_OK;
                _lineBuffer = "";
                _promptSeen = false;
                _waitingForOkSend=true;
                _okSendReceived=false;
                //Serial.println(F("[AsyncWifi] Prompt seen, data sent"));
            } else if (millis() - _stateStartTime > 1000) {
                // Timeout waiting for prompt
                Serial.println(F("[AsyncWifi] AT+CIPSEND prompt timeout"));
                _sendState = STATE_IDLE;
                _promptSeen = false;
            }
            break;

        case STATE_WAIT_OK:

            // Look for "SEND OK" in _lineBuffer (processed on newline)
            if (_okSendReceived) {
                _sendState = STATE_IDLE;
                _okSendReceived=false;
                _waitingForOkSend=false;
                _lineBuffer = "";
                //Serial.println(F("[AsyncWifi] SEND OK received in "));
                //Serial.println(millis()-_stateStartTime);

            } else if (millis() - _stateStartTime > 30) {
             //   Serial.println(F("[AsyncWifi] SEND OK timeout"));
                _sendState = STATE_IDLE;
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// update()  — core 1 loop
// ---------------------------------------------------------------------------
void AsyncWifiLogger::update()
{
    if (!_ready || !_serial) return;

    readAndParseESP();
    processSendState();
}

// ---------------------------------------------------------------------------
// sendAsync()  — called from core 0
// ---------------------------------------------------------------------------
void AsyncWifiLogger::sendAsync(const String& line)
{
    // Only queue if a client is connected
    if (!isClientConnected()) {
        return;  // silently drop
    }

    //Serial.println("Starting async send to client");
    //Serial.println(millis());
    bool ok = _sendQueue.push(line);
    if (!ok) {
        static unsigned long lastWarn = 0;
        if (millis() - lastWarn > 2000) {
            Serial.println(F("[Async] WARN: send queue full (client connected)"));
            lastWarn = millis();
        }
    }
}

// ---------------------------------------------------------------------------
// newClientConnected()  — thread‑safe
// ---------------------------------------------------------------------------
bool AsyncWifiLogger::newClientConnected()
{
    critical_section_enter_blocking(&_connCrit);
    bool ret = _newClientConnected;
    _newClientConnected = false;
    critical_section_exit(&_connCrit);
    return ret;
}

// ---------------------------------------------------------------------------
// hasCommand()  — thread‑safe
// ---------------------------------------------------------------------------
bool AsyncWifiLogger::hasCommand()
{
    critical_section_enter_blocking(&_connCrit);
    bool ret = _hasCommand;
    critical_section_exit(&_connCrit);
    return ret;
}

// ---------------------------------------------------------------------------
// getCommand()  — thread‑safe
// ---------------------------------------------------------------------------
String AsyncWifiLogger::getCommand()
{
    critical_section_enter_blocking(&_connCrit);
    _hasCommand = false;
    String cmd = _pendingCommand;
    _pendingCommand = "";
    critical_section_exit(&_connCrit);
    return cmd;
}

// ---------------------------------------------------------------------------
// isClientConnected()  — thread‑safe
// ---------------------------------------------------------------------------
bool AsyncWifiLogger::isClientConnected()
{
    critical_section_enter_blocking(&_connCrit);
    bool ret = _clientConnected;
    critical_section_exit(&_connCrit);
    return ret;
}
