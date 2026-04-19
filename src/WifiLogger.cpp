#include "WifiLogger.h"
#include <Arduino.h>

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
    : _ssid(ssid), _password(password), _port(port), _server(port), _ready(false)
{
}

// ---------------------------------------------------------------------------
// begin()  — call once in setup() after Serial1.begin()
// ---------------------------------------------------------------------------
// Helper: print a human-readable label for a WiFiEspAT status code
static void printWifiStatus(int status)
{
    switch (status)
    {
        case WL_NO_MODULE:      Serial.print(F("WL_NO_MODULE (255)"));      break;
        case WL_IDLE_STATUS:    Serial.print(F("WL_IDLE_STATUS (0)"));      break;
        case WL_NO_SSID_AVAIL:  Serial.print(F("WL_NO_SSID_AVAIL (1)"));    break;
        case WL_CONNECTED:      Serial.print(F("WL_CONNECTED (3)"));        break;
        case WL_CONNECT_FAILED: Serial.print(F("WL_CONNECT_FAILED (4)"));   break;
        case WL_DISCONNECTED:   Serial.print(F("WL_DISCONNECTED (6)"));     break;
        case WL_AP_LISTENING:   Serial.print(F("WL_AP_LISTENING (7)"));     break;
        case WL_AP_CONNECTED:   Serial.print(F("WL_AP_CONNECTED (8)"));     break;
        case WL_AP_FAILED:      Serial.print(F("WL_AP_FAILED (9)"));        break;
        default:
            Serial.print(F("UNKNOWN ("));
            Serial.print(status);
            Serial.print(F(")"));
            break;
    }
}

bool WifiLogger::begin(Stream& espSerial)
{
    _ready = false;

    // ---- Step 1: hand the serial port to the WiFiEspAT stack ----
    Serial.println(F("[WifiLogger] Step 1: WiFi.init()..."));
    WiFi.init(espSerial);

    int status = WiFi.status();
    Serial.print(F("[WifiLogger]   status after init: "));
    printWifiStatus(status);
    Serial.println();

    if (status == WL_NO_MODULE)
    {
        Serial.println(F("[WifiLogger] ERROR: ESP module not found. Check wiring on Serial1 (RX=1, TX=0) and baud rate."));
        return false;
    }

    // ---- Step 2: firmware version (sanity check) ----
    Serial.print(F("[WifiLogger] Step 2: firmware version: "));
    Serial.println(WiFi.firmwareVersion());

    // ---- Step 3: don't persist config to ESP flash ----
    Serial.println(F("[WifiLogger] Step 3: setPersistent(false)..."));
    WiFi.setPersistent(false);

    // ---- Step 4: clear any saved connection / mode ----
    Serial.println(F("[WifiLogger] Step 4: WiFi.disconnect()..."));
    WiFi.disconnect();
    delay(300);

    status = WiFi.status();
    Serial.print(F("[WifiLogger]   status after disconnect: "));
    printWifiStatus(status);
    Serial.println();

    // ---- Step 5: bring up the SoftAP ----
    Serial.print(F("[WifiLogger] Step 5: beginAP('"));
    Serial.print(_ssid);
    Serial.print(F("', password="));
    Serial.print((_password && strlen(_password) >= 8) ? F("set") : F("NONE/too short"));
    Serial.println(F(")..."));

    if (_password && strlen(_password) >= 8)
    {
        status = WiFi.beginAP(_ssid, _password);
    }
    else
    {
        status = WiFi.beginAP(_ssid);
    }

    Serial.print(F("[WifiLogger]   status after beginAP: "));
    printWifiStatus(status);
    Serial.println();

    if (status != WL_AP_LISTENING)
    {
        Serial.println(F("[WifiLogger] ERROR: SoftAP did not reach WL_AP_LISTENING. Hotspot will NOT appear."));
        return false;
    }

    // ---- Step 6: start TCP server ----
    Serial.print(F("[WifiLogger] Step 6: starting TCP server on port "));
    Serial.print(_port);
    Serial.println(F("..."));
    _server.begin();

    _ready = true;

    Serial.print(F("[WifiLogger] Done. AP IP: "));
    Serial.println(WiFi.localIP());
    Serial.println(F("[WifiLogger] Connect laptop to the hotspot and run:"));
    Serial.println(F("  python3 -c \"import socket; s=socket.create_connection(('192.168.4.1',8888)); [print(l,end='') for l in s.makefile()]\""));

    return true;
}

// ---------------------------------------------------------------------------
// update()  — call every loop(), non-blocking
// ---------------------------------------------------------------------------
void WifiLogger::update()
{
    if (!_ready)
        return;

    // Drop client if it has gone away
    if (_client && !_client.connected())
    {
        _client.stop();
        Serial.println(F("[WifiLogger] Client disconnected"));
    }

    // Accept a new client if the slot is free
    if (!_client)
    {
        WiFiClient incoming = _server.accept();
        if (incoming)
        {
            _client = incoming;
            Serial.println(F("[WifiLogger] Client connected"));
            // Send the CSV header so the receiver knows the column layout
            _client.println(CSV_HEADER);
        }
    }
}

// ---------------------------------------------------------------------------
// log()  — send one CSV line; non-blocking, silent no-op if not connected
// ---------------------------------------------------------------------------
void WifiLogger::log(const String& line)
{
    if (!_ready || !_client || !_client.connected())
        return;

    _client.println(line);
}

// ---------------------------------------------------------------------------
// isClientConnected()
// ---------------------------------------------------------------------------
bool WifiLogger::isClientConnected()
{
    return _client && _client.connected();
}
