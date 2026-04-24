// ...existing code...
// #include <WiFiEspAT.h>
#include "AsyncWifiLogger.h"
#include "WifiLogger.h"
#include "pico/multicore.h"

// Replace the old WifiLogger instance with AsyncWifiLogger

#include "StrengthSensor.h"
#include "Vesc.h"
#include "config.h"
#include "utils.h"
// ...existing code...
#include "BikeDisplay.h"
#include "main.h"
// ...existing code...

const int CALIBRATE_BUTTON_PIN = 18; // Use pin 4 for calibration button
#include "HX711_ADC.h"
#include "LoadCell.h"
#include <Arduino.h>
// ...existing code...
#include "InputControl.h"
#include <Chrono.h>
#include <PID_v1.h>
#include <SoftFilters.h>
// put function declarations here:
int myFunction(int, int);

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// WiFi logger — SoftAP + TCP server via ESP8285 on Serial1
// WifiLogger wifiLogger("PicoTrailer", "12345678");

AsyncWifiLogger wifiLogger("PicoTrailer", "12345678");
// When true: stream every raw sensor sample (for noise analysis).
// When false: send the full CSV telemetry line every 200 ms instead.
bool wifiStreamSensor = false;

// Sensor stream buffer — samples are accumulated here and flushed as one
// AT+CIPSEND every SENSOR_FLUSH_MS milliseconds.
static String sensorBuffer = "";
static unsigned long lastSensorFlush = 0;
const unsigned long SENSOR_FLUSH_MS = 500;

float minLoadCellValue = 0;
float maxLoadCellValue = 0;

/*
 * Paramètres sur lesquels influer:
 *
 *
 *
 *
 *
 */

/////////////////////////////////////////////////
/////////////// Debug ///////////////////////////
/////////////////////////////////////////////////

bool debugPython =
    0; // pour envoyer les données au format attendu par le script python
bool debug = 0; // pour envoyer les données de debug au format texte dans le
                // moniteur série
bool debugCsv =
    0; // pour envoyer les données de debug au format csv dans le moniteur série
bool debugMotor = 0; // si aucun moteur n'est branché, pour le simuler
bool debugFrein = 0; // si aucun interrupteur n'est branché, sur le frein à
                     // inertie. Inutilisé actuellement
bool debugCapteur =
    0;               // si aucun capteur de force n'est branché, pour le simuler
bool debugOther = 0; // utilisé à des fins de tests.
bool old = 0;    // pour assurer la compatibilité avec une ancienne version de
                 // carte électronique. A garder à 0.
int csvIter = 0; // compteur d'itération pour l'affichage csv

/////////////////////////////////////////////////
/////////////// capteur force ///////////////////
/////////////////////////////////////////////////

const int HX711_dout = 20; // mcu > HX711 dout pin
const int HX711_sck = 21;  // mcu > HX711 sck pin
double capteur_offset = 838.6 / 2.5f;
float capteur_calibration = 25000;
StrengthSensor capteur(HX711_dout, HX711_sck, capteur_offset,
                       capteur_calibration, 0.5f);
// pour calculer la dérivée du capteur de force. Inutilisé
DifferentialFilter<double, unsigned long> diffCapteur;
Reading<double, unsigned long> rCapteur;
Reading<Differential<double>, unsigned long> dCapteur;
// median filter window of 3
static float med_buf[3] = {0, 0, 0};
static uint8_t med_idx = 0;
static bool med_full = false;
double valeurCapteurInstant;

MovingAverageFilter<double, double> valeurCapteurFiltered(10);
double valeurCapteurMoyenne;
bool newDataReady = 0;
bool capteurInitialise = 0;
Chrono resetOffsetChrono; // Chrono pour ?
int resetOffsetIter;      // compteur pour compter le nombre d'aller retour sur
                          // l'interrupteur brake
MovingAverageFilter<double, double>
    movingOffset(16); // moyenne lissée sur 16 valeurs
double newOffset, rawValue;

bool capteurResponsive = false;
long capteurTimeout = 2000;
const int calVal_eepromAdress = 0;

///

/////////////////////////////////////////////////
/////////////////// VESC /////////////////////////
/////////////////////////////////////////////////
const int serial2RX = 5;
const int serial2TX = 4;
Vesc vesc;
bool vescResponsive = false;
// VESC telemetry storage
struct VescTelemetry {
  float rpm = 0;
  float voltage = 0;
  float current = 0;
  float duty = 0;
  float tempMosfet = 0;
  float tempMotor = 0;
  float ampHours = 0;
  float ampHoursCharged = 0;
  float wattHours = 0;
  float wattHoursCharged = 0;
  long tachometer = 0;
  long tachometerAbs = 0;
};
VescTelemetry vescTelemetry;
const int VESC_TELEM_SIZE = 78; // adjust after measuring
static unsigned long lastVescUpdate = 0;
const unsigned long vescUpdateInterval = 50; // ms

/////////////////////////////////////////////////
/////////////// Display ///////////////////////////
/////////////////////////////////////////////////

BikeDisplay bikeDisplay;
const int sdaWire1 = 10;
const int sclWire1 = 11;
/////////////////////////////////////////////////
///////////////// PID ///////////////////////////
/////////////////////////////////////////////////

/*
 * Pour lier l'information du capteur de force au signal PWM envoyé dans la
 * gachette, on utilise un PID. https://playground.arduino.cc/Code/PIDLibrary/
 */

// Paramètres à changer:

double K1[3] = {1, 3, 0.125}; // boost, mode 0 pour les led
double K2[3] = {1, 2, 0.1};   // marche, mode 1 pour les led
float betaTab[2] = {-4, -3};
float gammaTab[2] = {-8, -6};
double consigneCapteurTab[2] = {-0.1, 0.0};

double sortieMoteurAccel; // output
double consigneCapteur =
    consigneCapteurTab[1]; // setpoint, valeur visée par le PID comme valeur de
                           // capteur. les valeurs minimales et maximales pour
                           // le PWM de la gachette.
// 110 a été trouvée expérimentalement avec une batterie 48V et un contrôleur
// Ozo. En deça la roue ne tourne pas. ces valeurs sont à tester et corriger en
// cas de changement de batterie ou contrôleur.gvgv

/// KNOBS
float minCurrent = 1.5, maxCurrent = 30;

float minBrakeCurrent = 0, maxBrakeCurrent = 50;

int pidDir = REVERSE;

PID mainPID(&valeurCapteurMoyenne, &sortieMoteurAccel, &consigneCapteur, K1[0],
            K1[1], K1[2], P_ON_E, pidDir);
// Entrée: ValeurCapteur
// Valeur asservie: SortieMoteur, qui est un PWM allant de pwmMin à pwmMax
// ConsigneCapteur: Valeur visée pour ValeurCapteur
// Kp,Ki,Kd, les paramètres dont dépendent l'asservissement du pwm
// P_ON_E, Proportionnal on Erroruuu
// REVERSE, augmenter la valeur asservie, diminuera l'entrée.uuuuuuasuuu
//
/////////////////////////////////////////////////
//////////////// Etats //////////////////////////
/////////////////////////////////////////////////

enum etats_enum {
  INITIALISATION, // 0
  ATTENTE,        // 1
  ROULE,          // 2
  STATU_QUO,      // 3
  DECCELERATION,  // 4
  FREINAGE,       // 5
  MARCHE,         // 6
  RESET_CAPTEUR   // 7
};

int etat = INITIALISATION;

static String etatsStr[] = {"INIT",  "ATT",   "ROULE",  "S_QUO",
                            "DECEL", "FREIN", "MARCHE", "RESET"};
// Ancre paramètre 2
float rollSpeedThreshold = 1;
float alpha = 1;         // seuil au dessus duquel le PID se calcule et se lance
float beta = betaTab[0]; // seuil en deça duquel on passe sur déccélération
                         // (pwm=0, pid manual)
float brakeThreshold =
    gammaTab[0]; // seuil en deça duquel on passe sur du freinage

/////////////////////////////////////////////////
//////////// Vitesse moyenne ////////////////////
/////////////////////////////////////////////////gvgv

/*
 * Stockage de la vitesse du moteur.
 * Vitesse lue grâce aux capteurs à effet hall du moteur
 * En cas de défaillance de cette information, le système n'est plus utilisable
 */

MovingAverageFilter<double, double> vitesseFiltree(4);
double vitesseMoyenne;
double vitesseInstantanee;

// in mm
float wheelDiam = 550;
// motor poles pair

int polePairs = 50;

/////////////////////////////////////////////////
/////////////// Frein à inertie /////////////////
/////////////////////////////////////////////////

/*
 * Inutilisée
 *
 * Pour ajouter de la sécurité, l'idée est de mettre un contacteur au niveau du
 * frein à inertie. Quand ça s'ouvre, la remorque commence à aller plus vite que
 * le vélo. L'information est plus basique que celle du capteur de force mais
 * permet d'ajouter une redondance d'informations. Il n'a pas été trouvé encore
 * de position correcte ou de contacteur fiable et robuste. Idée en repos, mais
 * importante à mettre en oeuvre par la suite.
 */

const int frein = debugFrein ? 4 : 5;
// Chrono chronoFrein;
/*bool freinFlag = 0;
int t1 = 300;
int t2 = 1000;
int t3 = 1500;
*/
////gachette frein

float brakeThumbThrottleThreshold =
    0.03; // Seuil au delà duquel on considère que la gachette de frein est
          // actionnée

/////////////////////////////////////////////////
///////////// Pin interrupteur debug ////////////
/////////////////////////////////////////////////

int plus = 2;
int moins = 3;
int halt = 4;
bool haltFlag = 0;

/////////////////////////////////////////////////
///////////// Pin controleur ////////////////////
/////////////////////////////////////////////////

int ctrlAlive = 12;  // sur cette pin arrive le 5V de la gachette. Cette
                     // information nous renseigne sur l'état du contrôleur.
int ctrlSwitch = 13; // pin utilisée pour allumer le relais qui activait le
                     // contrôleur. Inutilisée depuis que le relais est activé
                     // en direct par un interrupteur. A nettoyer
bool isCtrlAlive = debugMotor ? 1 : 0;

bool walkMode = 1;
bool motorBrakeMode = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
    ;
  Serial.println("###################");
  Serial.println("## version 0.1: ");
  Serial.println("## date: : ");
  Serial.println("## Boulanger ");
  Serial.println("###################");

  Serial.println(F("[setup] Input control..."));
  setupInputControl();
  Serial.println(F("[setup] Input control OK"));
  /*
    On a un nouveau boîtier de contrôle.
    Le mode est lu en fonction de analogRead sur walkPin.
      Avec un pont diviseur de tension, on a empiriquement:
        Mode 1: analogRead = 426 425
        Mode 2: analogRead = 71 72
        Mode 0: analogRead = 1023 (éteint)

  */
  // kkinterrupteur sur la carte
  Serial.println(F("[setup] Buttons pinMode..."));
  pinMode(plus, INPUT_PULLUP);
  pinMode(moins, INPUT_PULLUP);
  pinMode(halt, INPUT_PULLUP);
  Serial.println(F("[setup] Buttons OK"));

  // PID
  Serial.println(F("[setup] PID init..."));
  mainPID.SetMode(MANUAL);
  mainPID.SetOutputLimits(minCurrent, maxCurrent);
  mainPID.SetSampleTime(200);
  Serial.println(F("[setup] PID OK"));

  // Controleur
  Serial.println(F("[setup] Controller pins..."));
  pinMode(ctrlAlive, INPUT);
  pinMode(ctrlSwitch, OUTPUT);
  digitalWrite(ctrlSwitch, false);
  Serial.println(F("[setup] Controller pins OK"));

  Serial.println(F("[setup] Waiting 1s..."));
  delay(1000);
  Serial.println(F("[setup] Wire1 + display init..."));
  Wire1.setSDA(sdaWire1);
  Wire1.setSCL(sclWire1);
  Wire1.begin();
  Serial.println(F("[setup] Wire1 OK"));
  bikeDisplay.begin();
  bikeDisplay.displayMessage("Bike Display Ready");
  Serial.println(F("[setup] Display OK"));
  delay(1000);

  Serial.println(F("[setup] VESC init..."));
  initializeVesc();
  Serial.println(F("[setup] VESC init done, waiting 2s..."));
  delay(2000);
  Serial.println(F("[setup] Strength sensor init..."));
  InitializeStrengthSensor();
  Serial.println(F("[setup] Strength sensor done"));

  bikeDisplay.displayMessage("Starting Charr");
  Serial.println(F("[setup] Setup complete. Entering loop."));
  Serial.println();
  bikeDisplay.displayMessage("Starting Charr");
  Serial.println("Starting Charrette yo...");

  // multicore_launch_core1(setup1);
}

// Core 1: WiFi AT handling
// ------------------------------------------------------------
void setup1() {
  delay(2000);
  Serial.println("starting setup1");
  // Core 1 initialisation
  // Serial1.setRX(1);   // GPIO1 = UART0 RX (adjust if your wiring differs)
  // Serial1.setTX(0);   // GPIO0 = UART0 TX
  Serial1.begin(115200);

  // Give the ESP time to boot
  delay(1000);

  if (!wifiLogger.begin(Serial1)) {
    // If begin fails, we could retry or just continue (no WiFi)
    Serial.println("Core1: WiFi init failed");
  } else {

    Serial.println("Core1: WiFi init success");
  }
}

void loop1() {
  static unsigned long lastHeartbeat = 0;
  // Run the non‑blocking WiFi update
  wifiLogger.update();
  // Send the CSV header to a freshly connected client.
  // Done here rather than inside update() so that log() is never called
  // from within the serial-parsing loop (which would block update() for
  // up to 2 s and swallow incoming +IPD bytes).
  // Small delay to prevent starving core 0 (not strictly necessary but safe)

  if (wifiLogger.newClientConnected()) {
    Serial.println("New Client connected, sent csv header");
    wifiLogger.sendAsync(String(WifiLogger::CSV_HEADER));
  }

  if (wifiLogger.hasCommand()) {
    String cmd = wifiLogger.getCommand();
    Serial.print(F("[loop] WiFi command received: '"));
    Serial.print(cmd);
    Serial.println(F("'"));
    handleWifiCommand(cmd);
  }

  delay(1);
}
void loop() {

  // vitesseMoyenne = 2;
  InputControlHandler();
  static long lastCapteurUpdate = 0;
  // Call the toggle handler to check for toggle presses
  if (capteurInitialise) {
    capteur.update(&newDataReady, &valeurCapteurInstant);
    if (newDataReady) {
      capteurResponsive = true;
      lastCapteurUpdate = millis();

      // Median pre‑filter (window 3) – removes isolated spikes
      med_buf[med_idx] = valeurCapteurInstant;
      med_idx = (med_idx + 1) % 3;
      if (!med_full && med_idx == 0)
        med_full = true;

      float med_val = valeurCapteurInstant; // fallback if not yet full
      if (med_full) {
        // sort a copy of the buffer to find median
        float a = med_buf[0], b = med_buf[1], c = med_buf[2];
        if (a > b) {
          float t = a;
          a = b;
          b = t;
        }
        if (a > c) {
          float t = a;
          a = c;
          c = t;
        }
        if (b > c) {
          float t = b;
          b = c;
          c = t;
        }
        med_val = b; // the middle value
      }

      // Feed the median‑filtered value into your moving average
      valeurCapteurFiltered.push(&valeurCapteurInstant, &valeurCapteurMoyenne);
      rCapteur.value = valeurCapteurInstant * 1000;
      rCapteur.timestamp += millis() / 1000;
      diffCapteur.push(&rCapteur, &dCapteur);

      newDataReady = 0;
      // Accumulate sensor data into a buffer; flushed every 300 ms below.
      // if (wifiStreamSensor) {
      //Serial.println("=======");
      String str = String(F("S:")) + String(millis()) + "," +
                   String(valeurCapteurInstant, 4) + "," +
                   String(valeurCapteurMoyenne, 4) + "," + String(med_val, 4) +
                   "\n";

      //Serial.println(str);
      sensorBuffer += str;
      // }
    }
  }
  // Flush the sensor buffer every 300 ms in one AT+CIPSEND call.
  // Batching avoids a separate AT round-trip per sample and gives the
  // control loop far more breathing room than calling logStream() at 10 Hz.
 // if (wifiStreamSensor && sensorBuffer.length() > 0 &&
 //     millis() - lastSensorFlush >= SENSOR_FLUSH_MS) {
 //   Serial.println("Sending sendbuffer");
 //   wifiLogger.sendAsync(sensorBuffer);
 //   sensorBuffer = "";
 //   lastSensorFlush = millis();
 // }

  if (millis() - lastCapteurUpdate > capteurTimeout) {
    capteurResponsive = false;
  }

  // Update VESC telemetry every vescUpdateInterval ms
  if (millis() - lastVescUpdate >= vescUpdateInterval) {
    if (Serial2.available() >= VESC_TELEM_SIZE) {
      updateVescTelemetry(); // this will now complete instantly
      lastVescUpdate = millis();
    }
    // If not enough bytes, we simply skip this update – no blocking
  }

  // watddetre.update();

  /* if (powerChrono.elapsed() > debounceTime && digitalRead(powerPin) ==
  powerNewState)
  {
    powerCtrl = powerNewState;
    powerChrono.restart();
    powerChrono.stop();
    switchCtrl(powerCtrl);
  }*/
  // Spam the button to reset???

  /*   if (motorBrakeChrono.elapsed() > debounceTime &&
    digitalRead(motorBrakePin) == motorBrakeNewState)
    {
      motorBrakeMode = !motorBrakeNewState;
      motorBrakeChrono.restart();
      motorBrakeChrono.stop();

      // reset capteur de force
      // Quand on active le frein moteur, si en mode 0
      if (powerCtrl == 0 && etat == INITIALISATION)
      {
        resetOffsetChrono.restart();
        resetOffsetIter++;
      }
    } */

  ////////////////////////////////////////////////////:
  ///////////////////  0  ////////////////////////////
  ////////////////////////////////////////////////////:

  if (etat == INITIALISATION) {

    // moteur.setMoteurState(STOPPED);
    sortieMoteurAccel = 0;
    capteur.setThresholdSensor(0.5);
    // switchCtrl(powerCtrl);

    // Si ça fait plus de 1000ms que le chrono est lancé, on ne veut pas
    // réinitialiser le capteur, remise à 0.
    if (resetOffsetChrono.elapsed() > 1000) {
      resetOffsetIter = 0;
      resetOffsetChrono.stop();
    }

    if (transition5()) {
      // if (debug)
      //{
      Serial.println("Transition 5 from INIT");
      //}
      etat = FREINAGE;
    } else if (transition01()) {
      Serial.println("Transition ATTENTE from INIT");
      etat = ATTENTE;
    } else if (transition07()) {
      Serial.println("Transition RESET from INIT");
      etat = RESET_CAPTEUR;
    }
  }
  ////////////////////////////////////////////////////:
  ///////////////////  1  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == ATTENTE) {

    mainPID.SetMode(MANUAL);
    // moteur.setMoteurState(STOPPED);
    sortieMoteurAccel = 0;
    capteur.setThresholdSensor(0.5);

    // En mode 1, on arrête le chrono et remet à 0 les itérations pour interdire
    // le reset capteur.
    resetOffsetIter = 0;
    resetOffsetChrono.stop();

    /* a supprimer si on retire le calcul de puissance consommée.
    if (flowingChrono.isRunning()) {
      flowingChrono.restart();
      flowingChrono.stop();
    }
    */

    if (transition5()) {
      etat = FREINAGE;
    } else if (transition0())
      etat = INITIALISATION;
    else if (transition12())
      etat = ROULE;
    else if (transition15())
      etat = FREINAGE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  2  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == ROULE) {
    // moteur.setMoteurState(SPINNING);
    mainPID.SetMode(AUTOMATIC);
    miseAJourPID();

    // if (!flowingChrono.isRunning())
    //{
    // flowingChrono.restart(); //
    //}

    capteur.setThresholdSensor(0.0);
    //   flowingOrNot();

    if (transition5())

    {
      etat = FREINAGE;
    } else if (transition21())
      etat = ATTENTE;
    else if (transition0())
      etat = INITIALISATION;
    else if (transition23())
      etat = STATU_QUO;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  3  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == STATU_QUO) {
    // moteur.setMoteurState(SPINNING);

    mainPID.SetMode(AUTOMATIC);
    miseAJourPID();

    if (transition5()) {
      etat = FREINAGE;
    } else if (transition0())
      etat = INITIALISATION;
    else if (transition31())
      etat = ATTENTE;
    else if (transition32())
      etat = ROULE;
    else if (transition34())
      etat = DECCELERATION;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  4  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == DECCELERATION) {

    // moteur.setMoteurState(STOPPED);
    mainPID.SetMode(MANUAL);
    sortieMoteurAccel = 0;

    if (transition5()) {
      etat = FREINAGE;
    } else if (transition0())
      etat = INITIALISATION;
    else if (transition42())
      // etat = ROULE;
      // c'était en état == ATTENTE, càd  état 1. Pourquoi?
      etat = ATTENTE;
    else if (transition45())
      etat = FREINAGE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  5  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == FREINAGE) {

    mainPID.SetMode(MANUAL);
    // moteur.setMoteurState(BRAKING);
    sortieMoteurAccel = 0;
    capteur.setThresholdSensor(0.5);

    // En mode 5, on arrête le chrono et remet à 0 les itérations pour interdire
    // le reset capteur.
    resetOffsetIter = 0;
    resetOffsetChrono.stop();

    /*
    if (flowingChrono.isRunning()) {
      flowingChrono.restart();
      flowingChrono.stop();
    }
    */

    if (transition5()) {
      etat = FREINAGE;
    } else if (transition0())
      etat = INITIALISATION;
    else if (transition51())
      etat = ATTENTE;
    else if (transition52())
      etat = ROULE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  7  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == RESET_CAPTEUR) {
    /*
       On attend 500ms puis la valeur actuelle du capteur est stockée et lissée
       sur 32 valeurs
    */

    /*
      Je comprends pas le rapport au nombre 10.
    */
    if (resetOffsetIter != 10)
      resetOffsetChrono.restart();
    resetOffsetIter = 10;
    if (resetOffsetChrono.elapsed() > 500) {
      rawValue = capteur.getRaw();
      movingOffset.push(&rawValue, &newOffset);
    }
    if (transition70()) {
      //  Serial.print(" ## capteur valeur: ");Serial.println(newOffset);
      EEPROM.put(calVal_eepromAdress, newOffset);
      capteur.setOffset(newOffset);
      resetOffsetIter = 0;
      resetOffsetChrono.restart();
      resetOffsetChrono.stop();
      etat = INITIALISATION;
    }
  }

  else {
    Serial.println("Sortie de cas");
  }

  // --- Test: Display 8 parameters in 4 rows ---
  static unsigned long lastParamTest = 0;
  static int n = 0;
  static int displayAorB = 0;

  if (millis() - lastParamTest >= 200) {
    // Serial.println(">200");
    // Serial.print("timestamp:");
    // Serial.print(millis());
    // Serial.println();
    lastParamTest = millis();

    n = n + 1;
    if (n >= 10) {
      displayAorB = 1 - displayAorB;
      n = 0;
    }

    readADCInputs();
    lastParamTest = millis();
    int b2 = (int)digitalRead(toggle2Pin);
    s16_t a0 = getADC0();
    float brakesMapped = getBrakesMapped();
    int a1 = getADC1();
    String walkModeStr = walkMode ? "Walk" : "Ride";
    int b1 = (int)digitalRead(toggle1Pin);
    int b3 = toggle3State;
    String b1str = b1 ? "On" : "Off";
    String b3str = b3 ? "Off" : "On";
    String motorStateStr = isCtrlAlive ? "Vesc" : "novesc";
    String motorBrakeStr = motorBrakeMode ? "MBrake" : "NoMBrk";
    float actualBrakeCurrent =
        fmap(brakesMapped, 0.0, 1.0, minBrakeCurrent, maxBrakeCurrent);
    actualBrakeCurrent = actualBrakeCurrent * (-1);

    String cstr = displayAorB ? b3str : motorStateStr;
    bikeDisplay.displayEightParams(
        String("A:") + etatsStr[etat],
        String("B:") + String(valeurCapteurMoyenne, 2), String("C:") + cstr,
        String("D:") + walkModeStr, String("E:") + String(brakesMapped, 2),
        String("F:") + String(vitesseMoyenne, 2),

        String("G:") + String(sortieMoteurAccel, 2),
        String("H:") + String(actualBrakeCurrent, 2));

    // --- WiFi telemetry log (CSV, 200 ms cadence) ---
    String csvLine =
        String(millis()) + "," + etatsStr[etat] + "," +
        String(valeurCapteurMoyenne, 2) + "," + String(vitesseMoyenne, 2) +
        "," + String(sortieMoteurAccel, 2) + "," + String(brakesMapped, 3) +
        "," + String(actualBrakeCurrent, 2) + "," +
        String(vescTelemetry.rpm, 0) + "," + String(vescTelemetry.voltage, 1) +
        "," + String(vescTelemetry.current, 2) + "," +
        String(vescTelemetry.duty, 3) + "," +
        String(vescTelemetry.tempMosfet, 1);+"/";
    // Only send the full CSV telemetry when not in sensor-stream mode.
    // Both use the same TCP connection so mixing them at different rates
    // would interleave SEND OK responses and confuse the AT command flow.
    if (!wifiStreamSensor) {


      String full_message= "C-"+csvLine+"\nS-\n"+sensorBuffer;
      wifiLogger.sendAsync(full_message);
      sensorBuffer = "";
      lastSensorFlush = millis();
    }
  }
  // send commands to VESC every 50ms
  static unsigned long lastCommandSent = 0;
  if (millis() - lastCommandSent > 50) {

    sendVescCommands();
    lastCommandSent = millis();
  }
}

// TODO envoi des instructions au vesc
void sendVescCommands() {
  // blocage manuel au bouton
  if (toggle3State == 1) {
    // vesc.setCurrent(0);
    //  vesc.setBrake(maxBrakeCurrent);

    if (etat == DECCELERATION || etat == FREINAGE) {
      float brakesMapped = getBrakesMapped();
      float actualBrakeCurrent =
          fmap(brakesMapped, 0, 1, minBrakeCurrent, maxBrakeCurrent);
      vesc.setBrake(actualBrakeCurrent * (-1));
      // vesc.setCurrent(0);
    } else {
      vesc.setCurrent(0);
    }
    return;
  }
  if (!capteurResponsive) {
    vesc.setCurrent(0);
    return;
  }

  if (isCtrlAlive) {
    if (etat == ROULE || etat == STATU_QUO) {

      if (sortieMoteurAccel > 0) {
        vesc.setCurrent(sortieMoteurAccel);
      } else {

        vesc.setBrake(sortieMoteurAccel);
      }
    }
    // vesc.setBrake(0);
    else if (etat == DECCELERATION || etat == FREINAGE) {
      float brakesMapped = getBrakesMapped();
      float actualBrakeCurrent =
          fmap(brakesMapped, 0, 1, minBrakeCurrent, maxBrakeCurrent);
      vesc.setBrake(actualBrakeCurrent * (-1));
      // vesc.setCurrent(0);
    }
  } else {
    Serial.println("no vesc to send to ");
    // pas tres util Vu Q'uon est deco
    vesc.setCurrent(0);
  }
}

void updateVescTelemetry() {
  Serial.println("up vesc val");
  Serial.println(millis());
  if (vesc.getValues()) {
    isCtrlAlive = true;
    vescResponsive = true;
    vescTelemetry.rpm = vesc.getRPM();
    vescTelemetry.voltage = vesc.getVoltage();
    vescTelemetry.current = vesc.getCurrent();
    vescTelemetry.duty = vesc.getDuty();
    vescTelemetry.tempMosfet = vesc.getTempMosfet();
    vescTelemetry.tempMotor = vesc.getTempMotor();
    vescTelemetry.ampHours = vesc.getAmpHours();
    vescTelemetry.ampHoursCharged = vesc.getAmpHoursCharged();
    vescTelemetry.wattHours = vesc.getWattHours();
    vescTelemetry.wattHoursCharged = vesc.getWattHoursCharged();
    vescTelemetry.tachometer = vesc.getTachometer();
    vescTelemetry.tachometerAbs = vesc.getTachometerAbs();
    miseAJourVitesse();
    if (debug) {
      Serial.print("VESC RPM: ");
      Serial.print(vescTelemetry.rpm);
      Serial.print(" | Voltage: ");
      Serial.print(vescTelemetry.voltage);
      Serial.print(" V | Current: ");
      Serial.print(vescTelemetry.current);
      Serial.print(" A | Duty: ");
      Serial.print(vescTelemetry.duty);
      Serial.print(" | Temp MOSFET: ");
      Serial.print(vescTelemetry.tempMosfet);
      Serial.print(" | Temp Motor: ");
      Serial.print(vescTelemetry.tempMotor);
      Serial.print(" | Ah: ");
      Serial.print(vescTelemetry.ampHours);
      Serial.print(" | Ah Charged: ");
      Serial.print(vescTelemetry.ampHoursCharged);
      Serial.print(" | Wh: ");
      Serial.print(vescTelemetry.wattHours);
      Serial.print(" | Wh Charged: ");
      Serial.print(vescTelemetry.wattHoursCharged);
      Serial.print(" | Tacho: ");
      Serial.print(vescTelemetry.tachometer);
      Serial.print(" | TachoAbs: ");
      Serial.println(vescTelemetry.tachometerAbs);
    }
  } else {

    // bikeDisplay.displayMessage("VESC not responsive");
    vescResponsive = false;
    isCtrlAlive = false;

    if (debugMotor) {
      isCtrlAlive = true;
    }
    if (debug) {
      Serial.println("Failed to get VESC values");
    }
  }
}

int InitializeStrengthSensor() {
  if (!capteurInitialise) {
    if (debugCapteur) {
      capteurInitialise = 1;
      return 1;
    } else {
      // capteur
      capteur.begin();
      // true pour tare
      if (capteur.start(/*true*/)) {
        Serial.println("Initialisation du capteur bien réussie");
        capteurInitialise = 1;
        return 1;
      } else {
        Serial.println("Initialisation du capteur échouée. Vérifier connexion");
        capteurInitialise = 0;
        return 0;
      }
    }
  } else
    return 1;
}

int initializeVesc() {
  bikeDisplay.displayMessage("Initializing VESC");
  delay(300);
  Serial2.setRX(serial2RX);
  Serial2.setTX(serial2TX);
  Serial2.setFIFOSize(256);
  Serial2.begin(115200);

  delay(1000);
  uint32_t startTime = millis();
  while (!Serial2 && (millis() - startTime < 3000)) {
    // Optional: Blink an LED or print debug info
  }

  if (!Serial2) {
    bikeDisplay.displayMessage("Serial2 Error");

    // Fallback: Use Seial (USB) to report error
    Serial.println("Error: Serial2 failed to initialize!");
    return 1; // Or enter a safe state
  }
  vesc.setSerialPort(&Serial2);
  bikeDisplay.displayMessage("VESC Initialized");
  return 0;
  // vesc.setDebugPort(&Serial);  //
  //  Start Serial1 for VESC UART
  //  setupVesc(&Serial1); // Pass Serial1 to VESC UART setup
}

void interruptCtrlAlive() {
  if (digitalRead(ctrlAlive))
    isCtrlAlive = 1;
  else
    isCtrlAlive = 0;
}

void setMode(int mode) {
  Serial.print("Setting mode ");
  Serial.println(mode);
  if (mode == 1) {
    walkMode = 1;
    consigneCapteur = consigneCapteurTab[1];
    beta = betaTab[1];
    brakeThreshold = gammaTab[1];
    // mainPID.SetOutputLimits(pwmMin, pwmMax);
    mainPID.SetOutputLimits(minCurrent, maxCurrent);
  } else {
    walkMode = 0;
    beta = betaTab[0];
    brakeThreshold = gammaTab[0];
    consigneCapteur = consigneCapteurTab[0];
    // mainPID.SetOutputLimits(pwmMin, pwmMax);

    mainPID.SetOutputLimits(minCurrent, maxCurrent);
  }

  setPIDMode(walkMode);
}

/*
   Fonctions annexes
*/
void miseAJourVitesse() {
  if (debugMotor) {
    // Serial.println("Oups");
  } else {
    float wheelRpm = (float)vescTelemetry.rpm /
                     (float)polePairs; // Vitesse de la roue en rpm
    float wheelCircumference =
        3.14159 * (float)wheelDiam / 1000.0; // circonférence de la roue en m
    float vitesseMps = (wheelRpm * wheelCircumference) / 60.0; // vitesse en m/s
    vitesseInstantanee = vitesseMps * 3.6;

    vitesseFiltree.push(&vitesseInstantanee, &vitesseMoyenne);
  }
}

void setPIDMode(bool walkOrNot) {
  if (walkOrNot) {
    mainPID.SetTunings(K2[0], K2[1], K2[2]);
  } else {
    mainPID.SetTunings(K1[0], K1[1], K1[2]);
  }
}

void miseAJourPID() {
  // lectureVitesse = vitesseInstantanee;
  if (!haltFlag)
    mainPID.Compute();
}

bool transition01() {
  // Serial.print("transition 01"); Serial.print(" - "); Serial.print(etat ==
  // INITIALISATION); Serial.print(" - ");
  // Serial.print(initialisationCapteur()); Serial.print(" - ");
  // Serial.print(vitesseMoyenne < 1.0); Serial.print(" - ");
  // Serial.print(isCtrlAlive); Serial.println();
  return (etat == INITIALISATION && InitializeStrengthSensor() &&
          vitesseMoyenne < rollSpeedThreshold && isCtrlAlive);
}
bool transition07() {
  return (etat == INITIALISATION && resetOffsetIter > 3 && !isCtrlAlive);
}
bool transition12() {
  return (etat == ATTENTE && valeurCapteurMoyenne > alpha &&
          vitesseMoyenne > rollSpeedThreshold && isCtrlAlive);
}
bool transition15() {
  return (etat == ATTENTE &&
          (valeurCapteurMoyenne < brakeThreshold ||
           getBrakesMapped() > brakeThumbThrottleThreshold) &&
          isCtrlAlive);
}
bool transition21() {
  return (etat == ROULE && vitesseMoyenne < rollSpeedThreshold &&
          valeurCapteurMoyenne < alpha && isCtrlAlive);
}
bool transition23() {
  return (etat == ROULE &&
          (valeurCapteurMoyenne < 0.5 /*|| chronoFrein.elapsed() > t1*/) &&
          vitesseMoyenne > 0 && isCtrlAlive);
}

bool transition31() {
  return (etat == STATU_QUO && vitesseMoyenne < rollSpeedThreshold &&
          valeurCapteurMoyenne < alpha && isCtrlAlive);
}
bool transition32() {
  return (etat == STATU_QUO &&
          valeurCapteurMoyenne > alpha /*&& !chronoFrein.isRunning()*/ &&
          isCtrlAlive);
}

bool transition34() {
  return (
      etat == STATU_QUO &&
      (valeurCapteurMoyenne < beta &&
       valeurCapteurMoyenne > brakeThreshold) /*|| chronoFrein.elapsed() > t2 */
      && isCtrlAlive);
}
bool transition42() {
  return (etat == DECCELERATION &&
          valeurCapteurMoyenne > alpha /* && !chronoFrein.isRunning() */ &&
          isCtrlAlive);
}
bool transition45() {
  return (
      etat == DECCELERATION &&
      valeurCapteurMoyenne < brakeThreshold /*|| chronoFrein.elapsed() > t3*/ &&
      isCtrlAlive);
}
bool transition52() {
  return (etat == FREINAGE &&
          getBrakesMapped() <
              brakeThumbThrottleThreshold /*&& (valeurCapteurMoyenne > 2 * alpha
                                             || (valeurCapteurMoyenne > alpha &&
                                             vitesseMoyenne < 1.0)) /*&&
                                             !chronoFrein.isRunning()*/
          && isCtrlAlive && !motorBrakeMode);
}
bool transition51() {
  return (etat == FREINAGE &&
          getBrakesMapped() <
              brakeThumbThrottleThreshold /*&& !chronoFrein.isRunning()*/
          && vitesseMoyenne < rollSpeedThreshold &&
          valeurCapteurMoyenne < alpha && isCtrlAlive && !motorBrakeMode);
}
bool transition70() {
  return (etat == RESET_CAPTEUR && !isCtrlAlive && !motorBrakeMode &&
          resetOffsetChrono.elapsed() > 3000);
}
bool transition0() { return (!isCtrlAlive || !InitializeStrengthSensor()); }

bool transition5() {
  if (debug) {
    Serial.print("Transition 5: ");
    Serial.print(valeurCapteurMoyenne);
    Serial.print(" <? ");
    Serial.println(brakeThreshold);
  }

  return (isCtrlAlive &&
          (motorBrakeMode || valeurCapteurMoyenne < brakeThreshold ||
           getBrakesMapped() > brakeThumbThrottleThreshold));
}

// ---------------------------------------------------------------------------
// handleWifiCommand()  — parse and act on commands received over WiFi TCP
//
// Supported commands:
//   SET <key> <value>   — update a tunable parameter
//   GET ALL             — dump all tunable parameters back to the client
//
// Keys:
//   kp1 ki1 kd1         — PID tunings for boost/ride mode  (K1)
//   kp2 ki2 kd2         — PID tunings for walk mode        (K2)
//   alpha               — sensor threshold above which PID engages
//   beta0  beta1        — decel threshold per mode
//   gamma0 gamma1       — brake threshold per mode
//   sp0    sp1          — PID setpoint (consigne) per mode
//   minA   maxA         — motor current limits (A)
//   minBrk maxBrk       — brake current limits (A)
//   rollThr             — minimum speed to be considered "rolling" (km/h)
//   brakeThr            — brake thumb-throttle dead-zone threshold
// ---------------------------------------------------------------------------
void handleWifiCommand(const String &cmd) {
  Serial.print(F("[CMD] Handling: '"));
  Serial.print(cmd);
  Serial.println(F("'"));

  // ---- SET <key> <value> ----
  if (cmd.startsWith("SET ")) {
    String rest = cmd.substring(4);
    rest.trim();

    int spaceIdx = rest.indexOf(' ');
    if (spaceIdx == -1) {
      Serial.println(F("[CMD] ERR: SET needs a value (e.g. SET kp1 2.5)"));
      wifiLogger.sendAsync(F("ERR SET requires: SET <key> <value>"));
      return;
    }

    String key = rest.substring(0, spaceIdx);
    float value = rest.substring(spaceIdx + 1).toFloat();
    bool found = true;

    Serial.print(F("[CMD] SET "));
    Serial.print(key);
    Serial.print(F(" = "));
    Serial.println(value, 4);

    // --- PID tunings ---
    if (key == "kp1") {
      K1[0] = value;
      if (!walkMode)
        setPIDMode(walkMode);
      Serial.println(F("[CMD]  -> K1[0] updated, PID re-tuned"));
    } else if (key == "ki1") {
      K1[1] = value;
      if (!walkMode)
        setPIDMode(walkMode);
      Serial.println(F("[CMD]  -> K1[1] updated, PID re-tuned"));
    } else if (key == "kd1") {
      K1[2] = value;
      if (!walkMode)
        setPIDMode(walkMode);
      Serial.println(F("[CMD]  -> K1[2] updated, PID re-tuned"));
    } else if (key == "kp2") {
      K2[0] = value;
      if (walkMode)
        setPIDMode(walkMode);
      Serial.println(F("[CMD]  -> K2[0] updated, PID re-tuned"));
    } else if (key == "ki2") {
      K2[1] = value;
      if (walkMode)
        setPIDMode(walkMode);
      Serial.println(F("[CMD]  -> K2[1] updated, PID re-tuned"));
    } else if (key == "kd2") {
      K2[2] = value;
      if (walkMode)
        setPIDMode(walkMode);
      Serial.println(F("[CMD]  -> K2[2] updated, PID re-tuned"));
    }
    // --- thresholds ---
    else if (key == "alpha") {
      alpha = value;
      Serial.println(F("[CMD]  -> alpha updated"));
    } else if (key == "beta0") {
      betaTab[0] = value;
      if (!walkMode)
        beta = betaTab[0];
      Serial.println(F("[CMD]  -> betaTab[0] updated"));
    } else if (key == "beta1") {
      betaTab[1] = value;
      if (walkMode)
        beta = betaTab[1];
      Serial.println(F("[CMD]  -> betaTab[1] updated"));
    } else if (key == "gamma0") {
      gammaTab[0] = value;
      if (!walkMode)
        brakeThreshold = gammaTab[0];
      Serial.println(F("[CMD]  -> gammaTab[0] updated"));
    } else if (key == "gamma1") {
      gammaTab[1] = value;
      if (walkMode)
        brakeThreshold = gammaTab[1];
      Serial.println(F("[CMD]  -> gammaTab[1] updated"));
    }
    // --- setpoints ---
    else if (key == "sp0") {
      consigneCapteurTab[0] = value;
      if (!walkMode)
        consigneCapteur = value;
      Serial.println(F("[CMD]  -> consigneCapteurTab[0] updated"));
    } else if (key == "sp1") {
      consigneCapteurTab[1] = value;
      if (walkMode)
        consigneCapteur = value;
      Serial.println(F("[CMD]  -> consigneCapteurTab[1] updated"));
    }
    // --- current limits ---
    else if (key == "minA") {
      minCurrent = value;
      mainPID.SetOutputLimits(minCurrent, maxCurrent);
      Serial.println(F("[CMD]  -> minCurrent updated, PID limits refreshed"));
    } else if (key == "maxA") {
      maxCurrent = value;
      mainPID.SetOutputLimits(minCurrent, maxCurrent);
      Serial.println(F("[CMD]  -> maxCurrent updated, PID limits refreshed"));
    } else if (key == "minBrk") {
      minBrakeCurrent = value;
      Serial.println(F("[CMD]  -> minBrakeCurrent updated"));
    } else if (key == "maxBrk") {
      maxBrakeCurrent = value;
      Serial.println(F("[CMD]  -> maxBrakeCurrent updated"));
    }
    // --- stream mode toggle ---
    else if (key == "stream") {
      // SET stream 1  → sensor stream mode (raw samples via logStream)
      // SET stream 0  → full CSV telemetry mode (200 ms cadence via log)
      wifiStreamSensor = (value != 0.0f);
      Serial.print(F("[CMD]  -> wifiStreamSensor = "));
      Serial.println(wifiStreamSensor);
      wifiLogger.sendAsync(wifiStreamSensor
                               ? "ACK stream SENSOR (raw samples, use graph.py)"
                               : "ACK stream CSV (200 ms telemetry)");
      found = false; // suppress the generic ACK below — we already sent one
    }
    // --- speed / brake thresholds ---
    else if (key == "rollThr") {
      rollSpeedThreshold = value;
      Serial.println(F("[CMD]  -> rollSpeedThreshold updated"));
    } else if (key == "brakeThr") {
      brakeThumbThrottleThreshold = value;
      Serial.println(F("[CMD]  -> brakeThumbThrottleThreshold updated"));
    } else {
      found = false;
      Serial.print(F("[CMD] ERR: unknown key '"));
      Serial.print(key);
      Serial.println(F("'"));
      wifiLogger.sendAsync("ERR unknown key: " + key);
    }

    if (found) {
      String ack = "ACK " + key + " " + String(value, 4);
      wifiLogger.sendAsync(ack);
      Serial.print(F("[CMD] Sent: "));
      Serial.println(ack);
    }
  }

  // ---- GET ALL ----
  else if (cmd == "GET ALL") {
    Serial.println(F("[CMD] GET ALL — building single payload..."));

    // Build the entire response as one string so it goes out in a single
    // AT+CIPSEND, avoiding the per-call SEND OK round-trip overhead.
    // Each line is \r\n terminated; log() will append one final \r\n.
    String p = "";
    p += "--- PARAMS BEGIN ---\r\n";
    p += "kp1=" + String(K1[0], 4) + "\r\n";
    p += "ki1=" + String(K1[1], 4) + "\r\n";
    p += "kd1=" + String(K1[2], 4) + "\r\n";
    p += "kp2=" + String(K2[0], 4) + "\r\n";
    p += "ki2=" + String(K2[1], 4) + "\r\n";
    p += "kd2=" + String(K2[2], 4) + "\r\n";
    p += "alpha=" + String(alpha, 4) + "\r\n";
    p += "beta0=" + String(betaTab[0], 4) + "\r\n";
    p += "beta1=" + String(betaTab[1], 4) + "\r\n";
    p += "gamma0=" + String(gammaTab[0], 4) + "\r\n";
    p += "gamma1=" + String(gammaTab[1], 4) + "\r\n";
    p += "sp0=" + String(consigneCapteurTab[0], 4) + "\r\n";
    p += "sp1=" + String(consigneCapteurTab[1], 4) + "\r\n";
    p += "minA=" + String(minCurrent, 4) + "\r\n";
    p += "maxA=" + String(maxCurrent, 4) + "\r\n";
    p += "minBrk=" + String(minBrakeCurrent, 4) + "\r\n";
    p += "maxBrk=" + String(maxBrakeCurrent, 4) + "\r\n";
    p += "rollThr=" + String(rollSpeedThreshold, 4) + "\r\n";
    p += "brakeThr=" + String(brakeThumbThrottleThreshold, 4) + "\r\n";
    p += "--- PARAMS END ---";

    Serial.print(F("[CMD] GET ALL payload: "));
    Serial.print(p.length());
    Serial.println(F(" bytes — sending as one AT+CIPSEND"));

    wifiLogger.sendAsync(p);
    Serial.println(F("[CMD] GET ALL done"));
  }

  // ---- unknown ----
  else {
    Serial.print(F("[CMD] ERR: unknown command '"));
    Serial.print(cmd);
    Serial.println(F("'"));
    wifiLogger.sendAsync("ERR unknown command: " + cmd);
    wifiLogger.sendAsync(
        F("ERR valid commands: SET <key> <value>  |  GET ALL"));
  }
}

// Display debug info on the OLED screen (tiny screen, keep it short)
void displayDebugInfo() {
  static unsigned long lastDebugDisplay = 0;
  const unsigned long debugDisplayInterval = 500; // ms
  if (millis() - lastDebugDisplay < debugDisplayInterval)
    return;
  lastDebugDisplay = millis();

  char buf[22];
  snprintf(buf, sizeof(buf), "rCapteur: %.1f", rCapteur.value);
  bikeDisplay.displayMessage(buf);
}
