// ...existing code...
#include "config.h"
#include "StrengthSensor.h"
#include "Vesc.h"
// ...existing code...
#include "main.h"
#include "BikeDisplay.h"
// ...existing code...

const int CALIBRATE_BUTTON_PIN = 18; // Use pin 4 for calibration button
#include <Arduino.h>
#include "HX711_ADC.h"
#include "LoadCell.h"
// ...existing code...
#include <Chrono.h>
#include <PID_v1.h>
#include <SoftFilters.h>
#include "InputControl.h"
// put function declarations here:
int myFunction(int, int);

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

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

bool debugPython = 0;  // pour envoyer les données au format attendu par le script python
bool debug = 0;        // pour envoyer les données de debug au format texte dans le moniteur série
bool debugCsv = 0;     // pour envoyer les données de debug au format csv dans le moniteur série
bool debugMotor = 0;   // si aucun moteur n'est branché, pour le simuler
bool debugFrein = 0;   // si aucun interrupteur n'est branché, sur le frein à inertie. Inutilisé actuellement
bool debugCapteur = 0; // si aucun capteur de force n'est branché, pour le simuler
bool debugOther = 0;   // utilisé à des fins de tests.
bool old = 0;          // pour assurer la compatibilité avec une ancienne version de carte électronique. A garder à 0.
int csvIter = 0;       // compteur d'itération pour l'affichage csv

/////////////////////////////////////////////////
/////////////// capteur force ///////////////////
/////////////////////////////////////////////////

const int HX711_dout = 12; // mcu > HX711 dout pin
const int HX711_sck = 13;  // mcu > HX711 sck pin
double capteur_offset = 841.86;

StrengthSensor capteur(HX711_dout, HX711_sck, capteur_offset);
// pour calculer la dérivée du capteur de force. Inutilisé
DifferentialFilter<double, unsigned long> diffCapteur;
Reading<double, unsigned long> rCapteur;
Reading<Differential<double>, unsigned long> dCapteur;

double valeurCapteur;
bool newDataReady = 0;
bool capteurInitialise = 0;
Chrono resetOffsetChrono;                             // Chrono pour ?
int resetOffsetIter;                                  // compteur pour compter le nombre d'aller retour sur l'interrupteur brake
MovingAverageFilter<double, double> movingOffset(16); // moyenne lissée sur 16 valeurs
double newOffset, rawValue;

const int calVal_eepromAdress = 0;

///

/////////////////////////////////////////////////
/////////////////// VESC /////////////////////////
/////////////////////////////////////////////////
const int serial1RX = 1;
const int serial1TX = 0;
const int serial2RX = 5;
const int serial2TX = 4;
Vesc vesc;
bool vescResponsive = true;
// VESC telemetry storage
struct VescTelemetry
{
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
 * Pour lier l'information du capteur de force au signal PWM envoyé dans la gachette, on utilise un PID.
 * https://playground.arduino.cc/Code/PIDLibrary/
 */

// Paramètres à changer:

double K1[3] = {1, 4, 0.1}; // boost, mode 0 pour les led
double K2[3] = {1, 2, 0.1}; // marche, mode 1 pour les led
float betaTab[2] = {-6, -3};
float gammaTab[2] = {-10, -6};
double consigneCapteurTab[2] = {-2.0, 0.0};

double sortieMoteur;                            // output
double consigneCapteur = consigneCapteurTab[1]; // setpoint, valeur visée par le PID comme valeur de capteur.
float pwmMin = 110, pwmMax = 255;               // les valeurs minimales et maximales pour le PWM de la gachette.
                                                // 110 a été trouvée expérimentalement avec une batterie 48V et un contrôleur Ozo. En deça la roue ne tourne pas.
                                                // ces valeurs sont à tester et corriger en cas de changement de batterie ou contrôleur.

PID mainPID(&valeurCapteur, &sortieMoteur, &consigneCapteur, K1[0], K1[1], K1[2], P_ON_E, REVERSE);
// Entrée: ValeurCapteur
// Valeur asservie: SortieMoteur, qui est un PWM allant de pwmMin à pwmMax
// ConsigneCapteur: Valeur visée pour ValeurCapteur
// Kp,Ki,Kd, les paramètres dont dépendent l'asservissement du pwm
// P_ON_E, Proportionnal on Error
// REVERSE, augmenter la valeur asservie, diminuera l'entrée.
//
/////////////////////////////////////////////////
//////////////// Etats //////////////////////////
/////////////////////////////////////////////////

enum etats_enum
{
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

// Ancre paramètre 2

float alpha = 1;                    // seuil au dessus duquel le PID se calcule et se lance
float beta = betaTab[0];            // seuil en deça duquel on passe sur déccélération (pwm=0, pid manual)
float brakeThreshold = gammaTab[0]; // seuil en deça duquel on passe sur du freinage

/////////////////////////////////////////////////
//////////// Vitesse moyenne ////////////////////
/////////////////////////////////////////////////

/*
 * Stockage de la vitesse du moteur.
 * Vitesse lue grâce aux capteurs à effet hall du moteur
 * En cas de défaillance de cette information, le système n'est plus utilisable
 */

// TODO lire la vitesse depuis le VESC A la place

MovingAverageFilter<double, double> vitesseFiltree(4);
double vitesseMoyenne;
double vitesseInstantanee;
/////////////////////////////////////////////////
/////////////// Frein à inertie /////////////////
/////////////////////////////////////////////////

/*
 * Inutilisée
 *
 * Pour ajouter de la sécurité, l'idée est de mettre un contacteur au niveau du frein à inertie.
 * Quand ça s'ouvre, la remorque commence à aller plus vite que le vélo.
 * L'information est plus basique que celle du capteur de force mais permet d'ajouter une redondance d'informations.
 * Il n'a pas été trouvé encore de position correcte ou de contacteur fiable et robuste.
 * Idée en repos, mais importante à mettre en oeuvre par la suite.
 */

const int frein = debugFrein ? 4 : 5;
Chrono chronoFrein;
bool freinFlag = 0;
int t1 = 300;
int t2 = 1000;
int t3 = 1500;
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

int ctrlAlive = 12;  // sur cette pin arrive le 5V de la gachette. Cette information nous renseigne sur l'état du contrôleur.
int ctrlSwitch = 13; // pin utilisée pour allumer le relais qui activait le contrôleur. Inutilisée depuis que le relais est activé en direct par un interrupteur. A nettoyer
bool isCtrlAlive = debugMotor ? 1 : 0;

bool walkMode = 1;
bool motorBrakeMode = 0;

void setup()
{

  Serial.begin(9600);
  delay(500);
  Serial.println("###################");
  Serial.println("## version 0.1: ");
  Serial.println("## date: : ");
  Serial.println("## Boulanger ");
  Serial.println("###################");

  setupInputControl();
  /*
    On a un nouveau boîtier de contrôle.
    Le mode est lu en fonction de analogRead sur walkPin.
      Avec un pont diviseur de tension, on a empiriquement:
        Mode 1: analogRead = 426 425
        Mode 2: analogRead = 71 72
        Mode 0: analogRead = 1023 (éteint)

  */
  // kkinterrupteur sur la carte
  pinMode(plus, INPUT_PULLUP);
  pinMode(moins, INPUT_PULLUP);
  pinMode(halt, INPUT_PULLUP);

  // PID
  mainPID.SetMode(MANUAL);
  mainPID.SetOutputLimits(pwmMin, pwmMax);
  mainPID.SetSampleTime(200);

  // Controleur
  pinMode(ctrlAlive, INPUT);
  pinMode(ctrlSwitch, OUTPUT);

  digitalWrite(ctrlSwitch, false);
  delay(1000);
  Wire1.setSDA(sdaWire1);
  Wire1.setSCL(sclWire1);
  Wire1.begin();
  bikeDisplay.begin();
  bikeDisplay.displayMessage("Bike Display Ready");
  delay(1000);

  initializeVesc();
  delay(2000);
  InitializeStrengthSensor();
  Serial.println();
  bikeDisplay.displayMessage("Starting Charrette");
  Serial.println("Starting Charrette yo...");
  // Example: show a value on the display at startup
  // bikeDisplay.displayMainDistance(1234); // Show 1234 meters as a test
}

void loop()
{
  InputControlHandler();
  // Call the toggle handler to check for toggle presses
  if (capteurInitialise)
  {
    capteur.update(&newDataReady, &valeurCapteur);
    rCapteur.value = valeurCapteur * 1000;
    rCapteur.timestamp += millis() / 1000;
    diffCapteur.push(&rCapteur, &dCapteur);
  }

  // Update VESC telemetry every vescUpdateInterval ms
  if (millis() - lastVescUpdate >= vescUpdateInterval)
  {
    ;
    updateVescTelemetry();
    lastVescUpdate = millis();
  }

  // watddetre.update();

  /* if (powerChrono.elapsed() > debounceTime && digitalRead(powerPin) == powerNewState)
  {
    powerCtrl = powerNewState;
    powerChrono.restart();
    powerChrono.stop();
    switchCtrl(powerCtrl);
  }*/
  // Spam the button to reset???

  /*   if (motorBrakeChrono.elapsed() > debounceTime && digitalRead(motorBrakePin) == motorBrakeNewState)
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

  if (etat == INITIALISATION)
  {

    // moteur.setMoteurState(STOPPED);
    sortieMoteur = 0;
    capteur.setThresholdSensor(0.5);
    // switchCtrl(powerCtrl);

    // Si ça fait plus de 1000ms que le chrono est lancé, on ne veut pas réinitialiser le capteur, remise à 0.
    if (resetOffsetChrono.elapsed() > 1000)
    {
      resetOffsetIter = 0;
      resetOffsetChrono.stop();
    }

    if (transition5())
    {
      etat = FREINAGE;
    }
    else if (transition01())
    {
      etat = ATTENTE;
    }
    else if (transition07())
    {
      etat = RESET_CAPTEUR;
    }
  }
  ////////////////////////////////////////////////////:
  ///////////////////  1  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == ATTENTE)
  {

    mainPID.SetMode(MANUAL);
    // moteur.setMoteurState(STOPPED);
    sortieMoteur = 0;
    capteur.setThresholdSensor(0.5);

    // En mode 1, on arrête le chrono et remet à 0 les itérations pour interdire le reset capteur.
    resetOffsetIter = 0;
    resetOffsetChrono.stop();

    /* a supprimer si on retire le calcul de puissance consommée.
    if (flowingChrono.isRunning()) {
      flowingChrono.restart();
      flowingChrono.stop();
    }
    */

    if (transition5())
    {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else if (transition12())
      etat = ROULE;
    else if (transition15())
      etat = FREINAGE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  2  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == ROULE)
  {
    // moteur.setMoteurState(SPINNING);
    mainPID.SetMode(AUTOMATIC);
    // miseAJourPID();
    // if (!flowingChrono.isRunning())
    //{
    // flowingChrono.restart(); //
    //}

    capteur.setThresholdSensor(0.0);
    //   flowingOrNot();

    if (transition5())

    {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else if (transition23())
      etat = STATU_QUO;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  3  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == STATU_QUO)
  {
    // moteur.setMoteurState(SPINNING);

    mainPID.SetMode(AUTOMATIC);
    miseAJourPID();

    if (transition5())
    {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else if (transition32())
      etat = ROULE;
    else if (transition34())
      etat = DECCELERATION;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  4  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == DECCELERATION)
  {

    // moteur.setMoteurState(STOPPED);
    mainPID.SetMode(MANUAL);
    sortieMoteur = 0;

    if (transition5())
    {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else if (transition42())
      etat = ROULE;
    // c'était en état == ATTENTE, càd  état 1. Pourquoi?
    //  etat = ATTENTE;
    else if (transition45())
      etat = FREINAGE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  5  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == FREINAGE)
  {

    mainPID.SetMode(MANUAL);
    // moteur.setMoteurState(BRAKING);
    sortieMoteur = 0;
    capteur.setThresholdSensor(0.5);

    // En mode 5, on arrête le chrono et remet à 0 les itérations pour interdire le reset capteur.
    resetOffsetIter = 0;
    resetOffsetChrono.stop();

    /*
    if (flowingChrono.isRunning()) {
      flowingChrono.restart();
      flowingChrono.stop();
    }
    */

    if (transition5())
    {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else if (transition52())
      etat = ATTENTE; // ROULE;
    else if (transition51())
      etat = ATTENTE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  7  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == RESET_CAPTEUR)
  {
    /*
       On attend 500ms puis la valeur actuelle du capteur est stockée et lissée sur 32 valeurs
    */

    /*
      Je comprends pas le rapport au nombre 10.
    */
    if (resetOffsetIter != 10)
      resetOffsetChrono.restart();
    resetOffsetIter = 10;
    if (resetOffsetChrono.elapsed() > 500)
    {
      rawValue = capteur.getRaw();
      movingOffset.push(&rawValue, &newOffset);
    }
    if (transition70())
    {
      //  Serial.print(" ## capteur valeur: ");Serial.println(newOffset);
      EEPROM.put(calVal_eepromAdress, newOffset);
      capteur.setOffset(newOffset);
      resetOffsetIter = 0;
      resetOffsetChrono.restart();
      resetOffsetChrono.stop();
      etat = INITIALISATION;
    }
  }

  else
  {
    Serial.println("Sortie de cas");
  }

  // --- Test: Display 8 parameters in 4 rows ---
  static unsigned long lastParamTest = 0;
  if (millis() - lastParamTest > 1000)
  {
    lastParamTest = millis();
    int b2 = (int)digitalRead(toggle2Pin);
    String walkModeStr = walkMode ? "Walk" : "Ride";
    bikeDisplay.displayEightParams(
        String("A:") + String(rCapteur.value, 1),
        String("B:") + String(valeurCapteur, 2),
        String("C:") + String(vescTelemetry.rpm, 0),
        String("D:") + walkModeStr,
        String("E:") + String(vescTelemetry.current, 1),
        String("F:") + String(vescTelemetry.duty, 2),
        String("G:") + String(vescTelemetry.tempMosfet, 1),
        String("H:") + String(vescTelemetry.tempMotor, 1));
  }
}

void updateVescTelemetry()
{
  if (vesc.getValues())
  {
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
    if (debug)
    {
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
  }
  else
  {

    // bikeDisplay.displayMessage("VESC not responsive");

    if (debug)
    {
      vescResponsive = false;
      Serial.println("Failed to get VESC values");
    }
  }
}
int InitializeStrengthSensor()
{
  if (!capteurInitialise)
  {
    if (debugCapteur)
    {
      capteurInitialise = 1;
      return 1;
    }
    else
    {
      // capteur
      capteur.begin();
      if (capteur.start())
      {
        Serial.println("Initialisation du capteur bien réussie");
        capteurInitialise = 1;
        return 1;
      }
      else
      {
        Serial.println("Initialisation du capteur échouée. Vérifier connexion");
        capteurInitialise = 0;
        return 0;
      }
    }
  }
  else
    return 1;
}
void test2()
{
  Serial.println("Test 2 function called");
}
int initializeVesc()
{
  bikeDisplay.displayMessage("Initializing VESC");
  delay(300);
  Serial2.setRX(serial2RX);
  Serial2.setTX(serial2TX);
  Serial2.setFIFOSize(128);
  Serial2.begin(115200);

  delay(1000);
  uint32_t startTime = millis();
  while (!Serial2 && (millis() - startTime < 3000))
  {
    // Optional: Blink an LED or print debug info
  }

  if (!Serial2)
  {
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

void interruptCtrlAlive()
{
  if (digitalRead(ctrlAlive))
    isCtrlAlive = 1;
  else
    isCtrlAlive = 0;
}
void setMode(int mode)
{
  Serial.print("Setting mode ");
  Serial.println(mode);
  if (mode == 1)
  {
    walkMode = 1;
    consigneCapteur = consigneCapteurTab[1];
    beta = betaTab[1];
    brakeThreshold = gammaTab[1];
    mainPID.SetOutputLimits(pwmMin, pwmMax);
  }
  else
  {
    walkMode = 0;
    beta = betaTab[0];
    brakeThreshold = gammaTab[0];
    consigneCapteur = consigneCapteurTab[0];
    mainPID.SetOutputLimits(pwmMin, pwmMax);
  }

  setPIDMode(walkMode);
}

/*
   Fonctions annexes
*/
void miseAJourVitesse()
{
  if (debugMotor)
  {
    // Serial.println("Oups");
  }
  else
  {

    vitesseInstantanee = vescTelemetry.tachometer;
    vitesseFiltree.push(&vitesseInstantanee, &vitesseMoyenne);
  }
}
void setPIDMode(bool walkOrNot)
{
  if (walkOrNot)
  {
    mainPID.SetTunings(K2[0], K2[1], K2[2]);
  }
  else
  {
    mainPID.SetTunings(K1[0], K1[1], K1[2]);
  }
}

void miseAJourPID()
{
  // lectureVitesse = vitesseInstantanee;
  if (!haltFlag)
    mainPID.Compute();
}

bool transition01()
{
  // Serial.print("transition 01"); Serial.print(" - "); Serial.print(etat == INITIALISATION); Serial.print(" - "); Serial.print(initialisationCapteur()); Serial.print(" - "); Serial.print(vitesseMoyenne < 1.0); Serial.print(" - "); Serial.print(isCtrlAlive);
  // Serial.println();
  return (etat == INITIALISATION && InitializeStrengthSensor() && vitesseMoyenne < 1.0 && isCtrlAlive);
}
bool transition07()
{
  return (etat == INITIALISATION && resetOffsetIter > 3 && !isCtrlAlive);
}
bool transition12()
{
  return (etat == ATTENTE && valeurCapteur > alpha && vitesseMoyenne > 1.0 && isCtrlAlive);
}
bool transition15()
{
  return (etat == ATTENTE && valeurCapteur < brakeThreshold || chronoFrein.elapsed() > t1 && isCtrlAlive);
}
bool transition23()
{
  return (etat == ROULE && (valeurCapteur < 0.5 || chronoFrein.elapsed() > t1) && vitesseMoyenne > 0 && isCtrlAlive);
}
bool transition32()
{
  return (etat == STATU_QUO && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition34()
{
  return (etat == STATU_QUO && (valeurCapteur < beta && valeurCapteur > brakeThreshold) || chronoFrein.elapsed() > t2 && isCtrlAlive);
}
bool transition42()
{
  return (etat == DECCELERATION && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition45()
{
  return (etat == DECCELERATION && valeurCapteur < brakeThreshold || chronoFrein.elapsed() > t3 && isCtrlAlive);
}
bool transition52()
{
  return (etat == FREINAGE && (valeurCapteur > 2 * alpha || (valeurCapteur > alpha && vitesseMoyenne < 1.0)) && !chronoFrein.isRunning() && isCtrlAlive && !motorBrakeMode);
}
bool transition51()
{
  return (etat == FREINAGE && !chronoFrein.isRunning() && vitesseMoyenne < 1.0 && valeurCapteur >= 0.0 && valeurCapteur < alpha && isCtrlAlive && !motorBrakeMode);
}
bool transition70()
{
  return (etat == RESET_CAPTEUR && !isCtrlAlive && !motorBrakeMode && resetOffsetChrono.elapsed() > 3000);
}
bool transition0()
{
  return (!isCtrlAlive || !InitializeStrengthSensor());
}

bool transition5()
{
  return (isCtrlAlive && (motorBrakeMode || valeurCapteur < brakeThreshold));
}

// Display debug info on the OLED screen (tiny screen, keep it short)
void displayDebugInfo()
{
  static unsigned long lastDebugDisplay = 0;
  const unsigned long debugDisplayInterval = 500; // ms
  if (millis() - lastDebugDisplay < debugDisplayInterval)
    return;
  lastDebugDisplay = millis();

  char buf[22];
  snprintf(buf, sizeof(buf), "rCapteur: %.1f", rCapteur.value);
  bikeDisplay.displayMessage(buf);
}
