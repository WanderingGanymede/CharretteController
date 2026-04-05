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

const int HX711_dout = 20; // mcu > HX711 dout pin
const int HX711_sck = 21;  // mcu > HX711 sck pin
double capteur_offset = 838.6 / 2.5f;
float capteur_calibration = 25000;
StrengthSensor capteur(HX711_dout, HX711_sck, capteur_offset, capteur_calibration, 0.5f);
// pour calculer la dérivée du capteur de force. Inutilisé
DifferentialFilter<double, unsigned long> diffCapteur;
Reading<double, unsigned long> rCapteur;
Reading<Differential<double>, unsigned long> dCapteur;

double valeurCapteurInstant;

MovingAverageFilter<double, double> valeurCapteurFiltered(10);
double valeurCapteurMoyenne;
bool newDataReady = 0;
bool capteurInitialise = 0;
Chrono resetOffsetChrono;                             // Chrono pour ?
int resetOffsetIter;                                  // compteur pour compter le nombre d'aller retour sur l'interrupteur brake
MovingAverageFilter<double, double> movingOffset(16); // moyenne lissée sur 16 valeurs
double newOffset, rawValue;

bool capteurResponsive = false;
long capteurTimeout = 2000;
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
bool vescResponsive = false;
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

double K1[3] = {1, 3, 0.125}; // boost, mode 0 pour les led
double K2[3] = {1, 2, 0.1};   // marche, mode 1 pour les led
float betaTab[2] = {-4, -3};
float gammaTab[2] = {-8, -6};
double consigneCapteurTab[2] = {-0.1, 0.0};

double sortieMoteurAccel;                       // output
double consigneCapteur = consigneCapteurTab[1]; // setpoint, valeur visée par le PID comme valeur de capteur.
                                                // les valeurs minimales et maximales pour le PWM de la gachette.
// 110 a été trouvée expérimentalement avec une batterie 48V et un contrôleur Ozo. En deça la roue ne tourne pas.
// ces valeurs sont à tester et corriger en cas de changement de batterie ou contrôleur.gvgv

/// KNOBS
float minCurrent = 1.5, maxCurrent = 30;

float minBrakeCurrent = 0, maxBrakeCurrent = 50;

int pidDir = REVERSE;

PID mainPID(&valeurCapteurMoyenne, &sortieMoteurAccel, &consigneCapteur, K1[0], K1[1], K1[2], P_ON_E, pidDir);
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

static String etatsStr[] = {"INIT", "ATT", "ROULE", "S_QUO", "DECEL", "FREIN", "MARCHE", "RESET"};
// Ancre paramètre 2
float rollSpeedThreshold = 1;
float alpha = 1;                    // seuil au dessus duquel le PID se calcule et se lance
float beta = betaTab[0];            // seuil en deça duquel on passe sur déccélération (pwm=0, pid manual)
float brakeThreshold = gammaTab[0]; // seuil en deça duquel on passe sur du freinage

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
 * Pour ajouter de la sécurité, l'idée est de mettre un contacteur au niveau du frein à inertie.
 * Quand ça s'ouvre, la remorque commence à aller plus vite que le vélo.
 * L'information est plus basique que celle du capteur de force mais permet d'ajouter une redondance d'informations.
 * Il n'a pas été trouvé encore de position correcte ou de contacteur fiable et robuste.
 * Idée en repos, mais importante à mettre en oeuvre par la suite.
 */

const int frein = debugFrein ? 4 : 5;
// Chrono chronoFrein;
/*bool freinFlag = 0;
int t1 = 300;
int t2 = 1000;
int t3 = 1500;
*/
////gachette frein

float brakeThumbThrottleThreshold = 0.03; // Seuil au delà duquel on considère que la gachette de frein est actionnée

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
  // mainPID.SetOutputLimits(pwmMin, pwmMax);
  mainPID.SetOutputLimits(minCurrent, maxCurrent);
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
}

void loop()
{

  // vitesseMoyenne = 2;
  InputControlHandler();
  static long lastCapteurUpdate = 0;
  // Call the toggle handler to check for toggle presses
  if (capteurInitialise)
  {
    capteur.update(&newDataReady, &valeurCapteurInstant);
    if (newDataReady)
    {
      capteurResponsive = true;
      lastCapteurUpdate = millis();
      valeurCapteurFiltered.push(&valeurCapteurInstant, &valeurCapteurMoyenne);
      rCapteur.value = valeurCapteurInstant * 1000;
      rCapteur.timestamp += millis() / 1000;
      diffCapteur.push(&rCapteur, &dCapteur);
    }
  }
  if (millis() - lastCapteurUpdate > capteurTimeout)
  {
    capteurResponsive = false;
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
    sortieMoteurAccel = 0;
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
      // if (debug)
      //{
      Serial.println("Transition 5 from INIT");
      //}
      etat = FREINAGE;
    }
    else if (transition01())
    {
      Serial.println("Transition ATTENTE from INIT");
      etat = ATTENTE;
    }
    else if (transition07())
    {
      Serial.println("Transition RESET from INIT");
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
    sortieMoteurAccel = 0;
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
    }
    else if (transition21())
      etat = ATTENTE;
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

  else if (etat == DECCELERATION)
  {

    // moteur.setMoteurState(STOPPED);
    mainPID.SetMode(MANUAL);
    sortieMoteurAccel = 0;

    if (transition5())
    {
      etat = FREINAGE;
    }
    else if (transition0())
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

  else if (etat == FREINAGE)
  {

    mainPID.SetMode(MANUAL);
    // moteur.setMoteurState(BRAKING);
    sortieMoteurAccel = 0;
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
    else if (transition51())
      etat = ATTENTE;
    else if (transition52())
      etat = ROULE;
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
  static int n = 0;
  static int displayAorB = 0;
  if (millis() - lastParamTest > 200)
  {
    n = n + 1;
    if (n >= 10)
    {
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
    float actualBrakeCurrent = fmap(brakesMapped, 0.0, 1.0, minBrakeCurrent, maxBrakeCurrent);
    actualBrakeCurrent = actualBrakeCurrent * (-1);

    String cstr = displayAorB ? b3str : motorStateStr;
    bikeDisplay.displayEightParams(
        String("A:") + etatsStr[etat],
        String("B:") + String(valeurCapteurMoyenne, 2),
        String("C:") + cstr,
        String("D:") + walkModeStr,
        String("E:") + String(brakesMapped, 2),
        String("F:") + String(vitesseMoyenne, 2),

        String("G:") + String(sortieMoteurAccel, 2),
        String("H:") + String(actualBrakeCurrent, 2));
  }
  // send commands to VESC every 50ms

  static unsigned long lastCommandSent = 0;
  if (millis() - lastCommandSent > 50)
  {

    sendVescCommands();
    lastCommandSent = millis();
  }
}

// TODO envoi des instructions au vesc
void sendVescCommands()
{
  // blocage manuel au bouton
  if (toggle3State == 1)
  {
    // vesc.setCurrent(0);
    //  vesc.setBrake(maxBrakeCurrent);

    if (etat == DECCELERATION || etat == FREINAGE)
    {
      float brakesMapped = getBrakesMapped();
      float actualBrakeCurrent = fmap(brakesMapped, 0, 1, minBrakeCurrent, maxBrakeCurrent);
      vesc.setBrake(actualBrakeCurrent * (-1));
      // vesc.setCurrent(0);
    }
    else
    {
      vesc.setCurrent(0);
    }
    return;
  }
  if (!capteurResponsive)
  {
    vesc.setCurrent(0);
    return;
  }

  if (isCtrlAlive)
  {
    if (etat == ROULE || etat == STATU_QUO)
    {

      if (sortieMoteurAccel > 0)
      {
        vesc.setCurrent(sortieMoteurAccel);
      }
      else
      {

        vesc.setBrake(sortieMoteurAccel);
      }
    }
    // vesc.setBrake(0);
    else if (etat == DECCELERATION || etat == FREINAGE)
    {
      float brakesMapped = getBrakesMapped();
      float actualBrakeCurrent = fmap(brakesMapped, 0, 1, minBrakeCurrent, maxBrakeCurrent);
      vesc.setBrake(actualBrakeCurrent * (-1));
      // vesc.setCurrent(0);
    }
  }
  else
  {
    // pas tres util Vu Q'uon est deco
    vesc.setCurrent(0);
  }
}

void updateVescTelemetry()
{
  if (vesc.getValues())
  {
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
    vescResponsive = false;
    isCtrlAlive = false;

    if (debugMotor)
    {
      isCtrlAlive = true;
    }
    if (debug)
    {
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
      // true pour tare
      if (capteur.start(/*true*/))
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

int initializeVesc()
{
  bikeDisplay.displayMessage("Initializing VESC");
  delay(300);
  Serial2.setRX(serial2RX);
  Serial2.setTX(serial2TX);
  Serial2.setFIFOSize(256);
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
    // mainPID.SetOutputLimits(pwmMin, pwmMax);
    mainPID.SetOutputLimits(minCurrent, maxCurrent);
  }
  else
  {
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
void miseAJourVitesse()
{
  if (debugMotor)
  {
    // Serial.println("Oups");
  }
  else
  {
    float wheelRpm = (float)vescTelemetry.rpm / (float)polePairs;   // Vitesse de la roue en rpm
    float wheelCircumference = 3.14159 * (float)wheelDiam / 1000.0; // circonférence de la roue en m
    float vitesseMps = (wheelRpm * wheelCircumference) / 60.0;      // vitesse en m/s
    vitesseInstantanee = vitesseMps * 3.6;

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
  return (etat == INITIALISATION && InitializeStrengthSensor() && vitesseMoyenne < rollSpeedThreshold && isCtrlAlive);
}
bool transition07()
{
  return (etat == INITIALISATION && resetOffsetIter > 3 && !isCtrlAlive);
}
bool transition12()
{
  return (etat == ATTENTE && valeurCapteurMoyenne > alpha && vitesseMoyenne > rollSpeedThreshold && isCtrlAlive);
}
bool transition15()
{
  return (etat == ATTENTE && (valeurCapteurMoyenne < brakeThreshold || getBrakesMapped() > brakeThumbThrottleThreshold) && isCtrlAlive);
}
bool transition21()
{
  return (etat == ROULE && vitesseMoyenne < rollSpeedThreshold && valeurCapteurMoyenne < alpha && isCtrlAlive);
}
bool transition23()
{
  return (etat == ROULE && (valeurCapteurMoyenne < 0.5 /*|| chronoFrein.elapsed() > t1*/) && vitesseMoyenne > 0 && isCtrlAlive);
}

bool transition31()
{
  return (etat == STATU_QUO && vitesseMoyenne < rollSpeedThreshold && valeurCapteurMoyenne < alpha && isCtrlAlive);
}
bool transition32()
{
  return (etat == STATU_QUO && valeurCapteurMoyenne > alpha /*&& !chronoFrein.isRunning()*/ && isCtrlAlive);
}

bool transition34()
{
  return (etat == STATU_QUO && (valeurCapteurMoyenne < beta && valeurCapteurMoyenne > brakeThreshold) /*|| chronoFrein.elapsed() > t2 */ && isCtrlAlive);
}
bool transition42()
{
  return (etat == DECCELERATION && valeurCapteurMoyenne > alpha /* && !chronoFrein.isRunning() */ && isCtrlAlive);
}
bool transition45()
{
  return (etat == DECCELERATION && valeurCapteurMoyenne < brakeThreshold /*|| chronoFrein.elapsed() > t3*/ && isCtrlAlive);
}
bool transition52()
{
  return (etat == FREINAGE && getBrakesMapped() < brakeThumbThrottleThreshold /*&& (valeurCapteurMoyenne > 2 * alpha || (valeurCapteurMoyenne > alpha && vitesseMoyenne < 1.0)) /*&& !chronoFrein.isRunning()*/ && isCtrlAlive && !motorBrakeMode);
}
bool transition51()
{
  return (etat == FREINAGE && getBrakesMapped() < brakeThumbThrottleThreshold /*&& !chronoFrein.isRunning()*/ && vitesseMoyenne < rollSpeedThreshold && valeurCapteurMoyenne < alpha && isCtrlAlive && !motorBrakeMode);
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
  if (debug)
  {
    Serial.print("Transition 5: ");
    Serial.print(valeurCapteurMoyenne);
    Serial.print(" <? ");
    Serial.println(brakeThreshold);
  }

  return (isCtrlAlive && (motorBrakeMode || valeurCapteurMoyenne < brakeThreshold || getBrakesMapped() > brakeThumbThrottleThreshold));
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
