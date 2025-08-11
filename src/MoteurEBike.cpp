#include "MoteurEBike.h"
#include "Vesc.h"
/*
 * Constructeur
 */
MoteurEBike::MoteurEBike(uint8_t pwm_pin, uint8_t brake_pin, uint8_t walk_pin, uint8_t U_pin, uint8_t V_pin)
{
  _moteur_state = STOPPED;
  _pwm_pin = pwm_pin;
  _brake_pin = brake_pin;
  _walk_pin = walk_pin;

  _hall_U_pin = U_pin;
  _hall_V_pin = V_pin;

  pinMode(_pwm_pin, OUTPUT);
  pinMode(_walk_pin, OUTPUT);
  pinMode(_brake_pin, OUTPUT);

  pinMode(_hall_U_pin, INPUT);
  pinMode(_hall_V_pin, INPUT);
}

/*
 *  récupérer l'état du moteur
 */
moteur_state_t MoteurEBike::getMoteurState()
{
  return (moteur_state_t)_moteur_state;
}

/*
 * Définir l'état du moteur
 */
void MoteurEBike::setMoteurState(moteur_state_t state)
{
  _moteur_state = state;
  switch (state)
  {
  case STOPPED:
    _gachette = 0;
    break;
  case WALKING:
    _gachette = 135;
    break;
  case BRAKING:
    _gachette = 0;
    // activer le frein moteur;
    break;
  case SPINNING:
    _gachette = 135;
    break;
  default:
    _gachette = 12;
  }
}

/*
 * Récupérer la consigne de la gachette à atteindre
 */
uint8_t MoteurEBike::getConsigneGachette()
{
  return _consigne;
}

/*
 * Définir la consigne de la gachette à atteindre
 */
void MoteurEBike::setConsigneGachette(uint8_t cg)
{
  _consigne = cg;
}

/*
 * Récupérer la valeur imposée à la gachette actuellement
 */
uint8_t MoteurEBike::getGachette()
{
  return _gachette;
}

/*
 * diminuer la consigne de la gachette sous l'interruption.
 *  On récupère la valeur imposée actuelle diminuée de "moins"
 */

void MoteurEBike::ralentir(uint8_t moins)
{
  if (_gachette - moins >= 120)
  {
    _gachette -= moins;
    _consigne = _gachette;
  }
  if (_consigne < 135)
  {
    setMoteurState(STOPPED);
    setConsigneGachette();
  }
}

/*
 * augmenter la gachette
 */

void MoteurEBike::accelerer(uint8_t plus)
{
  if (_gachette + plus <= _consigne)
    _gachette += plus;
}

/*
 * augmenter la consigne
 */

void MoteurEBike::augmenterConsigne(uint8_t plus)
{
  if (_consigne + plus < 256)
    _consigne += plus;
  else
    _consigne = 255;
}

/*
 * interruption pour l'effet Hall
 */

void MoteurEBike::interruptU()
{
  if (digitalRead(_hall_U_pin))
  {
    interruptTime[0][1] = interruptTime[0][0];
    interruptTime[0][0] = micros();
    if (!_UFlag)
      _UFlag = 1;
  }
}

void MoteurEBike::interruptV()
{
  if (digitalRead(_hall_V_pin))
  {
    interruptTime[1][1] = interruptTime[1][0];
    interruptTime[1][0] = micros();
    if (!_VFlag)
      _VFlag = 1;
  }
}

uint8_t MoteurEBike::getUPin()
{
  return _hall_U_pin;
}

uint8_t MoteurEBike::getVPin()
{
  return _hall_V_pin;
}

void MoteurEBike::calculerVitesse()
{
  long diff = interruptTime[0][0] - interruptTime[0][1];
  if (diff)
  {
    float tmpVitesse = (float)ratio / diff;
    if (tmpVitesse < 100)
      vitesse = tmpVitesse;
    // else vitesse = -2.0;
  }
  else
  {
    vitesse = -1.0;
  }
  if (micros() - interruptTime[0][0] > 1000000)
  {
    vitesse = 0;
  }
}

float MoteurEBike::getVitesse()
{
  return vitesse;
}

/*
 * Envoyer le signal PWM
 */
void MoteurEBike::mettreLesGaz(double consigneVitesse)
{
  switch (_moteur_state)
  {
  case SPINNING:

    analogWrite(_pwm_pin, consigneVitesse);
    digitalWrite(_walk_pin, 0);
    digitalWrite(_brake_pin, 0);
    break;
  case WALKING:
    analogWrite(_pwm_pin, consigneVitesse);
    digitalWrite(_walk_pin, 1);
    digitalWrite(_brake_pin, 0);
    break;
  case STOPPED:
    analogWrite(_pwm_pin, 0);
    digitalWrite(_brake_pin, 0);
    digitalWrite(_walk_pin, 0);
    break;
  case BRAKING:
    analogWrite(_pwm_pin, 0);
    digitalWrite(_brake_pin, 1);
    digitalWrite(_walk_pin, 0);
    break;
  default:
    analogWrite(_pwm_pin, 0);
    digitalWrite(_brake_pin, 1);
    digitalWrite(_walk_pin, 1);
  }
}
