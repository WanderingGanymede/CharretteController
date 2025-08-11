
#include "Arduino.h"
#ifndef MoteurEBike_h
#define MoteurEBike_h

// the #include statment and code go here...

typedef enum
{
  STOPPED = 0b00,  //  moteur arrêté, défaut
  SPINNING = 0b01, //  moteur qui tourne
  BRAKING = 0b10,  //  frein activé
  WALKING = 0b11   //  moteur en mode piéton
} moteur_state_t;

class MoteurEBike
{
public:
  MoteurEBike(uint8_t pwm_pin = 6, uint8_t brake_pin = 10, uint8_t walk_pin = 11, uint8_t uPin = A0, uint8_t vPin = A1);
  void begin();
  moteur_state_t getMoteurState();
  void setMoteurState(moteur_state_t state);
  uint8_t getConsigneGachette();
  void setConsigneGachette(uint8_t gachette = 200);
  uint8_t getGachette();
  void ralentir(uint8_t moins = 10);         //diminuer consigne et gachette
  void accelerer(uint8_t plus = 5);          // augmenter _gachette
  void augmenterConsigne(uint8_t plus = 30); // augmenter _gachette

  void interruptU();
  void interruptV();
  uint8_t getUPin();
  uint8_t getVPin();
  void calculerVitesse();
  float getVitesse();

  void mettreLesGaz(double consigneVitesse); // envoyer le PWM

  uint8_t _pwm_pin, _brake_pin, _walk_pin;


protected:
  moteur_state_t _moteur_state; // etat du moteur
  uint8_t _consigne;            // valeur de la consigne à atteindre
  uint8_t _gachette;            // valeur de la consigne actuellement
  uint8_t _hall_U_pin, _hall_V_pin;
  volatile long interruptTime[3][2] = {};
  bool _UFlag = 0, _VFlag = 0;
  float vitesse = 0;
  float ratio = 256000.0;
  long lastSpeedTick = 0;
};
#endif
