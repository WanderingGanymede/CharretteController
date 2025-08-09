#include "Arduino.h"
#include <HX711_ADC.h>

#ifndef StrengthSensor_h
#define StrengthSensor_h



typedef enum{
  EQUILIBRIUM = 0,
  PULLED = 1,
  PRESSED = 2,
} strength_sensor_state_t;

class StrengthSensor{
  public:
    StrengthSensor(uint8_t dataPin = 8, uint8_t sckPin = 7, double offset = 838.66, float calibration = 10000, float threshold_sensor_eq = 0.5); //constructeur
    void begin(); // initialisation
    bool start(bool tare = false); // démarrage avec tarage ou pas. Renvoie 0 ou 1 si problème  ou tout va bien
    void update(bool *newDataReady, double *value); // à lancer à chaque cycle. Si prêt, drapeau levé, valeur à lire;
  	double getRaw();
  	void setSamplesInUse(int samples); // samples must be 2, 4, 8, 16, 32, 64 or 128.
  	void setThresholdSensor(float new_threshold);
    void setOffset(double offset);

  protected:
    uint8_t _dataPin;
    uint8_t _sckPin;
    float _calibrationValue;
    HX711_ADC _cell; //= HX711_ADC(_dataPin,_sckPin);
    strength_sensor_state_t _state;
	float _threshold_sensor_equilibrium;
	double _offset;
	double _tmpValue = 0.0;
};

#endif
