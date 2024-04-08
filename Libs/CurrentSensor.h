#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

#include "mbed.h"

class CurrentSensor {

public:
  typedef enum CurrentSensorType {
    max5A,
  } CurrentSensorType;

  CurrentSensor(PinName out, CurrentSensorType sensorType = max5A);

  int getCurrentADC(void);

  float getCurrent(void);

private:
  CurrentSensorType m_sensorType;

  AnalogIn m_out;

  int m_currentSensorZeroValue;
  float m_currentSensorKoeff;
};

#endif /* CURRENTSENSOR_H */