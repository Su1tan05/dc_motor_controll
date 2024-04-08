#include "CurrentSensor.h"

CurrentSensor::CurrentSensor(PinName out, CurrentSensorType sensorType)
    : m_out(out) {

  m_sensorType = sensorType;
  if (m_sensorType == CurrentSensorType::max5A) {
    m_currentSensorZeroValue =
        46000; // можно сделать калибровку для вычсчитывания этого значения при
               // старте программы
    float sens = 185; // mВ / A
    m_currentSensorKoeff = sens * 65536 / 5000;
  }
}

int CurrentSensor::getCurrentADC(void) {
    return m_out.read_u16();
}

float CurrentSensor::getCurrent(void) {
    int currentSensorValue = getCurrentADC(); 
    float current = (m_currentSensorZeroValue - (float)currentSensorValue) / m_currentSensorKoeff * 1000.00; // mA
    return current;
}
 