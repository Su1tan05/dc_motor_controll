#include "Potentiometer.h"
#include "mbed.h"

Potentiometer::Potentiometer(PinName pin, float zeroAngleDeg) : _pin(pin) {
    m_zeroAngleDeg = zeroAngleDeg; 
    m_zeroAngleRad = zeroAngleDeg * 3.14 / 180; 
}

void Potentiometer::setKalmanFilter(float KalmanCoef) {
  m_KalmanCoef = KalmanCoef;
}

float Potentiometer::getPotData() {
  float potData = _pin.read();
  return potData;
}

int Potentiometer::getPotDataU16() {
  int potData = _pin.read_u16();
  return potData;
}

const int NUM_READ = 10;  // количество усреднений для средних арифм. фильтров
 

float Potentiometer::getCurAngleDeg() {
//   static float curAngle = getPotData() * potMaxAngleDeg;
//   curAngle = curAngle * m_KalmanCoef +
//              getPotData() * potMaxAngleDeg * (1 - m_KalmanCoef);
//   return curAngle;

  float sum = 0;                      // локальная переменная sum
  for (int i = 0; i < NUM_READ; i++)  // согласно количеству усреднений
    sum += getPotData() * potMaxAngleDeg;              // суммируем значения с любого датчика в переменную sum
  return (sum / NUM_READ - m_zeroAngleDeg);

}

float Potentiometer::getCurAngleRad() {
  float curAngle = getPotData() * potMaxAngleRad;
    curAngle =
    curAngle * m_KalmanCoef + getPotData() * potMaxAngleRad * (1 -
    m_KalmanCoef);
  return curAngle - m_zeroAngleRad;
}