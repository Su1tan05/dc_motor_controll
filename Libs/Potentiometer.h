#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include "mbed.h"

class Potentiometer {

private:
  float potMaxAngleDeg = 540.00; // максимальный угол для измерений
  float potMaxAngleRad = potMaxAngleDeg * 3.14 / 180; // максимальный угол для измерений
  float m_KalmanCoef = 0.8;
  float m_zeroAngleDeg = 0.0; 
  float m_zeroAngleRad = 0.0; 

public:
  AnalogIn _pin;
  Potentiometer(PinName pin, float zeroAngleDeg); // конструктор класса
  void
  setKalmanFilter(float KalmanCoef); // сохранение коэффициента фильтра Калмана
  float getPotData(); // неотфильтрованные данные потенциометра
  int getPotDataU16();
  float getCurAngleDeg(); // возврат текущего угла поворота в градусах
  float getCurAngleRad(); // возврат текущего угла поворота в радианах
};

#endif /* POTENTIOMETER_H */