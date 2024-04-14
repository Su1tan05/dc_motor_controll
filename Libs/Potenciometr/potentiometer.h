#include "mbed.h"

class Potentiometer{

    private:
    float potMaxAngle = 270.00; // максимальный угол для измерений
    float m_KalmanCoef;

    public:
    AnalogIn _pin;    
    Potentiometer(PinName pin); // конструктор класса
    void setKalmanFilter(float KalmanCoef); // сохранение коэффициента фильтра Калмана
    float getPotData(); // неотфильтрованные данные потенциометра
    float getCurAngle();  // возврат текущего угла поворота

};