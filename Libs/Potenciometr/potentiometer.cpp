#include "mbed.h"
#include "potentiometer.h"

Potentiometer::Potentiometer(PinName pin) : _pin(pin){
    
}

void Potentiometer::setKalmanFilter(float KalmanCoef){
    m_KalmanCoef = KalmanCoef;
}

float Potentiometer::getPotData(){
    float potData = _pin.read();
    return potData;
}

float Potentiometer::getCurAngle(){

    static float curAngle = getPotData()*potMaxAngle;
    curAngle = curAngle * m_KalmanCoef + getPotData()*potMaxAngle * (1 - m_KalmanCoef);
    return curAngle;

}