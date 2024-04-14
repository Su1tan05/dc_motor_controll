#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"

#define PREV_MASK 0x1 //Mask for the previous state in determining direction
//of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction
//of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.

class Encoder{

    private:
    InterruptIn _ChA;
    InterruptIn _ChB;
    bool m_twoChannelMode;
    int m_encImpulseRotate;
    int m_reductioRatio;
    int m_pulseRepeats;
    volatile int m_encTicks;

    volatile int m_curState;
    volatile int m_prevState;
    bool m_resetFlag;

    bool m_clockwise;
    
    void incrementEncTicks();

    public:
    Encoder(PinName ChA, PinName ChB, int reductioRatio, int encImpulseRotate, bool twoChannelMode);
    int getEncTicks(); // получение тиков энкодера
    bool getCurDirection(); // получение текущего направления вращения
    float getCurAngle(); // получение текущего угла
    float getCurSpeed(); // получение текущей скорости
    void reset(); // сброс энкодера

/*    AnalogIn _pin;    
    Potentiometer(PinName pin);
    float getPotData();
*/
};

#endif