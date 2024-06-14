#include "mbed.h"

class MotorDriver{

    private:
    DigitalOut _IN1;
    DigitalOut _IN2;
    PwmOut _EN;
    
    public:
    MotorDriver(PinName IN1, PinName IN2, PinName EN);
    void revolute(float PWM);
    void directRevolute(float PWM);
    void inverseRevolute(float PWM);
    void stop();

};