#include "mbed.h"
#include "MotorDriver.h"

MotorDriver::MotorDriver(PinName IN1, PinName IN2, PinName EN) : _IN1(IN1), _IN2(IN2), _EN(EN){
    
}

void MotorDriver::revolute(float PWM){
    
    if (PWM >= 1){ 
        PWM = 1;
        directRevolute(PWM);
        }
    else if (PWM <= -1){
        PWM = 1;
        inverseRevolute(PWM);
    }
    else if (PWM > 0){
        directRevolute(PWM);
    }
    else{
        PWM = -PWM;
        inverseRevolute(PWM);
    }
}

void MotorDriver::directRevolute(float PWM){
    
    _IN1.write(1);
    _IN2.write(0);
    _EN.write(PWM);

}
void MotorDriver::inverseRevolute(float PWM){

    _IN1.write(0);
    _IN2.write(1);
    _EN.write(PWM);

}

void MotorDriver::stop(){

    _IN1.write(0);
    _IN2.write(0);

}