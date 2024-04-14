#include "mbed.h"
#include "motordriver.h"

MotorDriver::MotorDriver(PinName IN1, PinName IN2, PinName EN) : _IN1(IN1), _IN2(IN2), _EN(EN){
    
}

void MotorDriver::revolute(float PWM){
    
    if (PWM >= 1){ 
        PWM = 1;
        directRevolute(PWM);
        }
    // If PWM < -1 ---> PWM = 1, counterclockwise rotation
    else if (PWM <= -1){
        PWM = 1;
        inverseRevolute(PWM);
    }
    // If 0 < PWM < 1 ---> clockwise rotation
    else if (PWM > 0){
        directRevolute(PWM);
    }
    // If -1 < PWM < 0 ---> counterclockwise rotation
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

