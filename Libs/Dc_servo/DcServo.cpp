#include "mbed.h"
#include "DcServo.h"

#define PID_RATE 0.01

DcServo::DcServo(PinName IN1, PinName IN2, PinName EN, Encoder* encoder) : _IN1(IN1), _IN2(IN2), _EN(EN), _encoder{encoder}, pid(1, 0, 0, PID_RATE) {
    setpoint = 0;
    PWM = 0;
    attached = false;
    initPid();
    attachPidTiker();
}

float DcServo::getPWM(){
    return PWM;
}

float DcServo::getAngle(){
    return - _encoder->getCurAngle();
}

void DcServo::revolute(float PWM){
    this->PWM = PWM;
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

void DcServo::directRevolute(float PWM){
    
    _IN1.write(1);
    _IN2.write(0);
    _EN.write(PWM);

}
void DcServo::inverseRevolute(float PWM){

    _IN1.write(0);
    _IN2.write(1);
    _EN.write(PWM);

}

void DcServo::stop(){

    _IN1.write(0);
    _IN2.write(0);
    _EN.write(0);
    if (attached){
        pid_ticker.detach();
        attached = false;
    }
    wait(0.5);
    PWM = 0;
    setpoint = 0;
    _encoder->reset();  
}

void DcServo::setPid(float Kc, float tauI, float tauD){
    pid.setTunings(Kc, tauI, tauD);
}

void DcServo::setAngle(float angle){
    if (!attached){
        attachPidTiker();
    }
    setpoint = angle;
}

void DcServo:: revoluteMotor(){
    pid.setProcessValue(setpoint - getAngle());
    revolute(pid.compute());
}

void DcServo::initPid(){
    pid.setInputLimits(-360.0, 360.0);
    pid.setOutputLimits(-1.0, 1.0);
    pid.setBias(0.0);
    pid.setMode(AUTO_MODE);
}

void DcServo::attachPidTiker(){
    pid_ticker.attach(callback(this, &DcServo::revoluteMotor), PID_RATE);
    attached = true;
}