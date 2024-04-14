#include "mbed.h"
#include "encoder.h"
#include "PID.h"

class DcServo{

    private:
    DigitalOut _IN1;
    DigitalOut _IN2;
    PwmOut _EN;
    Encoder* _encoder;
    PID pid;
    Ticker pid_ticker;
    bool attached;
    float current_angle;
    float setpoint;
    void revoluteMotor();
    void initPid();
    void attachPidTiker();
    float PWM;

    public:
    DcServo(PinName IN1, PinName IN2, PinName EN, Encoder* encoder);
    void revolute(float PWM);
    void directRevolute(float PWM);
    void inverseRevolute(float PWM);
    void setAngle(float angle);
    void stop();
    void setPid(float Kc, float tauI, float tauD);
    float getPWM();
    float getAngle();
};