#ifndef MOTORCONTOLLER_H
#define MOTORCONTOLLER_H

#include "Encoder.h"
#include "mbed.h"
#include "MotorDriver.h"
#include "PID.h"

class MotorController {
    private:
        int m_mechID;
        MotorDriver *m_driver = nullptr;
        Encoder *m_encoder = nullptr;
        Timer *m_workingTimer = nullptr;
        PID *m_pid = nullptr;

        int u_max = 12; // напряжение движка (макс)
        
        // float m_interval = 0.5; // сколько раз в секунду слать сообщения по pc2
        // int u_max = 12;

    public:
    MotorController(MotorDriver *driver, Encoder *encoder, Timer *workingTimer, PID *pid);
    ~MotorController();

    void stopMotor();

    void setAngle(float degree);
};

#endif /* MOTORCONTOLLER_H */