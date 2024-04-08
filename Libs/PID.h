#ifndef PID_H
#define PID_H

#include "Encoder.h"
#include "mbed.h"
#include "MotorDriver.h"


class PID {
    private:
        float kp = 0, kd = 0, ki = 0, td = 0.3;
        float K_1, K_2, K_3, T_d; // коэфф-ты дискретного PID

    public:
        PID(float Td, float K_p, float K_i, float K_d);

        void setPID(float Td, float K_p, float K_i, float K_d);
        float getK1();
        float getK2();
        float getK3();
        float getTD();
        void resetPID();
};

#endif /* PID_H */