#include "MotorController.h"
#include "iostream"


MotorController::MotorController(MotorDriver *driver, Encoder *encoder, Timer *workingTimer, PID *pid) {
    m_driver = driver;
    m_encoder = encoder;
    m_workingTimer = workingTimer;
    m_pid = pid;
}

MotorController::~MotorController() {
    delete m_driver;
    delete m_encoder;
    delete m_workingTimer;
    delete m_pid;
}

void MotorController::setAngle(float desiredAngle) {

    float Td_current = 0.0;
    float t = 0.0;
    float t1 = 0.0;
    float previousMsgSendTime = 0.0;

    float u = 0.0, u_1 = 0.0, u_norm = 0.0;
    float e = 0.0, e_1 = 0.0, e_2 = 0.0;

    int currentADC = 0;
    float current = 0.0;

    int currentSplineNumber = 0;
    float previousSplineIntervalTime = 0.0;
    float intervalTime = 0.0;

    float K_1 = m_pid -> getK1();
    float K_2 = m_pid -> getK2();
    float K_3 = m_pid -> getK3();
    float T_d = m_pid -> getTD();

    while (true) {
        t = m_workingTimer->read();

        float realAngle = m_encoder->getCurrentAngleRad();

        e = desiredAngle - realAngle;
        u = K_1 * e + K_2 * e_1 + K_3 * e_2 + u_1;

        if (u > u_max) {
            u_norm = u_max;
        } else if (u < -u_max) {
            u_norm = -u_max;
        } else {
            u_norm = u;
        }

        float pwm = u_norm / u_max;
        m_driver->revolute(pwm);

        u_1 = u;

        e_2 = e_1;
        e_1 = e;

        Td_current = t - t1;
        if (Td_current < T_d) {
            Thread:
            wait_ms((T_d - Td_current) * 1000);
        }
        t1 = t;
    }
}