#include "PID.h"
#include "iostream"

PID::PID(float Td, float K_p, float K_i, float K_d) : kp(K_p), kd(K_d), ki(K_i), td(Td) {

}

void PID::setPID(float Td, float K_p, float K_i, float K_d) {
    T_d = Td;
    K_1 = (2 * K_p * Td + K_i * pow(Td, 2) + K_d) / (2 * Td);
    K_2 = (-2 * K_p * Td + K_i * pow(Td, 2) - 2 * K_d) / (2 * Td);
    K_3 = K_d / (2 * Td);
}

void PID::resetPID() {
    kp = 0;
    kd = 0;
    ki = 0;
    td = 0.3;
    T_d = 0;
    K_1 = 0;
    K_2 = 0;
    K_3 = 0;
}

float PID::getK1() { return K_1; }
float PID::getK2() { return K_2; }
float PID::getK3() { return K_3; }
float PID::getTD() { return T_d; }