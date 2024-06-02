#include "mbed.h"
#include "encoder.h"
#include "DcServo.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>  

#include "config.h"
#define PUB_RATE 0.1

float motor1_setpoint = 0;
float motor2_setpoint = 0;
float motor3_setpoint = 0;

Ticker motor_info_ticker;
Timer timer;

Encoder motor1_encoder(PA_12, PC_8, REDUCTER_RATIO, ENCODER_IMPULSES, true);
// DcServo motor1_driver{PC_9, PB_8, D3, &motor1_encoder};

Encoder motor2_encoder{D13, D14, REDUCTER_RATIO, ENCODER_IMPULSES, true};
// DcServo motor2_driver{D7, D8, D5, &motor2_encoder};

Encoder motor3_encoder{D9, D10, REDUCTER_RATIO, ENCODER_IMPULSES, true};
// DcServo motor3_driver{D1, D4, D0, &motor3_encoder};

ros::NodeHandle nh;
std_msgs::Float32MultiArray monitoring_msg;

void pwmSub(const std_msgs::Float32 &pwm_msg);
void setAngleMotor1(const std_msgs::Float32 &angle_msg);
void setAngleMotor2(const std_msgs::Float32 &angle_msg);
// void setAngleMotor3(const std_msgs::Float32 &angle_msg);
void setMotor1PID(const geometry_msgs::Vector3 &pid_tunings_msg);
void setMotor2PID(const geometry_msgs::Vector3 &pid_tunings_msg);
// void setMotor3PID(const geometry_msgs::Vector3 &pid_tunings_msg);

void stopMotor1(const std_msgs::Empty &stop_msg);
void stopMotor2(const std_msgs::Empty &stop_msg);
// void stopMotor3(const std_msgs::Empty &stop_msg);
void getMotorInfo(void);
void initRosTopics(void);

void startCalculation(const std_msgs::Empty &stop_msg);
void stopCalculation(const std_msgs::Empty &stop_msg);

ros::Subscriber<std_msgs::Empty> stop_motor1_sub("stop_motor1", &stopMotor1);
ros::Subscriber<std_msgs::Empty> stop_motor2_sub("stop_motor2", &stopMotor2);
// ros::Subscriber<std_msgs::Empty> stop_motor3_sub("stop_motor3", &stopMotor3);

ros::Subscriber<std_msgs::Float32> set_angle_motor1_sub("set_angle_motor1", &setAngleMotor1);
ros::Subscriber<std_msgs::Float32> set_angle_motor2_sub("set_angle_motor2", &setAngleMotor2);
// ros::Subscriber<std_msgs::Float32> set_angle_motor3_sub("set_angle_motor3", &setAngleMotor3);

ros::Subscriber<geometry_msgs::Vector3> set_pid_motor1_sub("set_PID_motor1", &setMotor1PID);
ros::Subscriber<geometry_msgs::Vector3> set_pid_motor2_sub("set_PID_motor2", &setMotor2PID);
// ros::Subscriber<geometry_msgs::Vector3> set_pid_motor3_sub("set_PID_motor3", &setMotor3PID);

ros::Publisher motor_monitoring_pub("monitoring", &monitoring_msg);

ros::Subscriber<std_msgs::Empty> begin_calculation_sub("begin_calculation", &startCalculation);
ros::Subscriber<std_msgs::Empty> stop_calculation_sub("stop_calculation", &stopCalculation);

int Td_us = 1000000;
int ticks, prev_ticks;
float angle, prev_angle, speed, speed_ticks;
int currentADC;
float current;
float t, t1, dt;

int main()
{
    nh.logdebug("Start Programm");
    monitoring_msg.data_length = 3;
    monitoring_msg.data = (float *)malloc(sizeof(float)*monitoring_msg.data_length);
    nh.initNode();
    initRosTopics();
    timer.start();
    motor_info_ticker.attach(getMotorInfo, PUB_RATE);
    while (1)
    {
        nh.spinOnce();
    }
}

void initRosTopics()
{
    nh.advertise(motor_monitoring_pub);

    nh.subscribe(stop_motor1_sub);
    nh.subscribe(stop_motor2_sub);
    // nh.subscribe(stop_motor3_sub);

    nh.subscribe(set_angle_motor1_sub);
    nh.subscribe(set_angle_motor2_sub);
    // nh.subscribe(set_angle_motor3_sub);
    
    nh.subscribe(set_pid_motor1_sub);
    nh.subscribe(set_pid_motor2_sub);
    // nh.subscribe(set_pid_motor3_sub);

    nh.subscribe(begin_calculation_sub);
    nh.subscribe(stop_calculation_sub);
}

void pwmSub(const std_msgs::Float32 &pwm_msg)
{
    // motor1_driver.revolute(pwm_msg.data);
}

void stopMotor1(const std_msgs::Empty &stop_msg)
{
    motor1_setpoint = 0;
    // motor1_driver.stop();
}

void stopMotor2(const std_msgs::Empty &stop_msg)
{
    motor2_setpoint = 0;
    // motor2_driver.stop();
}

// void stopMotor3(const std_msgs::Empty &stop_msg)
// {
//     motor3_setpoint = 0;
//     motor3_driver.stop();
// }

void setAngleMotor1(const std_msgs::Float32 &angle_msg){
    motor1_setpoint = angle_msg.data;
    // motor1_driver.setAngle(motor1_setpoint);
}

void setAngleMotor2(const std_msgs::Float32 &angle_msg){
    motor2_setpoint = angle_msg.data;
    // motor2_driver.setAngle(motor2_setpoint);
}

// void setAngleMotor3(const std_msgs::Float32 &angle_msg){
//     motor3_setpoint = angle_msg.data;
//     motor3_driver.setAngle(motor3_setpoint);
// }

void setMotor1PID(const geometry_msgs::Vector3 &pid_tunings_msg)
{
    // motor1_driver.setPid(pid_tunings_msg.x, pid_tunings_msg.y, pid_tunings_msg.z);
}

void setMotor2PID(const geometry_msgs::Vector3 &pid_tunings_msg)
{
    // motor2_driver.setPid(pid_tunings_msg.x, pid_tunings_msg.y, pid_tunings_msg.z);
}

// void setMotor3PID(const geometry_msgs::Vector3 &pid_tunings_msg)
// {
//     motor3_driver.setPid(pid_tunings_msg.x, pid_tunings_msg.y, pid_tunings_msg.z);
// }

// DcServo motor1_driver{PC_9, PB_8, D3, &motor1_encoder};

DigitalOut _IN1 = PC_9;
DigitalOut _IN2 = PB_8;
PwmOut _EN = D3;

void startCalculation(const std_msgs::Empty &stop_msg) {
    t = 0;
    dt = 0;
    t1 = 0;
    timer.reset();
    _IN1.write(1);
    _IN2.write(0);
    _EN.write(1);
}

void stopCalculation(const std_msgs::Empty &stop_msg) {
    // motor3_driver.stop();
    _IN1.write(0);
    _IN2.write(0);
    _EN.write(0);
    timer.reset();
}

void getMotorInfo(){    
    t = timer.read();
    int ticks = motor1_encoder.getEncTicks();
    float angle = motor1_encoder.getCurAngle();
    dt = t - t1;
    speed_ticks = ((float)ticks - (float)prev_ticks) / dt;
    speed = (angle - prev_angle) / dt;


    monitoring_msg.data[0] = t;
    monitoring_msg.data[1] = dt;
    monitoring_msg.data[2] = speed;


    prev_ticks = ticks;
    prev_angle = angle;
    t1 = t;
    
    motor_monitoring_pub.publish(&monitoring_msg);

    // time speed step

    // monitoring_msg.data[0] = motor1_setpoint;
    // monitoring_msg.data[1] = motor1_driver.getAngle();
    // monitoring_msg.data[2] = motor1_driver.getPWM();
    
    // monitoring_msg.data[3] = motor2_setpoint;
    // monitoring_msg.data[4] = motor2_driver.getAngle();
    // monitoring_msg.data[5] = motor2_driver.getPWM();

    // monitoring_msg.data[6] = motor3_setpoint;
    // monitoring_msg.data[7] = motor3_driver.getAngle();
    // monitoring_msg.data[8] = motor3_driver.getPWM();

    // if (timer.read() > 30)
    // {
    //     timer.reset();
    // }
    
    // monitoring_msg.data[9] = timer.read();
    // motor_monitoring_pub.publish(&monitoring_msg);
}