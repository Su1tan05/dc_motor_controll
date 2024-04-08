#include "mbed.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>  
#include "Encoder.h"
#include "MotorDriver.h"
#include "MotorController.h"
#include "PID.h"

#define PUB_RATE 0.1

float kp = 5;
float ki = 0.5;
float kd = 0;
float td = 0.3;
float setpoint = 0;

Timer workingTimer;

PID pid_motor1(kp, ki, kd, td);
Encoder enc_motor1(D7, D6, 667, 16, Encoder::X4_ENCODING);
MotorDriver driver_motor1(D10, D11, D12);
MotorController contoller_motor1(&driver_motor1, &enc_motor1, &workingTimer, &pid_motor1);

Ticker motor_info_ticker;
ros::NodeHandle nh;
geometry_msgs::Vector3 monitoring_msg;

void pwmSub(const std_msgs::Float32 &pwm_msg);
void setAngle(const std_msgs::Float32 &angle_msg);
void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg);
void stopMotor(const std_msgs::Empty &stop_msg);
void getMotorInfo(void);
void initRosTopics(void);

ros::Subscriber<std_msgs::Float32> motor_pwm_sub("motor_pwm", &pwmSub);
ros::Subscriber<std_msgs::Empty> stop_motor_sub("stop_motor", &stopMotor);
ros::Subscriber<std_msgs::Float32> set_angle_sub("set_angle", &setAngle);
ros::Subscriber<geometry_msgs::Vector3> pid_tunings_sub("pid_tunings", &pidTuningsCb);

ros::Publisher monitoring_pub("monitoring", &monitoring_msg);

int main()
{
    nh.initNode();
    initRosTopics();
    motor_info_ticker.attach(getMotorInfo, PUB_RATE);
    while (1)
    {
        nh.spinOnce();
    }
}

void initRosTopics()
{
    nh.advertise(monitoring_pub);
    nh.subscribe(motor_pwm_sub);
    nh.subscribe(stop_motor_sub);
    nh.subscribe(set_angle_sub);
    nh.subscribe(pid_tunings_sub);
}

void pwmSub(const std_msgs::Float32 &pwm_msg) {

}

void stopMotor(const std_msgs::Empty &stop_msg) {

}

void setAngle(const std_msgs::Float32 &angle_msg) {
    setpoint = angle_msg.data;
    contoller_motor1.setAngle(setpoint);
}

void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg) {
    kp = pid_tunings_msg.x;
    ki = pid_tunings_msg.y;
    kd = pid_tunings_msg.z;
    
    pid_motor1.setPID(kp, ki, kd, td);
}

void getMotorInfo() {
    monitoring_msg.x = setpoint;
    monitoring_msg.y = enc_motor1.getPulses();
    monitoring_msg.z = enc_motor1.getCurrentAngle();
    monitoring_pub.publish(&monitoring_msg);
}