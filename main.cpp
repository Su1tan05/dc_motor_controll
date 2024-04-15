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

float kp = 5;
float ki = 0.5;
float kd = 0;
float motor1_setpoint = 0;
float motor2_setpoint = 0;
float motor3_setpoint = 0;

Ticker motor_info_ticker;

// motor 1
Encoder motor1_encoder(PA_12, PC_8, REDUCTER_RATIO, ENCODER_IMPULSES, true);
DcServo motor1_driver{PC_9, PB_8, D3, &motor1_encoder};

// motor 2
Encoder motor2_encoder{D13, D14, REDUCTER_RATIO, ENCODER_IMPULSES, true};
DcServo motor2_driver{D7, D8, D5, &motor2_encoder};

// motor 3
Encoder motor3_encoder{D9, D10, REDUCTER_RATIO, ENCODER_IMPULSES, true};
DcServo motor3_driver{D4, D2, D6, &motor3_encoder};

ros::NodeHandle nh;
geometry_msgs::Vector3 motor1_monitoring_msg;
geometry_msgs::Vector3 motor2_monitoring_msg;
geometry_msgs::Vector3 motor3_monitoring_msg;

void pwmSub(const std_msgs::Float32 &pwm_msg);
void setAngleMotor1(const std_msgs::Float32 &angle_msg);
void setAngleMotor2(const std_msgs::Float32 &angle_msg);
void setAngleMotor3(const std_msgs::Float32 &angle_msg);
void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg);

void stopMotor1(const std_msgs::Empty &stop_msg);
void stopMotor2(const std_msgs::Empty &stop_msg);
void stopMotor3(const std_msgs::Empty &stop_msg);
void getMotorInfo(void);
void initRosTopics(void);

ros::Subscriber<std_msgs::Float32> motor_pwm_sub("motor_pwm", &pwmSub);
ros::Subscriber<std_msgs::Empty> stop_motor1_sub("stop_motor1", &stopMotor1);
ros::Subscriber<std_msgs::Empty> stop_motor2_sub("stop_motor2", &stopMotor2);
ros::Subscriber<std_msgs::Empty> stop_motor3_sub("stop_motor3", &stopMotor3);

ros::Subscriber<std_msgs::Float32> set_angle_motor1_sub("set_angle_motor1", &setAngleMotor1);
ros::Subscriber<std_msgs::Float32> set_angle_motor2_sub("set_angle_motor2", &setAngleMotor2);
ros::Subscriber<std_msgs::Float32> set_angle_motor3_sub("set_angle_motor3", &setAngleMotor3);


ros::Subscriber<geometry_msgs::Vector3> pid_tunings_sub("pid_tunings", &pidTuningsCb);

ros::Publisher motor1_monitoring_pub("motor1_monitoring", &motor1_monitoring_msg);
ros::Publisher motor2_monitoring_pub("motor2_monitoring", &motor2_monitoring_msg);
ros::Publisher motor3_monitoring_pub("motor3_monitoring", &motor3_monitoring_msg);

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
    nh.advertise(motor1_monitoring_pub);
    nh.advertise(motor2_monitoring_pub);
    nh.advertise(motor3_monitoring_pub);

    nh.subscribe(motor_pwm_sub);
    nh.subscribe(stop_motor1_sub);
    nh.subscribe(stop_motor2_sub);
    nh.subscribe(stop_motor3_sub);

    nh.subscribe(set_angle_motor1_sub);
    nh.subscribe(set_angle_motor2_sub);
    nh.subscribe(set_angle_motor3_sub);
    nh.subscribe(pid_tunings_sub);
}

void pwmSub(const std_msgs::Float32 &pwm_msg)
{
    motor1_driver.revolute(pwm_msg.data);
    motor1_monitoring_msg.y = pwm_msg.data;
}

void stopMotor1(const std_msgs::Empty &stop_msg)
{
    motor1_setpoint = 0;
    motor1_driver.stop();
}

void stopMotor2(const std_msgs::Empty &stop_msg)
{
    motor2_setpoint = 0;
    motor2_driver.stop();
}

void stopMotor3(const std_msgs::Empty &stop_msg)
{
    motor3_setpoint = 0;
    motor3_driver.stop();
}

void setAngleMotor1(const std_msgs::Float32 &angle_msg){
    motor1_setpoint = angle_msg.data;
    motor1_monitoring_msg.x = motor1_setpoint;
    motor1_driver.setAngle(motor1_setpoint);
}

void setAngleMotor2(const std_msgs::Float32 &angle_msg){
    motor2_setpoint = angle_msg.data;
    motor2_monitoring_msg.x = motor2_setpoint;
    motor2_driver.setAngle(motor2_setpoint);
}

void setAngleMotor3(const std_msgs::Float32 &angle_msg){
    motor3_setpoint = angle_msg.data;
    motor3_monitoring_msg.x = motor3_setpoint;
    motor3_driver.setAngle(motor3_setpoint);
}

void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg)
{
    kp = pid_tunings_msg.x;
    ki = pid_tunings_msg.y;
    kd = pid_tunings_msg.z;
    motor1_driver.setPid(kp, ki, kd);
}

void getMotorInfo(){
    motor1_monitoring_msg.x = motor1_setpoint;
    motor1_monitoring_msg.y = motor1_driver.getAngle();
    motor1_monitoring_msg.z = motor1_encoder.getEncTicks();

    motor2_monitoring_msg.x = motor2_setpoint;
    motor2_monitoring_msg.y = motor2_driver.getAngle();
    motor2_monitoring_msg.z = motor2_encoder.getEncTicks();

    motor3_monitoring_msg.x = motor3_setpoint;
    motor3_monitoring_msg.y = motor3_driver.getAngle();
    motor3_monitoring_msg.z = motor3_encoder.getEncTicks();

    motor1_monitoring_pub.publish(&motor1_monitoring_msg);
    motor2_monitoring_pub.publish(&motor2_monitoring_msg);
    motor3_monitoring_pub.publish(&motor3_monitoring_msg);
}