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
float setpoint = 0;

Ticker motor_info_ticker;

Encoder M1Encoder
{
    M1_ENCA_PIN,
    M1_ENCB_PIN,
    REDUCTER_RATIO,
    ENCODER_IMPULSES,
    true
};

// TODO
Encoder M2Encoder 
{ 
    M2_ENCA_PIN, 
    M2_ENCB_PIN, 
    REDUCTER_RATIO, 
    ENCODER_IMPULSES, 
    true 
};
// TODO
Encoder M3Encoder 
{
    M3_ENCA_PIN,
    M3_ENCB_PIN,
    REDUCTER_RATIO,
    ENCODER_IMPULSES,
    true 
};

DcServo M1Servo 
{
    M1_IN1_PIN,
    M1_IN2_PIN,
    M1_PWM_PIN,
    &M1Encoder 
};

//TODO
DcServo M2Servo 
{
    M2_IN1_PIN,
    M2_IN2_PIN,
    M2_PWM_PIN,
    &M2Encoder 
};
//TODO
DcServo M3Servo 
{
    M3_IN1_PIN,
    M3_IN2_PIN,
    M3_PWM_PIN,
    &M3Encoder 
};

ros::NodeHandle nh;
geometry_msgs::Vector3 monitoring_msg;

void pwmSub(const std_msgs::Float32 &pwm_msg);
void setAngleMotor1(const std_msgs::Float32 &angle_msg);
void setAngleMotor2(const std_msgs::Float32 &angle_msg);
void setAngleMotor3(const std_msgs::Float32 &angle_msg);
void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg);
void stopMotor(const std_msgs::Empty &stop_msg);
void getMotorInfo(void);
void initRosTopics(void);

ros::Subscriber<std_msgs::Float32> motor_pwm_sub("motor_pwm", &pwmSub);
ros::Subscriber<std_msgs::Empty> stop_motor_sub("stop_motor", &stopMotor);
ros::Subscriber<std_msgs::Float32> motor1_set_angle_sub("set_angle_motor1", &setAngleMotor1);
ros::Subscriber<std_msgs::Float32> motor2_set_angle_sub("set_angle_motor2", &setAngleMotor2);
ros::Subscriber<std_msgs::Float32> motor3_set_angle_sub("set_angle_motor3", &setAngleMotor3);
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
    nh.subscribe(motor1_set_angle_sub);
    nh.subscribe(motor2_set_angle_sub);
    nh.subscribe(motor3_set_angle_sub);
    nh.subscribe(pid_tunings_sub);
}

void pwmSub(const std_msgs::Float32 &pwm_msg)
{
    M1Servo.revolute(pwm_msg.data);
    monitoring_msg.y = pwm_msg.data;
}

void stopMotor(const std_msgs::Empty &stop_msg)
{
    setpoint = 0;
    M1Servo.stop();
}

void setAngleMotor1(const std_msgs::Float32 &angle_msg) {
    // setpoint = angle_msg.data;
    // monitoring_msg.x = setpoint;
    M1Servo.setAngle(setpoint);
}

void setAngleMotor2(const std_msgs::Float32 &angle_msg) {
    // setpoint = angle_msg.data;
    // monitoring_msg.x = setpoint;
    M2Servo.setAngle(setpoint);
}

void setAngleMotor3(const std_msgs::Float32 &angle_msg) {
    // setpoint = angle_msg.data;
    // monitoring_msg.x = setpoint;
    M3Servo.setAngle(setpoint);
}

void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg) {
    kp = pid_tunings_msg.x;
    ki = pid_tunings_msg.y;
    kd = pid_tunings_msg.z;
    M1Servo.setPid(kp, ki, kd);
}

void getMotorInfo(){
    monitoring_msg.x = setpoint;
    monitoring_msg.y = M1Servo.getAngle();
    monitoring_msg.z = M1Servo.getPWM();
    monitoring_pub.publish(&monitoring_msg);
}