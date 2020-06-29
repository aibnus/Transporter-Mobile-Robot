#ifndef _slave_H_
#define _slave_H_
#include <Arduino.h>

//Adding wire library to communnicate with I2C
#include <Wire.h>
#define slave_add 0x02
// Define Header For Communication
#define head_start      '#'
#define head_emergency  '%'
#define head_checkpoint '&'
// #define head_cal_done   '*'
#define head_servo_test ')'

#define head_down       '*'
#define head_pick       '<'
#define head_put        '>'

char c;

#include <Servo.h>
Servo servo_arm;
Servo servo_rotate;
Servo servo_lift;

//Define min and max value for each servo
#define SERVO_ARM_MIN     60    //Arm Open
#define SERVO_ARM_MAX     0     //Arm Close
#define SERVO_ROTATE_MIN  168   //Default Posisiton
#define SERVO_ROTATE_MAX  14    //180 degree Position
#define SERVO_LIFT_MIN    17    //Arm Down
#define SERVO_LIFT_MAX    68    //Arm Up

//Define posision of servo
#define OPEN 0
#define CLOSE 1
#define DEFAULT 0
#define ROTATE 1
#define DOWN 0
#define UP 1

bool pick_flag = 0;
bool ready_flag = 0;

#define threshold 8

//Define color sensor led enable
#define led_en 3
//Define photodiode analog input
#define analog_in A7

//===== Function =====//
void receive_event(int bytes);

class Slave {
    public:
        Slave();
        void Init();
        
        void arm(bool position);
        void rotate(bool position);
        void lift(bool position);

        void servoTestSequence();

        uint8_t getArm();
        uint8_t getRotate();
        uint8_t getLift();
        
        uint8_t readSensor();

    private:
        void servoDrive(Servo myServo, int pos1, int pos2);
};

Slave::Slave() {

}

void Slave::Init() {
    Wire.begin(slave_add);
    Wire.onReceive(receive_event);
    
    servo_arm.attach(6);
    servo_rotate.attach(5);
    servo_lift.attach(4);

    //Write default servo position
    servo_arm.write(SERVO_ARM_MIN);
    servo_rotate.write(SERVO_ROTATE_MIN);
    servo_lift.write(SERVO_LIFT_MAX);

    pinMode(led_en, OUTPUT);
}

void Slave::arm(bool position) {
    if (!position) {
        Slave::servoDrive(servo_arm, SERVO_ARM_MAX, SERVO_ARM_MIN);     //Arm open
    } else {
        Slave::servoDrive(servo_arm, SERVO_ARM_MIN, SERVO_ARM_MAX);     //Arm close
    }
}

void Slave::rotate(bool position) {
    if (!position) {
        Slave::servoDrive(servo_rotate, SERVO_ROTATE_MAX, SERVO_ROTATE_MIN);    //Arm rotate CW
    } else {
        Slave::servoDrive(servo_rotate, SERVO_ROTATE_MIN, SERVO_ROTATE_MAX);    //Arm rotate CCW
    }
}

void Slave::lift(bool position) {
    if (!position) {
        Slave::servoDrive(servo_lift, SERVO_LIFT_MAX, SERVO_LIFT_MIN);    //Arm rotate CW
    } else {
        Slave::servoDrive(servo_lift, SERVO_LIFT_MIN, SERVO_LIFT_MAX);    //Arm rotate CCW
    }   
}

void Slave::servoTestSequence() {
    //Servo testing sequence
    Slave::lift(DOWN);
    Slave::arm(CLOSE);
    Slave::lift(UP);
    Slave::rotate(ROTATE);
    Slave::lift(DOWN);
    Slave::arm(OPEN);
    Slave::lift(UP);
    Slave::rotate(DEFAULT);
}

uint8_t Slave::getArm() { return servo_arm.read(); }
uint8_t Slave::getRotate() { return servo_rotate.read(); }
uint8_t Slave::getLift() { return servo_lift.read(); }

void Slave::servoDrive(Servo myServo, int pos1, int pos2) {
    if (pos1 < pos2) {
        for (int i = pos1; i <= pos2; i += 2) {
            myServo.write(i);
            delay(15);
        }
    } else {
        for (int i = pos1; i >= pos2; i -= 2) {
            myServo.write(i);
            delay(15);
        }
    }
}

uint8_t Slave::readSensor() {
    //Reading color and send to master
    digitalWrite(led_en, HIGH);
    uint8_t pd_front = map(analogRead(analog_in), 0, 1023, 0, 255);

    return pd_front;
}

#endif