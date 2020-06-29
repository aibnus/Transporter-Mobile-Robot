#ifndef _master_H_
#define _master_H_
#include <Arduino.h>
#define VL

// Adding wire library to communicate with I2C
#include <Wire.h>
#define slave_add 0x02
//===== Communication Header =====//
////////////////////////////////////
// Define Header For Wifi Communication
#define head_batt       '!'
#define head_led_status '@'
#define head_start      '#'
#define head_pause      '$'
#define head_emergency  '%'

#define head_side       '^'
#define head_chekpoint  '&'
// #define head_cal_done   '*'
#define head_us_test    '('
#define head_servo_test ')'
#define tail            '?'
//Special character to communicate with slave
#define head_down       '*'
#define head_pick       '<'
#define head_put        '>'

char serialIn = ' ';
char serialIn_buffer = ' ';
boolean serial_flag = 0;


//===== Driver Motor =====//
#include <SparkFun_TB6612.h>
//Define motor driver pin
#define PWMA 5
#define AIN2 6
#define AIN1 7
#define BIN1 8
#define BIN2 9
#define PWMB 10

//Make left and right motor object
Motor motorL(AIN1, AIN2, PWMA, 1, 0);
Motor motorR(BIN1, BIN2, PWMB, 1, 0);

#ifdef VL
// Adding VL53L0X library to connect with mini Lidar
#include <VL53L0X.h>
//Define new address to connect multiple module
#define vl_add_s 0x29
#define vl_add_f 0x30
//Define XSHUT pin
#define xshut_s 12
#define xshut_f 11

//Make new object
VL53L0X lox_s;
VL53L0X lox_f;
#elif defined US
// Adding HC-SR04 library
#include <NewPing.h>
//Define HC-SR04 Pin
#define hc_s 12
#define hc_f 11

NewPing us_s(hc_s, hc_s, 99);
NewPing us_f(hc_f, hc_f, 99); 
#endif

#define DISTANCE_FRONT 0
#define DISTANCE_SIDE  1


//===== Line Sensor =====//
///////////////////////////
#define A A2
#define B A1
#define C A0
#define analog_pin A3
#define SENSOR_MID_LEFT     0b11111000
#define SENSOR_MID_RIGHT    0b00011111
#define SENSOR_NONE         0b00000000
#define SENSOR_ALL          0b11111111

const int select_pin[3] = {A, B, C};
int analog_value[8];
int analog_tresh[8] = {110, 130, 110, 120, 140, 130, 120, 160};

//Define color threshold
#define threshold 20

//===== Function =====//
void(* reset_function) (void) = 0;        //declare reset function @ address 0

class Master {
    public:
        Master();
        void serialBuffer();
        void serial_read();

        void VLInit();
        void LineSensInit();

        void move(int16_t _speedL, int16_t _speedR);
        void moveUntil(int16_t _speedL, int16_t _speedR, int _duration);
        void moveLeft(int16_t _speed, int _duration = 300);
        void moveRight(int16_t _speed, int _duration = 300);
        void moveFwd(int16_t _speed, int _duration = 300);
        void moveBwd(int16_t _speed, int _duration = 300);
        void moveUTurnLeft(int16_t _speed, int _duration = 500);
        void moveUTurnRight(int16_t _speed, int _duration = 500);
        void stop();
        
        void setPID(float _Kp, float _Ki, float _Kd);
        
        int readLine();
        void traceLine(int16_t _speed);
        void traceLineUntil(int16_t _speed, uint8_t _sensor_condition);
        
        int readDistanceSide();
        int readDistanceFront();
        void traceWall(int16_t _speed, uint16_t _setPoint, bool _side);
        void traceWallUntil(int16_t _speed, uint16_t _setPoint, bool _side, int _sensor_index, char _operation, int _sensor_value);

        void servo(char _condition);

    private:
        float Kp, Ki, Kd;
        int error = 0;
        int last_error = 0;
        int diff_error = 0;

        int8_t currSpeed;
};

Master::Master() {
    Kp = 15;
    Ki = 0;
    Kd = 100;
}

#ifdef VL
void Master::VLInit() {

    //Set all xshut pin to output
    pinMode(xshut_s, OUTPUT);
    pinMode(xshut_f, OUTPUT);

    //Change Address to every VL sensors
    lox_s.setAddress(vl_add_s);
    pinMode(xshut_s, INPUT);
    delay(10);
    lox_f.setAddress(vl_add_f);
    pinMode(xshut_f, INPUT);
    delay(10);

    lox_s.init();
    lox_f.init();

    lox_s.setTimeout(500);
    lox_f.setTimeout(500);

    // lox_s.setSignalRateLimit(0.1);
    // lox_s.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    // lox_s.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
    // lox_s.setMeasurementTimingBudget(50000);

    // lox_f.setSignalRateLimit(0.1);
    // lox_f.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    // lox_f.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
    // lox_f.setMeasurementTimingBudget(50000);

    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    lox_s.startContinuous();
    lox_f.startContinuous();
}
#endif

void Master::LineSensInit() {
    pinMode(A, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(C, OUTPUT);
}

void Master::move(int16_t _speedL, int16_t _speedR) {
    motorL.drive(_speedL);
    motorR.drive(_speedR);
}

void Master::moveUntil(int16_t _speedL, int16_t _speedR, int _duration) {
    uint32_t startTime = millis();
    while ((int)(millis() - startTime) < _duration) {
        motorL.drive(_speedL);
        motorR.drive(_speedR);
        Master::serialBuffer();
    }
    // delay(_duration);
}

void Master::moveLeft(int16_t _speed, int _duration) {
    delay(50);
    Master::moveUntil(_speed/2, _speed, _duration);
}

void Master::moveRight(int16_t _speed, int _duration) {
    delay(50);
    Master::moveUntil(-_speed, -_speed/2, _duration);
}

void Master::moveFwd(int16_t _speed, int _duration) {
    delay(50);
    Master::moveUntil(-_speed, _speed, _duration);
}

void Master::moveBwd(int16_t _speed, int _duration) {
    delay(50);
    Master::moveUntil(_speed, -_speed, _duration);
}

void Master::moveUTurnLeft(int16_t _speed, int _duration) {
    delay(50);
    Master::moveUntil(-_speed/2, _speed, _duration);
}

void Master::moveUTurnRight(int16_t _speed, int _duration) {
    delay(50);
    Master::moveUntil(-_speed, _speed/3, _duration);
}

void Master::stop() {
    brake(motorL, motorR);
}

void Master::setPID(float _Kp, float _Ki, float _Kd) {
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
}

int Master::readLine() {
    int bit_sensor = 0;

    for (byte pin = 0; pin < 8; pin++) {
        for (int i = 0; i < 3; i++) {
        if (pin & (1 << i)) digitalWrite(select_pin[i], HIGH);
        else digitalWrite(select_pin[i], LOW);
        }
        analog_value[pin] = map(analogRead(analog_pin), 0, 1023, 0, 255);

        if (analog_value[pin] <= analog_tresh[pin]) {
        bit_sensor = bit_sensor | (0b10000000 >> pin);
        }
    }

    return bit_sensor;
}

void Master::traceLine(int16_t _speed) {
    uint8_t sensor = Master::readLine();

    switch (sensor) {
        case 0b00000001: error = -7; break;
        case 0b00000011: error = -6; break;
        case 0b00000010: error = -5; break;
        case 0b00000110: error = -4; break;
        case 0b00000100: error = -3; break;
        case 0b00001100: error = -2; break;
        case 0b00001000: error = -1; break;
        case 0b00011000: error = 0; break;
        case 0b00010000: error = 1; break;
        case 0b00110000: error = 2; break;
        case 0b00100000: error = 3; break;
        case 0b01100000: error = 4; break;
        case 0b01000000: error = 5; break;
        case 0b11000000: error = 6; break;
        case 0b10000000: error = 7; break;
        case 0b00000000: //error = 0; break;
            if (last_error != 0) error = last_error;
            break;
    }

    int rate_error = error - last_error;
    diff_error += error;
    last_error = error;

    int mot_value = (int) (error * Kp) + (diff_error * Ki) + (rate_error * Kd);
  
    // if (currSpeed < _speed) currSpeed += 30;
    // else currSpeed = _speed;

    int mot_l = _speed - mot_value;
    int mot_r = _speed + mot_value;

    mot_l = constrain(mot_l, -125, 255);
    mot_r = constrain(mot_r, -125, 255);

    motorL.drive(-mot_l);
    motorR.drive(mot_r);
}

void Master::traceLineUntil(int16_t _speed, uint8_t _sensor_condition) {
    boolean cross_flag = 0;

    while(!cross_flag) {
        Master::serialBuffer();

        uint8_t sensor = Master::readLine();

        Master::traceLine(_speed);

        if (!cross_flag) {
            if (sensor == _sensor_condition || sensor == 0b00001111) { cross_flag = 1; }
        }
    }
}

#ifdef VL
int Master::readDistanceSide() {
    // Measure distance in cm unit
    int sensor = (lox_f.readRangeContinuousMillimeters() / 10);
    if (sensor >= 99) { sensor = 99; }
    if (lox_f.timeoutOccurred()) { sensor = 99; }

    return sensor;
}

int Master::readDistanceFront() {
    // Measure distance in cm unit
    int sensor = (lox_s.readRangeContinuousMillimeters() / 10);
    if (sensor >= 99) { sensor = 99; }
    if (lox_s.timeoutOccurred()) { sensor = 99; }

    return sensor;
}
#elif defined US
int Master::readDistanceSide() {
    return us_s.ping_cm();
}

int Master::readDistanceFront() {
    return us_f.ping_cm();
}
#endif

void Master::traceWall(int16_t _speed, uint16_t _setPoint, bool _side) {
    int mot_l, mot_r;

    unsigned int distance_side = Master::readDistanceSide();
    unsigned int distance_front = Master::readDistanceFront();

    error = distance_side - _setPoint;

    if (distance_side <= (_setPoint + 5) && distance_front < 25) { error = -30; }
    // if (distance_side > 50) { error = 30; }

    int rate_error = error - last_error;
    diff_error += error;
    last_error = error;

    int mot_value = (int) (error * Kp) + (diff_error * Ki) + (rate_error * Kd);
    // int8_t moveSpeed = Master::speedControl(_speed);
    // currSpeed = moveSpeed;
    // if (currSpeed < _speed) currSpeed += 30;

    if (!_side) {
        mot_l = _speed + mot_value;
        mot_r = _speed - mot_value;
    } else {
        mot_l = _speed - mot_value;
        mot_r = _speed + mot_value;
    }

    mot_l = constrain(mot_l, 30, 255);
    mot_r = constrain(mot_r, 30, 255);

    motorL.drive(-mot_l);
    motorR.drive(mot_r);
}

void Master::traceWallUntil(int16_t _speed, uint16_t _setPoint, bool _side, int _sensor_index, char _operation, int _sensor_value) {
    int sensor;

    if (_operation == '<') {
        while(sensor >= _sensor_value) {
            Master::serialBuffer();

            Master::traceWall(_speed, _setPoint, _side);
        
            if (_sensor_index == 0) { sensor = Master::readDistanceFront(); }
            else if (_sensor_index == 1) { sensor = Master::readDistanceSide(); }
        }
    } else if (_operation == '>') {
        while(sensor <= _sensor_value) {
            Master::serialBuffer();

            Master::traceWall(_speed, _setPoint, _side);
        
            if (_sensor_index == 0) { sensor = Master::readDistanceFront(); }
            else if (_sensor_index == 1) { sensor = Master::readDistanceSide(); }
        }
    }
}

void Master::servo(char _condition) {
    Wire.beginTransmission(slave_add);
    Wire.write(_condition);
    Wire.endTransmission();
}

void Master::serialBuffer() {
    if (Serial.available()) {
        serialIn = Serial.read();
    }

    while (serialIn == head_pause) {
        brake(motorL, motorR);

        Master::serial_read();
    }

    if (serialIn == head_emergency) {
        brake(motorL, motorR);

        //Soft reset arduino
        Wire.beginTransmission(slave_add);
        Wire.write(head_emergency);
        Wire.endTransmission();
        
        reset_function();
    }
}

void Master::serial_read () {
  //Read any incoming serial data from GUI
  if (Serial.available()) {
    serialIn = Serial.read();

    if (serialIn == head_emergency) {
        brake(motorL, motorR);

        //Soft reset arduino
        Wire.beginTransmission(slave_add);
        Wire.write(head_emergency);
        Wire.endTransmission();
        
        reset_function();
    }

    if (serialIn == serialIn_buffer) {  //Check if new serial data is the same as previous one
      //If the same check for serial flag
      if (serial_flag) serial_flag = 0; //Disable flag to stop testing loop
      else serial_flag = 1;             //Change value to 1 to start testing loop
    } else {
      //If new serial data is not the same as previous one
      if (serial_flag) serialIn = serialIn_buffer;  //Resist to change to another tesing loop when another testing loop is still continue
      else {
        serialIn_buffer = serialIn;                 //Change to another tesing loop when another testing loop is disable
        serial_flag = 1;
      }
    }
  }
}

#endif