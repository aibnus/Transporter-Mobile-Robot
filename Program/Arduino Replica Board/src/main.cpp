#include "slave.h"
/*Project Mechatronics

  Slave Microcontroller
  Arduino Nano

  Pin Configuration:
  D4-D6 --> Servo
  D3    --> LED Pin
  A3    --> Photodiode Input
  A4,A5 --> I2C
*/
//Define color threshold
// #include <EEPROM.h>
// #define black_line 12
// #define red_line 30
// #define blue_line 45
// #define white_line 100

Slave servo;

bool rotateFlag = 0;
uint8_t desire_color;
int pd_value, sens;

struct my_object {
  uint8_t red_obj;
  uint8_t blue_obj;
};
my_object ROM;

void(* reset_function) (void) = 0;                  //declare reset function @ address 0

void setup() {
  servo.Init();
  Serial.begin(9600);

  desire_color = 83;

  digitalWrite(led_en, LOW);
  delay(1000);
}

void loop() {
  // servo.lift(DOWN);
  // servo.arm(CLOSE);
  // servo.lift(UP);
  // delay(500);
  // digitalWrite(led_en, HIGH);
  // for (int i = 0; i < 100; i++) {
  //   pd_value += map(analogRead(analog_in), 0, 1023, 0, 255);
  //   delay(3);
  // }
  // pd_value /= 100;
  // int sens = abs(pd_value - (desire_color-threshold));
  // if (sens <= threshold * 2) {
  //   servo_rotate.write(SERVO_ROTATE_MAX);
  // }
  // while(1) {
  //   Serial.print(desire_color); Serial.print("\t");
  //   Serial.print(servo.readSensor()); Serial.print("\t");
  //   Serial.print(sens); Serial.print("\t");
  //   Serial.println(pd_value);
  // }

  switch (c) {
    case 'r': c = ' '; desire_color = 35; break;
    case 'b': c = ' '; desire_color = 83; break;
    case head_servo_test:
      c = ' ';
      //Servo Testing Sequence
      servo.servoTestSequence();
      break;

    case head_down:
      c = ' ';
      servo.lift(DOWN);
      digitalWrite(led_en, !digitalRead(led_en));

      break;
    case head_pick:
      c = ' ';
      //Pickup Sequence
      servo.arm(CLOSE);
      delay(200);
      servo.lift(UP);

      for (int i = 0; i < 100; i++) {
        pd_value += map(analogRead(analog_in), 0, 1023, 0, 255);
      }
      pd_value /= 100;
      sens = abs(pd_value - (desire_color-threshold));
      
      //If the object's color is corresponding to robot side, gripper turn it to 180 degree
      if (sens <= threshold * 2) {
        servo_rotate.write(SERVO_ROTATE_MAX);
        rotateFlag = 1;
      }
      break;

    case head_put:
      c = ' ';
      //===Ketika robot sudah berada pada posisi===//
      servo.arm(OPEN);
      servo_arm.write(SERVO_ARM_MIN);
      servo_arm.write(SERVO_ARM_MIN);
      servo.lift(UP);
      
      //If gripper arm rotate 180 degree. It will rotate it back to default position
      delay(150);
      if (rotateFlag) { 
        servo_rotate.write(SERVO_ROTATE_MIN);
        rotateFlag = 0;
      }
      break;

    case head_emergency:
      //Soft reset
      reset_function();
      break;
  }
}

void receive_event (int bytes) {
  c = Wire.read();

  Serial.println(c);
}