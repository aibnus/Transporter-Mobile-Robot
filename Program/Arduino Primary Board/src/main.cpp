#include <master.h>
// #include <EEPROM.h>

Master rbt;
//Choose how many object to pick before reaching for finish
#define pick2
// #define pick3

//===== Battery =====//
#define batt_cell 3
float batt_voltage;
int send_batt_voltage;

//===== Serial Communication =====//
char send_distance[10];
bool side, check_point = false;

//===== EEPROM =====//
struct my_object {
  uint8_t black_line[8];
  uint8_t red_line[8];
  uint8_t blue_line[8];
  uint8_t yellow_line[8];
};
my_object ROM;

uint8_t calibrate_sequence = 0;
uint8_t sensor_value;

void setup() {
  delay(1000);
  Serial.begin(9600);
  Wire.begin();

#ifdef VL
  rbt.VLInit();
#endif
  rbt.LineSensInit();
  rbt.setPID(15, 0, 100);

  // Robot will not move if start button is not pressed
  while (serialIn != head_start) {
    // Send battery voltage to GUI
    //Read battery voltage
    //Resistor value R1: 10k, R2: 2.2k. Max voltage: 27.727 V
    //1023 / 27.7 = 36.9  analog resolution / maxvoltage
    //36.9 / 0.08 = 461.25 complementary filter
    //418 = 461.25 + offset calibration (-43.25)
    //Offset calibration being manually check battery voltage and compare it to analog read
    batt_voltage = batt_voltage * 0.92 + ((float)analogRead(A7) / 418);
    Serial.print(head_batt);
    Serial.print(batt_voltage, 2);
    Serial.print(tail);

    rbt.serial_read();

    //Testing loop//
    //In order to change testing loop, user must send the same header twice. And then send another header
    if (serialIn == head_us_test && serial_flag) {
      //Convert all measurement data to 1 single string
      //Send back to GUI
      sprintf(send_distance, "%2d%2d", rbt.readDistanceFront(), rbt.readDistanceSide());

      Serial.print(head_us_test);
      Serial.print(send_distance);
      Serial.print(tail);
    } else if (serialIn == head_servo_test && serial_flag) {
      //User send character through GUI
      //Send command to slave microcontroller
      Wire.beginTransmission(slave_add);
      Wire.write(head_servo_test);
      Wire.endTransmission();

      serialIn = ' ';
    } else if (serialIn == head_chekpoint) {
      check_point = !check_point; serial_flag = 0;      
      serialIn = ' ';
    } else if (serialIn == 'b') { //Right or Blue Side
      side = 0; serial_flag = 0;
      rbt.servo('b');
      serialIn = ' ';
    } else if (serialIn == 'r') { //Left or Red Side
      side = 1; serial_flag = 0;
      rbt.servo('r');
      serialIn = ' ';
    }
  }

  uint32_t soft_start_time = millis();
  //To compensate unstable robot when the first time motor started
  //Slowly accelerate motor to desire speed
  for (int i = 0; i <= 100; i += 30) {
    rbt.moveUntil(-i, i, 50);
  }
  while ((millis()-soft_start_time) < 500);

  if (side == 0 ) {
    if (!check_point) {
      //===
      rbt.traceLineUntil(100, SENSOR_MID_RIGHT);
      rbt.servo(head_down);
      rbt.moveFwd(100, 160);
      rbt.stop();
      rbt.servo(head_pick);
      delay(1000);
      rbt.moveBwd(100, 200);
      rbt.stop();
      rbt.moveRight(100, 400);
      //===
      rbt.traceLineUntil(100, SENSOR_ALL);
      rbt.moveFwd(100, 40);
      rbt.servo(head_down);
      rbt.moveUntil(100, 100, 320);
      rbt.stop();
      rbt.moveBwd(100, 40);
      rbt.stop();
      rbt.servo(head_put);
      delay(1000);
      //===
      rbt.moveUntil(-100, -100, 270);
      rbt.servo(head_down);
      rbt.stop();
      rbt.moveFwd(100, 70);
      rbt.stop();
      rbt.servo(head_pick);
      delay(1000);
      //===
      rbt.moveUntil(-100, -100, 300);
      //===
      rbt.traceLineUntil(100, SENSOR_ALL);
#ifdef pick3        //=== If you want to put all object into places uncomment 7 line below
      rbt.moveFwd(100, 60);
      rbt.servo(head_down);
      rbt.moveUntil(-100, -100, 350);
      rbt.stop();
      rbt.servo(head_put);
      delay(1000);
      rbt.moveUntil(100, 100, 500);
      rbt.stop();
#elif defined pick2 //=== If you want to put only 2 object into places uncomment line below
      rbt.moveLeft(100, 360);
#endif
      //===
      rbt.traceLineUntil(70, SENSOR_MID_RIGHT);
      rbt.moveRight(100, 300);
      //===
      rbt.traceLineUntil(120, SENSOR_MID_LEFT);
#ifdef pick3        //=== If you want to put all object into places uncomment line below
      rbt.moveLeft(100, 350);
#elif defined pick2 //=== If you want to put only 2 object into places uncomment 6 line below
      rbt.servo(head_down);
      rbt.move(-110, 100);
      delay(280 );
      // rbt.moveFwd(100, 240);
      rbt.stop();
      rbt.servo(head_put);
      delay(1000);
      rbt.moveBwd(100, 200);
      rbt.moveUntil(100, 100, 320);
#endif
    }
      //==
      rbt.traceLineUntil(175, SENSOR_MID_LEFT);
      rbt.servo(head_down);
      rbt.moveFwd(100, 60);
      rbt.stop();
      rbt.servo(head_pick);
      delay(800);
#ifdef pick3        //=== If you want to put all object into places uncomment 9 line below
      rbt.moveUntil(100, 100, 500);
      rbt.traceLineUntil(175, SENSOR_MID_LEFT);
      rbt.moveFwd(100, 60);
      rbt.moveUntil(100, 100, 320);
      rbt.servo(head_put);
      delay(1000);
      rbt.moveUntil(100, 100, 300);
      rbt.traceLineUntil(175, SENSOR_MID_LEFT);
      rbt.moveLeft(175, 300);
#elif defined pick2 //=== If you want to put only 2 object into places uncomment 2 line below
      rbt.moveUntil(100, 100, 320);
      rbt.traceLineUntil(150, SENSOR_NONE);
#endif
      //==
      rbt.moveLeft(175, 200);
      rbt.moveFwd(175, 100);
      delay(100);

    //===== Wall Following =====//
    rbt.setPID(3, 0, 15);

    soft_start_time = millis();
    while ((millis() - soft_start_time) < 4400) {
      sensor_value = rbt.readLine();
      rbt.traceWall(150, 15, side);
      rbt.serialBuffer();
    }
    rbt.traceWallUntil(150, 15, side, DISTANCE_SIDE, '>', 50);
    
    // rbt.moveRight(175, 20);
    while (sensor_value == SENSOR_NONE) {
      sensor_value = rbt.readLine();
      rbt.move(-190, 150);
      rbt.serialBuffer();
    }

    //===== Line Tracing =====//
    rbt.setPID(20, 0, 100);
    rbt.traceLineUntil(100, SENSOR_MID_LEFT);
    rbt.moveLeft(100, 400);
    //==
    rbt.traceLineUntil(120, SENSOR_NONE);
    rbt.moveRight(120, 60);
    rbt.moveFwd(120, 600);
    //==
    rbt.setPID(25, 0.0015, 100);
    soft_start_time = millis();
    while ((millis()-soft_start_time) < 4100) {
      rbt.serialBuffer();
      rbt.traceLine(175);
    }
    //==
    rbt.traceLineUntil(100, SENSOR_MID_RIGHT);
    rbt.moveLeft(100, 150);
    rbt.setPID(20, 0, 100);
    rbt.traceLineUntil(175, SENSOR_NONE);
    //===1===
    rbt.moveUntil(-100, 100, 500);
    rbt.traceLineUntil(100, SENSOR_NONE);
    //===2===
    rbt.moveUntil(-100, 80, 500);
    rbt.traceLineUntil(80, SENSOR_NONE);
    //===3===
    rbt.moveUntil(-100, 80, 500);
    rbt.traceLineUntil(60, SENSOR_NONE);
    //===BWD===
    rbt.stop();
    rbt.moveBwd(100, 120);
    // rbt.moveUntil(-185, 178, 1050);
    rbt.stop();
    rbt.servo(head_down);
    delay(400);
    rbt.servo(head_put);
  } else if (side == 1) {   //==================== Left Side or Red Side ====================//
    if (!check_point) {
      //===
      rbt.traceLineUntil(100, SENSOR_MID_LEFT);
      rbt.servo(head_down);
      rbt.moveFwd(100, 180);
      rbt.stop();
      rbt.servo(head_pick);
      delay(800);
      rbt.moveBwd(100, 200);
      rbt.stop();
      rbt.moveLeft(100, 400);
      //===
      rbt.traceLineUntil(100, SENSOR_ALL);
      rbt.moveFwd(100, 60);
      rbt.servo(head_down);
      rbt.moveUntil(-100, -100, 340);
      rbt.stop();
      rbt.servo(head_put);
      delay(1200);
      //===
      rbt.moveUntil(100, 100, 280);
      rbt.servo(head_down);
      rbt.stop();
      rbt.moveFwd(100, 50);
      rbt.stop();
      rbt.servo(head_pick);
      delay(800);
      //===
      rbt.moveUntil(100, 100, 300);
      //===
      rbt.traceLineUntil(100, SENSOR_ALL);
#ifdef pick3        //=== If you want to put all object into places uncomment 7 line below
      rbt.moveFwd(100, 60);
      rbt.servo(head_down);
      rbt.moveUntil(100, 100, 350);
      rbt.stop();
      rbt.servo(head_put);
      delay(1000);
      rbt.moveUntil(-100, -100, 500);
      rbt.stop();
#elif defined pick2 //=== If you want to put only 2 object into places uncomment line below
      rbt.moveRight(100, 380);
#endif
      //===
      rbt.traceLineUntil(80, SENSOR_MID_LEFT);
      rbt.moveLeft(100, 300);
      //===
      rbt.traceLineUntil(110, SENSOR_MID_RIGHT);
#ifdef pick3        //=== If you want to put all object into places uncomment line below
      rbt.moveRight(100, 350);
#elif defined pick2 //=== If you want to put only 2 object into places uncomment 6 line below
      rbt.servo(head_down);
      rbt.moveUntil(-120, 100, 280);
      rbt.stop();
      rbt.servo(head_put);
      delay(1000);
      rbt.moveBwd(100, 200);
      rbt.moveUntil(-100, -100, 300);
#endif
    }
      // ==
      rbt.traceLineUntil(175, SENSOR_MID_RIGHT);
      rbt.servo(head_down);
      rbt.moveFwd(100, 63);
      rbt.stop();
      rbt.servo(head_pick);
      delay(800);
#ifdef pick3        //=== If you want to put all object into places uncomment 9 line below
      rbt.moveUntil(-100, -100, 500);
      rbt.traceLineUntil(175, SENSOR_MID_RIGHT);
      rbt.moveFwd(100, 60);
      rbt.moveUntil(-100, -100, 320);
      rbt.servo(head_put);
      delay(1000);
      rbt.moveUntil(-100, -100, 300);
      rbt.traceLineUntil(175, SENSOR_MID_RIGHT);
      rbt.moveRight(175, 300);
#elif defined pick2 //=== If you want to put only 2 object into places uncomment 2 line below
      rbt.moveUntil(-100, -100, 240);
      rbt.traceLineUntil(150, SENSOR_NONE);
#endif
      //==
      rbt.moveFwd(175, 100);
      rbt.moveRight(175, 200);
      rbt.moveFwd(175, 20);
      delay(100);

    //===== Wall Following =====//
    rbt.setPID(3, 0, 10);

    soft_start_time = millis();
    while ((millis() - soft_start_time) < 4750) {
      rbt.traceWall(150, 15, side);
      rbt.serialBuffer();
    }
    rbt.traceWallUntil(150, 15, side, DISTANCE_SIDE, '>', 50);
    
    // rbt.moveRight(175, 20);
    while (sensor_value == SENSOR_NONE) {
      sensor_value = rbt.readLine();
      rbt.move(-160, 160);
      rbt.serialBuffer();
    }
    
    //===== Line Tracing =====//
    rbt.setPID(20, 0, 100);
    rbt.traceLineUntil(100, SENSOR_MID_RIGHT);
    rbt.moveRight(100, 400);
    //==
    rbt.traceLineUntil(120, SENSOR_NONE);
    rbt.moveLeft(120, 10);
    rbt.moveFwd(120, 600);
    //==
    rbt.setPID(25, 0.0015, 100);
    uint32_t soft_start_time = millis();
    while ((millis()-soft_start_time) < 4300) {
      rbt.serialBuffer();
      rbt.traceLine(175);
    }
    //==
    rbt.traceLineUntil(100, SENSOR_MID_LEFT);
    rbt.moveRight(100, 150);
    rbt.setPID(20, 0, 100);
    rbt.traceLineUntil(175, SENSOR_NONE);
    //===1===
    rbt.moveUntil(-100, 100, 500);
    rbt.traceLineUntil(100, SENSOR_NONE);
    //===2===
    rbt.moveUntil(-100, 80, 500);
    rbt.traceLineUntil(80, SENSOR_NONE);
    //===3===
    rbt.moveUntil(-100, 80, 500);
    rbt.traceLineUntil(60, SENSOR_NONE);
    //===BWD===
    rbt.stop();
    rbt.moveBwd(100, 120);
    // rbt.moveUntil(-185, 178, 1050);
    rbt.stop();
    rbt.servo(head_down);
    delay(400);
    rbt.servo(head_put);
  }
}

void loop() {
  rbt.serialBuffer();
}