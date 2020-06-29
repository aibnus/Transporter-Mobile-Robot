#include <Arduino.h>
/* Mechatronics Project
   Wifi ESP8266 GUI
*/

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// To Communicate with Arduino
#define BLYNK_PRINT Serial

// Define Virtual Pin
//Home Page
#define battery           V0
#define led_status        V1
#define button_start      V2
#define button_pause      V3
#define button_emergency  V4

#define display           V5

//Setup Page
#define button_side       V6
#define button_checkpoint V7
// #define button_cal_done   V8
#define button_us_test    V9
#define button_servo_test V10
#define button_cancel     V20

// Read Data from Arduino (Serial)
char c;
String color_0, color_1, color_2, color_3;
String color_4, color_5, color_6, color_7;
String us1, us2, us3;
String read_batt;
String xData;
boolean head_flag = 0;
boolean button_flag = 0;
uint8_t calibrate_sequence = 1;

// Defined Header
#define head_batt       '!'
#define head_led_status '@'
#define head_start      '#'
#define head_pause      '$'
#define head_emergency  '%'

#define head_side       '^'
#define head_checkpoint '&'
// #define head_cal_done   '*'
#define head_us_test    '('
#define head_servo_test ')'
#define tail            '?'

// char auth[] = "PGtOk7CD4D2vxqxbWJ3EV7D41qN45KQf";
// char auth[] = "MS9nvwFgxgrOvqhxd0gavUghxB8_StRB";
// char auth[] = "d3bd72421dae42c89078d0eae565b60d";
char auth[] = "4VqvNLhXJm8HKMU7USn6F_E4vQfXK_XQ";

// Wifi and Password of Your Network (Mobile Hotspot is recommended)
// char ssid[] = "Nokia 6";
// char pass[] = "projectmecha";
// Laptop's Wifi Hotspot
char ssid[] = "kyk biasanya";
char pass[] = "subhanallah";
// Emak's Wifi Hotspot
// char ssid[] = "iPhone";
// char pass[] = "12345ABC";

WidgetLCD lcd(display);
WidgetLED led(led_status);

// Define Color of LED
#define BLYNK_GREEN     "#23C48E"
#define BLYNK_BLUE      "#04C0F8"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"
#define BLYNK_DARK_BLUE "#5F7CD8"

void setup() {
  Serial.begin(9600);
  delay(1000);
  // Blynk.begin(auth, ssid, pass);
  // Blynk.begin(auth, ssid, pass, IPAddress(182.168.1.100), 8080);
  Blynk.begin(auth, ssid, pass, IPAddress(10,252,134,120), 8080);    //IPAddress(10,252,134,120)
  led.off();
  led.on();
  Blynk.setProperty(led_status, "color", BLYNK_RED);

  lcd.clear();
  lcd.print(0, 0, "    Project     ");
  lcd.print(0, 1, "  Mechatronics  ");

  Blynk.syncVirtual(button_side);
  Blynk.syncVirtual(button_checkpoint);
}

void loop() {
  if (Serial.available()) {
    c = Serial.read();

    if (head_flag == 0) {
      if (c == head_batt || c == head_us_test) {
        xData = c;
        head_flag = 1;
      }
    } else {
      if (c == tail) {
        switch (xData[0]) {
          case head_batt:   //Reading battery percentage
            //Put all battery array value to one single string
            read_batt = ((char)xData[1]);
            for (int i = 2; i < sizeof(xData); i++) {
              read_batt += ((char)xData[i]);
            }
            // read_batt = ((char)xData[1]);
            // read_batt += ((char)xData[2]);
            // read_batt += ((char)xData[3]);

            Blynk.virtualWrite(battery, read_batt);
            break;

          case head_us_test:
            us1 = (char)xData[1];
            us1 += (char)xData[2];

            us2 = (char)xData[3];
            us2 += (char)xData[4];

            lcd.print(7, 0, us1);
            lcd.print(7, 1, us2);
            break;

        }
        head_flag = 0;
      } else {
        if (xData.length() > 16) head_flag = 0;
        xData += c;
      }
    }
  }

  Blynk.run();
}


//Get button state from server in case
//your hardware was disconnected
// BLYNK_CONNECTED() {
//   Blynk.syncVirtual(button_side);
//   Blynk.syncVirtual(button_checkpoint);
// }

//Start Page
BLYNK_WRITE(button_start) {
  Serial.print(head_start);

  Blynk.setProperty(led_status, "color", BLYNK_GREEN);
}
BLYNK_WRITE(button_pause) {
  Serial.print(head_pause);

  Blynk.setProperty(led_status, "color", BLYNK_YELLOW);
}
BLYNK_WRITE(button_emergency) {
  int button_emergency_state = param.asInt();

  if (button_emergency_state == 1) { 
    Serial.print(head_emergency);

    Blynk.setProperty(led_status, "color", BLYNK_RED);

    Blynk.virtualWrite(button_start, 2);
    Blynk.virtualWrite(button_pause, 2);
    Blynk.virtualWrite(button_checkpoint, 2);
    Blynk.virtualWrite(button_side, 1);
  }
}

BLYNK_WRITE(button_checkpoint) {
  int button_cal_state = param.asInt();

  Serial.print(head_checkpoint);

  if (button_cal_state == 1) {
    Blynk.setProperty(led_status, "color", BLYNK_BLUE);
  } else if (button_cal_state == 0) {
    Blynk.setProperty(led_status, "color", BLYNK_RED);
  }
}

//Setup Page
BLYNK_WRITE(button_side) {
  switch(param.asInt()) {
    case 1:
      Serial.print('b'); break;
    case 2:
      Serial.print('r'); break;
  }
}

BLYNK_WRITE(button_us_test) {
  int button_us_state = param.asInt();

  Serial.print(head_us_test);

  if (button_us_state == 1) {
    lcd.clear();
    lcd.print(0, 0, "Front: ");
    lcd.print(0, 1, "Side : ");
  } else if (button_us_state == 0) {
    lcd.clear();
    lcd.print(0, 0, "    Project     ");
    lcd.print(0, 1, "  Mechatronics  ");
  }
}

BLYNK_WRITE(button_servo_test) {
  int button_servo_state = param.asInt();

  Serial.print(head_servo_test);

  if (button_servo_state == 1) {
    lcd.clear();
    lcd.print(0, 0, " Testing Servo  ");
  } else if (button_servo_state == 0) {    
    lcd.clear();
    lcd.print(0, 0, "    Project     ");
    lcd.print(0, 1, "  Mechatronics  ");
  }
}

BLYNK_WRITE(button_cancel) {
  lcd.clear();
  lcd.print(0, 0, "    Project     ");
  lcd.print(0, 1, "  Mechatronics  ");

  Blynk.virtualWrite(button_us_test, 2);
  Blynk.virtualWrite(button_servo_test, 2);
}