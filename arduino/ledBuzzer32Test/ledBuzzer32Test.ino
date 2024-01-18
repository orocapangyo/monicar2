/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 * motor_controller_diff_drive_2.ino.
 */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED width,  in pixels
#define SCREEN_HEIGHT 64  // OLED height, in pixels

// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_B 5
#define BUZZER 13

/***********************Enumeration variable***********************/
enum LEDBEHAV {
  ALL_OFF = 0,  //all off
  LEDBACK,      //back on
  LEDLEFT,      //left on
  LEDRIGHT,     //right on
  LEDFRONT,     //left, right on
  ALL_ON        //all on
};


// Time interval for measurements in milliseconds
const int INTERVAL = 300;
long previousMillis = 0;
long currentMillis = 0;

// Record the time that the last velocity command was received
float lastCmdVelReceived = 0.0;
bool blinkState = false;

void RGB(enum LEDBEHAV ledbehav) {
  switch (ledbehav) {
    case ALL_OFF:
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_B, HIGH);
      break;
    case LEDBACK:
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_B, LOW);
      break;
    case LEDLEFT:
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_B, HIGH);
      break;
    case LEDRIGHT:
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_B, HIGH);
      break;
    case LEDFRONT:
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_B, HIGH);
      break;
    case ALL_ON:
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_B, LOW);
      break;
    default:
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_B, HIGH);
      break;
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("OLED, LED, Buzzer Test");

  // initialize OLED display with I2C address 0x3C
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  pinMode(LED_L, OUTPUT);  // RGB color lights red control pin configuration output
  pinMode(LED_R, OUTPUT);  // RGB color light green control pin configuration output
  pinMode(LED_B, OUTPUT);  // RGB color light blue control pin configuration output
  RGB(ALL_OFF);            // RGB LED all off

  pinMode(BUZZER, OUTPUT);

  Serial.println("ESP32 start");

  oled.clearDisplay();                    // clear display
  oled.setTextSize(1);                    // set text size
  oled.setTextColor(WHITE);               // set text color
  oled.setCursor(0, 0);                   // set position to display
  oled.println("ESP32 Start");            // set text
  oled.setCursor(0, 16);                  // set position to display
  oled.println("OLED, LED,Buzzer Test");  // set text
  oled.display();                         // display on OLED
}

void loop() {

  // Record the time
  currentMillis = millis();

  //input 0,1,2,3,4,5: LED control
  //input 10, 11 Buzzer on/off
  if (Serial.available() > 0) {

    int val = Serial.parseInt();
    Serial.println(val);
    if (val < 10)
      RGB(LEDBEHAV(val));
    else
      digitalWrite(BUZZER, val - 10);
  }

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }
}
