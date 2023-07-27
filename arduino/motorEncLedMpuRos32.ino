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
 *
 * MPU6050_DMP6_ESPWiFi.ino
 * https://github.com/soarbear/mpu6050_imu_ros
 */
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>

#define USE_IMU 1

#if USE_IMU == 1
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#endif

#define OLED 0
#if OLED == 1
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED width,  in pixels
#define SCREEN_HEIGHT 64  // OLED height, in pixels
// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

#define RXD2 16
#define TXD2 17

#define DEBUG 0
#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define PWM_PARAM 0  //PWM parameter via yaml
#define PUB_VEL 0    //publish velocity for debugging
#define USE_LED 1    //LED contorl

#define USE_PID 1
#if USE_PID == 1
double trackAdjustValueL = 0.0;
double trackSetpointL = 0.0;
double trackErrorL = 0.0;
double trackAdjustValueR = 0.0;
double trackSetpointR = 0.0;
double trackErrorR = 0.0;

double Kp = 5.0;   //Determines how aggressively the PID reacts to the current amount of error (Proportional)
double Ki = 10.0;  //Determines how aggressively the PID reacts to error over time (Integral)
double Kd = 1.0;   //Determines how aggressively the PID reacts to the change in error (Derivative)

PID trackPIDLeft(&trackErrorL, &trackAdjustValueL, &trackSetpointL, Kp, Ki, Kd, DIRECT);
PID trackPIDRight(&trackErrorR, &trackAdjustValueR, &trackSetpointR, Kp, Ki, Kd, DIRECT);
#endif

// Handles startup and shutdown of ROS
ros::NodeHandle nh;
#if USE_IMU == 1
geometry_msgs::Quaternion quat_msg;
ros::Publisher quat_pub("quaternion", &quat_msg);
#endif

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 36
#define ENC_IN_RIGHT_A 34
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 39
#define ENC_IN_RIGHT_B 35

// Motor A connections, Left
#define enA 32
// Motor B connections, Right
#define enB 33

#define ENA_CH 0
#define ENB_CH 1

#define ain1 26
#define ain2 25
#define bin1 27
#define bin2 14
#define STBY 4

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 32-bit integers
#define encoder_minimum -2147483648
#define encoder_maximum 2147483647

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_B 5
#define BUZZER 13
#define MPU_INT 12

/***********************Enumeration variable***********************/
enum LEDBEHAV {
  ALL_OFF = 0,  //all off
  LEDBACK,      //back on
  LEDLEFT,      //left on
  LEDRIGHT,     //right on
  LEDFRONT,     //left, right on
  ALL_ON        //all on
};

////////////////// Tick Data Publishing Variables and Constants ///////////////
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

#if PUB_VEL == 1
std_msgs::Float32 left_vel_pub;
ros::Publisher leftvelPub("velLeft", &left_vel_pub);
std_msgs::Float32 right_vel_pub;
ros::Publisher rightvelPub("velRight", &right_vel_pub);
#endif

// Time interval for measurements in milliseconds
const int INTERVAL = 30;
long previousMillis = 0;
long currentMillis = 0;

// Number of ticks per wheel revolution. We won't use this in this code.
// Wheel radius in meters. We won't use this in this code.
#define WHEEL_RADIUS (0.033)
#define WHEEL_DIAMETER (WHEEL_RADIUS * 2)
// Distance from center of the left tire to the center of the right tire in m. We won't use this in this code.
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
#define TICKS_PER_REVOLUTION (1860.0)
#define TICKS_PER_METER (TICKS_PER_REVOLUTION / (2.0 * 3.141592 * WHEEL_RADIUS))
#define WHEEL_BASE (0.160)

#if PWM_PARAM == 1
// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.
int K_P = 1125;
// Y-intercept for the PWM-Linear Velocity relationship for the robot
int K_b = 4;
// Turning PWM output (0 = min, 255 = max for PWM values)
// Set maximum and minimum limits for the PWM values
int PWM_MIN = 40;   // about x.xxx m/s
int PWM_MAX = 240;  // about x.xxx m/s
int K_bias = 5;
#else
#define K_P 1125.0
#define K_b 3.5
#define PWM_MIN 40.0   // about 0.05 m/s
#define PWM_MAX 240.0  // about 0.2 m/s
#define K_bias 5.0     // left is slow, then add this bias
#endif

#define PWM_TURN (PWM_MIN)
// How much the PWM value can change each cycle
#define PWM_INCREMENT 1

// Set linear velocity and PWM variable values for each wheel
float velLeftWheel = 0.0;
float velRightWheel = 0.0;
float vLeft = 0.0;
float vRight = 0.0;

int pwmLeftReq = 0;
int pwmRightReq = 0;

// Record the time that the last velocity command was received
float lastCmdVelReceived = 0.0;
bool blinkState = false;

#if USE_IMU == 1
MPU6050 mpu;

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;  // [w, x, y, z]         quaternion container

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void ICACHE_RAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}
#endif

/////////////////////// Tick Data Publishing Functions ////////////////////////
// Increment the number of ticks
void IRAM_ATTR left_wheel_tick() {
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = true;  // Reverse
  } else {
    Direction_left = false;  // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    } else {
      left_wheel_tick_count.data++;
    }
  } else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    } else {
      left_wheel_tick_count.data--;
    }
  }
}

// Increment the number of ticks
void IRAM_ATTR right_wheel_tick() {
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false;  // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    } else {
      right_wheel_tick_count.data++;
    }
  } else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    } else {
      right_wheel_tick_count.data--;
    }
  }
}

/////////////////////// Motor Controller Functions ////////////////////////////
// Calculate the left wheel linear velocity in m/s every time a
// tick count message is rpublished on the /left_ticks topic.
void calc_vel_left_wheel() {

  // Previous timestamp
  static float prevTime = 0.0;

  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;

  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 30000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velLeftWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
#if PUB_VEL == 1
  left_vel_pub.data = velLeftWheel;
#endif
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;

  // Update the timestamp
  prevTime = (millis() / 1000.0);
}

// Calculate the right wheel linear velocity in m/s every time a
// tick count message is published on the /right_ticks topic.
void calc_vel_right_wheel() {

  // Previous timestamp
  static float prevTime = 0.0;

  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;

  if (numOfTicks > 30000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velRightWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
#if PUB_VEL == 1
  right_vel_pub.data = velRightWheel;
#endif
  prevRightCount = right_wheel_tick_count.data;

  // Update the timestamp
  prevTime = (millis() / 1000.0);
}

// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  float vLeft, vRight;

  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis() / 1000.0);

  vLeft = cmdVel.linear.x - cmdVel.angular.z * WHEEL_BASE / 2.0;
  vRight = cmdVel.linear.x + cmdVel.angular.z * WHEEL_BASE / 2.0;

  if (vLeft >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmLeftReq = int(K_P * vLeft + K_b + K_bias);
  } else {
    pwmLeftReq = int(K_P * vLeft - K_b - K_bias);
  }
  if (vRight >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmRightReq = K_P * vRight + K_b;
  } else {
    pwmRightReq = K_P * vRight - K_b;
  }

  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }

#if USE_PID == 1
  //reset and start again PID controller
  trackPIDLeft.SetMode(MANUAL);
  trackAdjustValueL = 0.0;
  trackErrorL = 0.0;
  trackPIDLeft.SetMode(AUTOMATIC);

  trackPIDRight.SetMode(MANUAL);
  trackAdjustValueR = 0.0;
  trackErrorR = 0.0;
  trackPIDRight.SetMode(AUTOMATIC);
#endif  //USE_PID
}

void set_pwm_values() {

  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if (((pwmLeftReq * velLeftWheel < 0) && (pwmLeftOut != 0)) || ((pwmRightReq * velRightWheel < 0) && (pwmRightOut != 0))) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  // Set the direction of the motors
  if (pwmLeftReq > 0) {  // Left wheel forward
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else if (pwmLeftReq < 0) {  // Left wheel reverse
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  } else if (pwmLeftReq == 0 && pwmLeftOut == 0) {  // Left wheel stop
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
  } else {  // Left wheel stop
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
  }

  if (pwmRightReq > 0) {  // Right wheel forward
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else if (pwmRightReq < 0) {  // Right wheel reverse
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  } else if (pwmRightReq == 0 && pwmRightOut == 0) {  // Right wheel stop
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, LOW);
  } else {  // Right wheel stop
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, LOW);
  }

  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  } else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;

  } else {
    // reached calculated PWM, then start PID
#if USE_PID == 1
    // not stop case, run PID
    if (pwmLeftReq != 0) {
      trackErrorL = (velLeftWheel - vLeft) * 100.0;
      if (trackPIDLeft.Compute())  //true if PID has triggered
        pwmLeftOut += trackAdjustValueL;
    }
#endif
  }

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  } else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  } else {
    // reached calculated PWM, then start PID
#if USE_PID == 1
    if (pwmRightReq != 0) {
      trackErrorR = (velRightWheel - vRight) * 100.0;
      if (trackPIDRight.Compute())  //true if PID has triggered
        pwmRightOut += trackAdjustValueR;
    }
#endif
  }

  // Conditional operator to limit PWM output at the maximum
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;

  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  if (pwmLeftOut > 0) {
    // Set the PWM value on the pins
    ledcWrite(ENA_CH, pwmLeftOut);
  } else {
    // Set the PWM value on the pins
    ledcWrite(ENA_CH, -pwmLeftOut);
  }
  if (pwmRightOut > 0) {
    // Set the PWM value on the pins
    ledcWrite(ENB_CH, pwmRightOut);
  } else {
    // Set the PWM value on the pins
    ledcWrite(ENB_CH, -pwmRightOut);
  }
}

void RGB(enum LEDBEHAV ledbehav) {
  switch (ledbehav) {
    case ALL_OFF:
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_B, LOW);
      break;
    case LEDBACK:
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_B, HIGH);
      break;
    case LEDLEFT:
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_B, LOW);
      break;
    case LEDRIGHT:
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_B, LOW);
      break;
    case LEDFRONT:
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_B, LOW);
      break;
    case ALL_ON:
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, HIGH);
      digitalWrite(LED_B, HIGH);
      break;
    default:
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_B, LOW);
      break;
  }
}

void ledcb(const std_msgs::Int16& msg) {
  RGB(LEDBEHAV(msg.data));  // set the pin state to the message data
}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);
ros::Subscriber<std_msgs::Int16> subLed("rgbled", &ledcb);

void setup() {
#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif

#if (PWM_PARAM == 1)
  int pwm_constants[4];
  char buf[3];  //for debugging
#endif

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
  digitalWrite(STBY, HIGH);

  ledcSetup(ENA_CH, 5000, 8);  //enA, channel: 0, 5000Hz, 8bits = 256(0 ~ 255)
  ledcSetup(ENB_CH, 5000, 8);  //enB, channel: 1, 5000Hz, 8bits = 256(0 ~ 255)

  ledcAttachPin(enA, ENA_CH);
  ledcAttachPin(enB, ENB_CH);

  // Set the motor speed
  ledcWrite(ENA_CH, 0);
  ledcWrite(ENB_CH, 0);

  DEBUG_PRINTLN("SYSTEM Started");

#if USE_LED == 1
  pinMode(LED_L, OUTPUT);  // RGB color lights red control pin configuration output
  pinMode(LED_R, OUTPUT);  // RGB color light green control pin configuration output
  pinMode(LED_B, OUTPUT);  // RGB color light blue control pin configuration output
  RGB(ALL_OFF);            // RGB LED all off
#endif

  pinMode(BUZZER, OUTPUT);

#if USE_IMU == 1
  mpu_setup();
#endif

#if (OLED == 1)
  // initialize OLED display with I2C address 0x3C
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();           // clear display
  oled.setTextSize(1);           // set text size
  oled.setTextColor(WHITE);      // set text color
  oled.setCursor(0, 0);          // set position to display
  oled.println("Jessicar II+");  // set text
  oled.display();                // display on OLED
#endif

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);

#if USE_IMU == 1
  nh.advertise(quat_pub);
#endif

#if PUB_VEL == 1
  nh.advertise(rightvelPub);
  nh.advertise(leftvelPub);
#endif

#if USE_LED == 1
  nh.subscribe(subLed);
#endif

  while (!nh.connected()) {
    nh.spinOnce();
  }

  DEBUG_PRINTLN("\nROS connected");

#if PWM_PARAM == 1
  nh.logwarn("S");
  if (!nh.getParam("/pwmConstants", pwm_constants, 4)) {
    nh.logwarn("F");

  } else {
    K_P = pwm_constants[0];
    K_b = pwm_constants[1];
    PWM_MIN = pwm_constants[2];
    PWM_MAX = pwm_constants[3];

    DEBUG_PRINTLN("PWM parameters:");
    DEBUG_PRINT(K_P);
    DEBUG_PRINT(",");
    DEBUG_PRINT(K_b);
    DEBUG_PRINT(",");
    DEBUG_PRINT(PWM_MIN);
    DEBUG_PRINT(",");
    DEBUG_PRINTLN(PWM_MAX);
  }
#endif

#if USE_PID == 1
  trackPIDLeft.SetMode(AUTOMATIC);
  trackPIDLeft.SetSampleTime(200);
  trackPIDLeft.SetOutputLimits(-20, 20);
  trackPIDRight.SetMode(AUTOMATIC);
  trackPIDRight.SetSampleTime(200);
  trackPIDRight.SetOutputLimits(-20, 20);
#endif
}

void loop() {
  nh.spinOnce();

  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    DEBUG_PRINT("LT:");
    DEBUG_PRINT(int(left_wheel_tick_count.data));
    DEBUG_PRINT(",RT:");
    DEBUG_PRINTLN(int(right_wheel_tick_count.data));

    // Publish tick counts to topics
    leftPub.publish(&left_wheel_tick_count);
    rightPub.publish(&right_wheel_tick_count);
#if PUB_VEL == 1
    leftvelPub.publish(&left_vel_pub);
    rightvelPub.publish(&right_vel_pub);
#endif
    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    mpu_loop();

    //buzzer beep if robot move reverse
    if ((velLeftWheel < 0.0) || (velRightWheel < 0.0))
      digitalWrite(BUZZER, HIGH);
    else
      digitalWrite(BUZZER, LOW);
  }

  // Stop the car if there are no cmd_vel messages
  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}

void mpu_setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  DEBUG_PRINTLN(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(MPU_INT, INPUT);

  // verify connection
  DEBUG_PRINTLN(F("Testing device connections..."));
  DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  DEBUG_PRINTLN(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    DEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    DEBUG_PRINTLN(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    DEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    DEBUG_PRINT(F("DMP Initialization failed (code "));
    DEBUG_PRINT(devStatus);
    DEBUG_PRINTLN(F(")"));
  }
}

void mpu_loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    DEBUG_PRINT("quat\t");
    DEBUG_PRINT(q.w);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(q.x);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(q.y);
    DEBUG_PRINT("\t");
    DEBUG_PRINTLN(q.z);
    quat_msg.w = q.w;
    quat_msg.x = q.x;
    quat_msg.y = q.y;
    quat_msg.z = q.z;

    quat_pub.publish(&quat_msg);
  }
}
