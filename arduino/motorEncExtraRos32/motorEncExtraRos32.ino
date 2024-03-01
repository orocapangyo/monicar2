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
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <PID_v1.h>
#include <geometry_msgs/msg/quaternion.h>

#include "songlcdled.h"

#define DOMAINID 108

#define MOTOR_60RPM 1
#define MOTOR_178RPM 2
#define MOTOR_TYPE MOTOR_60RPM

#define PRINT_VEL 0
#define PRINT_PIDERR 0
#define PRINT_AVGPWM 0
#define PRINT_INSPWM 0

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

// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;

rcl_publisher_t left_pub, right_pub, quat_pub;
rcl_subscription_t cmd_vel_sub, ledSub, songSub, lcdSub;
std_msgs__msg__Int32 msg_right_tick_count, msg_left_tick_count;
std_msgs__msg__Int32 ledMsg, songMsg, lcdMsg;
geometry_msgs__msg__Twist cmd_vel;
geometry_msgs__msg__Quaternion quat_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define RXD2 16
#define TXD2 17

#define DEBUG 1
#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTY(x, y) Serial2.print(x, y)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

double trackAdjustValueL = 0.0;
double trackSetpointL = 0.0;
double trackErrorL = 0.0;
double trackAdjustValueR = 0.0;
double trackSetpointR = 0.0;
double trackErrorR = 0.0;

double Kp = 0.1;   //Determines how aggressively the PID reacts to the current amount of error (Proportional)
double Ki = 2.0;  //Determines how aggressively the PID reacts to error over time (Integral)
double Kd = 0.0;   //Determines how aggressively the PID reacts to the change in error (Derivative)

PID trackPIDLeft(&trackErrorL, &trackAdjustValueL, &trackSetpointL, Kp, Ki, Kd, DIRECT);
PID trackPIDRight(&trackErrorR, &trackAdjustValueR, &trackSetpointR, Kp, Ki, Kd, DIRECT);

// Encoder output to Arduino Interrupt pin. Tracks the tick count
#if MOTOR_TYPE == MOTOR_60RPM
#define ENC_IN_LEFT_A 36
#define ENC_IN_RIGHT_A 34
#define ENC_IN_LEFT_B 39
#define ENC_IN_RIGHT_B 35
#else
#define ENC_IN_LEFT_A 39
#define ENC_IN_RIGHT_A 35
#define ENC_IN_LEFT_B 36
#define ENC_IN_RIGHT_B 34
#endif
// Motor A, B control
#define AIN1 26
#define AIN2 25
#define BIN1 27
#define BIN2 14

// Motor speed control via PWM
#define ENA 32
#define ENB 33
#define ENA_CH 0
#define ENB_CH 1

#define STBY 4

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 32-bit integers
#define encoder_minimum -2147483648
#define encoder_maximum 2147483647

#define MPU_INT 12

long previousMillis = 0;
long currentMillis = 0;

#define INTERVAL 50  //50ms, so 20Hz

#if MOTOR_TYPE == MOTOR_60RPM
#define K_P 1153.0
#define K_b 10
#define PWM_MIN 44.0   // about 0.03 m/s
#define PWM_MAX 194.0  // about 0.16 m/s
#define TICKS_PER_REVOLUTION (1860.0)
#define K_Lbias (0.0)
#else
#define K_P 431.0
#define K_b 5
#define PWM_MIN 22.0   // about 0.04m/s
#define PWM_MAX 82.0   // about 0.18 m/s
#define TICKS_PER_REVOLUTION (620.0)
#define K_Lbias (-3)
#endif

// Number of ticks per wheel revolution. We won't use this in this code.
// Wheel radius in meters. We won't use this in this code.
#define WHEEL_RADIUS (0.033)
#define WHEEL_DIAMETER (WHEEL_RADIUS * 2)
// Distance from center of the left tire to the center of the right tire in m. We won't use this in this code.
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
#define TICKS_PER_METER (TICKS_PER_REVOLUTION / (2.0 * 3.141592 * WHEEL_RADIUS))
#define WHEEL_BASE (0.160)

// Set linear velocity and PWM variable values for each wheel
float velLeftWheel = 0.0;
float velRightWheel = 0.0;
float vLeft = 0.0;
float vRight = 0.0;

int pwmLeftReq = 0;
int pwmRightReq = 0;

// Record the time that the last velocity command was received
float lastCmdVelReceived = 0.0;

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

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      char buffer[40]; \
      sprintf(buffer, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      DEBUG_PRINTLN(buffer); \
    } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      char buffer[40]; \
      sprintf(buffer, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      DEBUG_PRINTLN(buffer); \
    } \
  }

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

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
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    } else {
      left_wheel_tick_count++;
    }
  } else {
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count= encoder_maximum;
    } else {
      left_wheel_tick_count--;
    }
  }
  msg_left_tick_count.data = left_wheel_tick_count;
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

    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    } else {
      right_wheel_tick_count++;
    }
  } else {
    if (right_wheel_tick_count == encoder_minimum) {
      right_wheel_tick_count = encoder_maximum;
    } else {
      right_wheel_tick_count--;
    }
  }
  msg_right_tick_count.data = right_wheel_tick_count;
}

// Calculate the left wheel linear velocity in m/s every time a
// tick count message is rpublished on the /left_ticks topic.
void calc_vel() {
  int numOfTicks;
  // Previous timestamp
  static float prevTime = 0.0;
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
  static int prevRightCount = 0;
#if PRINT_AVGVEL == 1
  float average;
#endif

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  numOfTicks = left_wheel_tick_count - prevLeftCount;

  // Calculate wheel velocity in meters per second
  velLeftWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  numOfTicks = right_wheel_tick_count - prevRightCount;

  // Calculate wheel velocity in meters per second
  velRightWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
  prevRightCount = right_wheel_tick_count;

  // Update the timestamp
  prevTime = (millis() / 1000.0);

#if PRINT_VEL == 1
  //DEBUG_PRINT("PWM:");
  //DEBUG_PRINT(pwmReq);
  DEBUG_PRINT("L:");
  DEBUG_PRINT(velLeftWheel,3);
  DEBUG_PRINT(",R:");
  DEBUG_PRINTLN(velRightWheel,3);
#endif

#if PRINT_AVGVEL == 1
  // subtract the last reading:
  totalL = totalL - avgL[readIndex];
  // avgL from the sensor:
  avgL[readIndex] = velLeftWheel*1000;
  // add the reading to the total:
  totalL = totalL + avgL[readIndex];

  // calculate the average:
  average = totalL / NUM_READ;
  DEBUG_PRINT("aL:");
  DEBUG_PRINT(average);

  // subtract the last reading:
  totalR = totalR - avgR[readIndex];
  // read from the sensor:
  avgR[readIndex] = velRightWheel*1000;
  // add the reading to the total:
  totalR = totalR + avgR[readIndex];

  // calculate the average:
  average = totalR / NUM_READ;
  DEBUG_PRINT(",aR:");
  DEBUG_PRINTLN(average);

  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= NUM_READ) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
#endif
}

// Take the velocity command as input and calculate the PWM values.
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *cmdVel = (const geometry_msgs__msg__Twist *)msgin;

  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis() / 1000.0);

  vLeft = cmdVel->linear.x - cmdVel->angular.z * WHEEL_BASE / 2.0;
  vRight = cmdVel->linear.x + cmdVel->angular.z * WHEEL_BASE / 2.0;

  if (vLeft >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmLeftReq = int(K_P * vLeft + K_b + K_Lbias);
  } else {
    pwmLeftReq = int(K_P * vLeft - K_b - K_Lbias);
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

  //reset and start again PID controller
  trackPIDLeft.SetMode(MANUAL);
  trackAdjustValueL = 0.0;
  trackErrorL = 0.0;
  trackPIDLeft.SetMode(AUTOMATIC);

  trackPIDRight.SetMode(MANUAL);
  trackAdjustValueR = 0.0;
  trackErrorR = 0.0;
  trackPIDRight.SetMode(AUTOMATIC);
}

void set_pwm_values() {
  int pwm_inc;
  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  // Set the direction of the motors
  if (pwmLeftReq > 0) {  // Left wheel forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmLeftReq < 0) {  // Left wheel reverse
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else if (pwmLeftReq == 0 && pwmLeftOut == 0) {  // Left wheel stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  } else {  // Left wheel stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  if (pwmRightReq > 0) {  // Right wheel forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (pwmRightReq < 0) {  // Right wheel reverse
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else if (pwmRightReq == 0 && pwmRightOut == 0) {  // Right wheel stop
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  } else {  // Right wheel stop
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }

#if PRINT_INSPWM == 1
  DEBUG_PRINT("rL:");
  DEBUG_PRINT(pwmLeftReq);
  DEBUG_PRINT(",rR:");
  DEBUG_PRINTLN(pwmRightReq);
#endif

  if ((abs(pwmLeftReq) - pwmLeftOut) > 16)
    pwm_inc = 8;
  else if ((pwmLeftOut - abs(pwmLeftReq)) > 16)
    pwm_inc = 8;
  else
    pwm_inc = 1;

  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += pwm_inc;
  } else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= pwm_inc;
  } else {
    // reached calculated PWM, then start PID
    // not stop case, run PID
    if (pwmLeftReq != 0) {
      trackErrorL = (velLeftWheel - vLeft) * 10.0;
      if (trackPIDLeft.Compute())  //true if PID has triggered
        pwmLeftOut += trackAdjustValueL;
    }
  }

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += pwm_inc;
  } else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= pwm_inc;
  } else {
    // reached calculated PWM, then start PID

    if (pwmRightReq != 0) {
      trackErrorR = (velRightWheel - vRight) * 10.0;
      if (trackPIDRight.Compute())  //true if PID has triggered
        pwmRightOut += trackAdjustValueR;
    }
  }

  // Conditional operator to limit PWM output at the maximum
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;

#if PRINT_INSPWM == 1
  DEBUG_PRINT(",oL:");
  DEBUG_PRINT(pwmLeftOut);
  DEBUG_PRINT(",oR:");
  DEBUG_PRINT(pwmRightOut);
#endif

  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

#if PRINT_PIDERR == 1
  DEBUG_PRINT("sErr:");
  DEBUG_PRINTY(trackErrorL, 3);
  DEBUG_PRINT(":");
  DEBUG_PRINTLN(trackErrorR, 3);
#endif

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

void subled_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  RGB(int(msg->data));
}

void subsong_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  playsong(int(msg->data));
}

void sublcd_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  showAnimation(int(msg->data));
}

void setup() {
  int i;
#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif
  DEBUG_PRINTLN("Enc/Motor/MPU6050/Other Starts");

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_F, OUTPUT);
  RGB(ALL_OFF);  // RGB LED all off

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT_PULLUP);

  pinMode(ENC_IN_RIGHT_A, INPUT);
  pinMode(ENC_IN_RIGHT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT_PULLUP);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);

  ledcSetup(ENA_CH, 1000, 8);  //ENA, channel: 0, 1000Hz, 8bits = 256(0 ~ 255)
  ledcSetup(ENB_CH, 1000, 8);  //enB, channel: 1, 1000Hz, 8bits = 256(0 ~ 255)
  ledcSetup(BUZZER, 1000, 8);  //enB, channel: 1, 1000Hz, 8bits = 256(0 ~ 255)

  ledcAttachPin(ENA, ENA_CH);
  ledcAttachPin(ENB, ENB_CH);
  ledcAttachPin(BUZZER, BUZZER_CH);

  // Set the motor speed
  ledcWrite(ENA_CH, 0);
  ledcWrite(ENB_CH, 0);
  ledcWrite(BUZZER, 0);

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  trackPIDLeft.SetMode(AUTOMATIC);
  trackPIDLeft.SetSampleTime(200);
  trackPIDLeft.SetOutputLimits(-20, 20);
  trackPIDRight.SetMode(AUTOMATIC);
  trackPIDRight.SetSampleTime(200);
  trackPIDRight.SetOutputLimits(-20, 20);

#if USE_IMU == 1
  mpu_setup();
#endif

  beginLcd();

  // ROS Setup
  DEBUG_PRINTLN("ROS Starts");
  set_microros_transports();

  delay(2000);

  //wait agent comes up
  do {
    EXECUTE_EVERY_N_MS(300, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(300);

    if ((i++ % 32) == 0) DEBUG_PRINT("\n");
    else DEBUG_PRINT(".");
    if (state == AGENT_AVAILABLE)
      break;
  } while (1);

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rcl_init_options_set_domain_id(&init_options, DOMAINID);
  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  DEBUG_PRINTLN("rclc_support_init done");

  RCCHECK(rclc_node_init_default(&node, "uros_arduino_node", "", &support));
  DEBUG_PRINTLN("rclc_node_init done");

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &right_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_ticks"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &left_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_ticks"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &quat_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "quaternion"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &ledSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ledSub"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &songSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "songSub"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &lcdSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lcdSub"));

  // create cmd_vel subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &ledSub, &ledMsg, &subled_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &songSub, &songMsg, &subsong_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lcdSub, &lcdMsg, &sublcd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel, cmd_vel_callback, ON_NEW_DATA));

  DEBUG_PRINTLN("ROS established");
  DEBUG_PRINTLN("Done setup");
}

void loop() {
  switch (state) {
    case AGENT_AVAILABLE:
      //if setup done, always here. then go to next step
      state = AGENT_CONNECTED;
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      //if ping doesn't work, then reset board(=wait agent comes up)
      ESP.restart();
      break;
    default:
      break;
  }

  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    // Calculate the velocity of the right and left wheels
    calc_vel();

    RCSOFTCHECK(rcl_publish(&left_pub, &msg_left_tick_count, NULL));
    RCSOFTCHECK(rcl_publish(&right_pub, &msg_right_tick_count, NULL));
    RCSOFTCHECK(rcl_publish(&quat_pub, &quat_msg, NULL));

    mpu_loop();

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Stop the car if there are no cmd_vel messages
  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}

#if USE_IMU == 1
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
  mpu.setXGyroOffset(103);
  mpu.setYGyroOffset(20);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(595);  // 1688 factory default for my test chip

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
#if 0
    DEBUG_PRINT("quat\t");
    DEBUG_PRINT(q.w);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(q.x);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(q.y);
    DEBUG_PRINT("\t");
    DEBUG_PRINTLN(q.z);
#endif
    quat_msg.w = q.w;
    quat_msg.x = q.x;
    quat_msg.y = q.y;
    quat_msg.z = q.z;
  }
}
#endif
