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

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

#define PWM_PARAM 0
#define PUB_VEL 0
#define VleftVRight 1
#define USE_LED 1

#define JGA25370 0
#define JGB520 1
#define MOTORTYPE JGB520

#define USE_PID 1
#if USE_PID == 1
double trackAdjustValueL = 0.0;
double trackSetpointL = 0.0;
double trackErrorL = 0.0;
double trackAdjustValueR = 0.0;
double trackSetpointR = 0.0;
double trackErrorR = 0.0;

double Kp = 20.0;  //Determines how aggressively the PID reacts to the current amount of error (Proportional)
double Ki = 20.0;  //Determines how aggressively the PID reacts to error over time (Integral)
double Kd = 1.0;   //Determines how aggressively the PID reacts to the change in error (Derivative)

PID trackPIDLeft(&trackErrorL, &trackAdjustValueL, &trackSetpointL, Kp, Ki, Kd, DIRECT);
PID trackPIDRight(&trackErrorR, &trackAdjustValueR, &trackSetpointR, Kp, Ki, Kd, DIRECT);
#endif

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5
#define ENC_IN_RIGHT_B 4

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// RGB Color lamp control pin
#define RLED A0
#define GLED A1
#define BLED A2

/***********************Enumeration variable***********************/
enum COLOR {
  RED = 0,  // red
  GREEN,    // green
  BLUE,     // blue
  YELLOW,   // yellow
  PURPLE,   // purple
  CYAN,     // cyan
  WHITE,    // white
  ALL_OFF   // off(black)
};

////////////////// Tick Data Publishing Variables and Constants ///////////////
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

#if PUB_VEL == 1
std_msgs::Float32 left_vel_pub;
ros::Publisher leftvelPub("velLeft", &left_vel_pub);
std_msgs::Float32 right_vel_pub;
ros::Publisher rightvelPub("velRight", &right_vel_pub);
#endif

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////
// Motor A connections, Left
const int enA = 10;
const int in1 = 12;
const int in2 = 13;
// Motor B connections, Right
const int enB = 9;
const int in3 = 7;
const int in4 = 6;

// TB6612 Chip control pins
#define TB6612_STBY 8

// Number of ticks per wheel revolution. We won't use this in this code.

// Wheel radius in meters. We won't use this in this code.
#define WHEEL_RADIUS (0.033)
#define WHEEL_DIAMETER (WHEEL_RADIUS * 2)
// Distance from center of the left tire to the center of the right tire in m. We won't use this in this code.
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
#if MOTORTYPE == JGB520
#define TICKS_PER_REVOLUTION (1860.0)
#else
#define TICKS_PER_REVOLUTION (102.0)
#endif

#define TICKS_PER_METER (TICKS_PER_REVOLUTION / (2.0 * 3.141592 * WHEEL_RADIUS))
#define WHEEL_BASE (0.160)

#if PWM_PARAM == 1
// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.
int K_P = 75;
// Y-intercept for the PWM-Linear Velocity relationship for the robot
int K_b = 42;
// Turning PWM output (0 = min, 255 = max for PWM values)
// Set maximum and minimum limits for the PWM values
int PWM_MIN = 50;  // about x.xxx m/s
int PWM_MAX = 80;  // about x.xxx m/s
#else

#if MOTORTYPE == JGB520
#define K_P 1125.0
#define K_b 3.5
#define PWM_MIN 40.0   // about 0.05 m/s
#define PWM_MAX 240.0  // about 0.2 m/s
#define K_bias 5.0     // left is slow, then add this bias
#else
#define K_P 125.0
#define K_b 35.0
#define PWM_MIN 40.0
#define PWM_MAX 80.0
#define K_bias 5.0
#endif

#endif

#define PWM_TURN (PWM_MIN)
// How much the PWM value can change each cycle
#define PWM_INCREMENT 1
// Correction multiplier for drift. Chosen through experimentation.
#define DRIFT_MULTIPLIER 80.0

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

/////////////////////// Tick Data Publishing Functions ////////////////////////

// Increment the number of ticks
void right_wheel_tick() {

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

// Increment the number of ticks
void left_wheel_tick() {

  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == HIGH) {
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

#if VleftVRight == 0
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {

  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis() / 1000.0);

  if (cmdVel.linear.x >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmLeftReq = K_P * cmdVel.linear.x + K_b + K_bias;
    pwmRightReq = K_P * cmdVel.linear.x + K_b;
  } else {
    pwmLeftReq = K_P * cmdVel.linear.x - K_b - K_bias;
    pwmRightReq = K_P * cmdVel.linear.x - K_b;
  }

  // Turn left
  if (cmdVel.angular.z > 0.1) {
    pwmLeftReq = -PWM_TURN;
    pwmRightReq = PWM_TURN;
  }
  // Turn right
  else if (cmdVel.angular.z < -0.1) {
    pwmLeftReq = PWM_TURN;
    pwmRightReq = -PWM_TURN;
  }

  // Go straight
  else {

    // Remove any differences in wheel velocities
    // to make sure the robot goes straight
    static float prevDiff = 0.0;
    static float prevPrevDiff = 0.0;
    float currDifference = velLeftWheel - velRightWheel;
    float avgDifference = (prevDiff + prevPrevDiff + currDifference) / 3.0;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;

    // Correct PWM values of both wheels to make the vehicle go straight
    pwmLeftReq -= (avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (avgDifference * DRIFT_MULTIPLIER);
  }

  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }
}
#else  //VleftVRight
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
#endif  //VleftVRight

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
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwmLeftReq < 0) {  // Left wheel reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (pwmLeftReq == 0 && pwmLeftOut == 0) {  // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else {  // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (pwmRightReq > 0) {  // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (pwmRightReq < 0) {  // Right wheel reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else if (pwmRightReq == 0 && pwmRightOut == 0) {  // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  } else {  // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
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

  // Set the PWM value on the pins
  analogWrite(enA, pwmLeftOut);
  analogWrite(enB, pwmRightOut);
}

#if USE_LED == 1
void RGB(enum COLOR color) {
  switch (color) {
    case RED:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, HIGH);
      break;
    case GREEN:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, HIGH);
      break;
    case BLUE:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, LOW);
      break;
    case YELLOW:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, HIGH);
      break;
    case PURPLE:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, LOW);
      break;
    case CYAN:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, LOW);
      break;
    case WHITE:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, LOW);
      break;
    default:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, HIGH);
      break;
  }
}

void ledcb(const std_msgs::Int16& msg) {
  RGB(msg.data);  // set the pin state to the message data
}
#endif

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);
#if USE_LED == 1
ros::Subscriber<std_msgs::Int16> subLed("rgbled", &ledcb);
#endif

void setup() {
#if PWM_PARAM == 1
  int pwm_constants[4];
  char buf[3];  //for debugging
#endif

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  pinMode(TB6612_STBY, OUTPUT);  // TB6612 Enable control pin configuration output
  digitalWrite(TB6612_STBY, HIGH);

  // Set the motor speed
  analogWrite(enA, 0);
  analogWrite(enB, 0);

#if USE_LED == 1
  pinMode(RLED, OUTPUT);  // RGB color lights red control pin configuration output
  pinMode(GLED, OUTPUT);  // RGB color light green control pin configuration output
  pinMode(BLED, OUTPUT);  // RGB color light blue control pin configuration output
  RGB(100);               // RGB LED all off
#endif

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
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

#if PWM_PARAM == 1
  nh.logwarn("S");
  if (!nh.getParam("/pwmConstants", pwm_constants, 4)) {
    nh.logwarn("F");

  } else {
    K_P = pwm_constants[0];
    K_b = pwm_constants[1];
    PWM_MIN = pwm_constants[2];
    PWM_MAX = pwm_constants[3];

    nh.logwarn("K");
    sprintf(buf, "%d", pwm_constants[0]);
    nh.logwarn(buf);
    sprintf(buf, "%d", pwm_constants[1]);
    nh.logwarn(buf);
    sprintf(buf, "%d", pwm_constants[2]);
    nh.logwarn(buf);
    sprintf(buf, "%d", pwm_constants[3]);
    nh.logwarn(buf);
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
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

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
  }

  // Stop the car if there are no cmd_vel messages
  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}
