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
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

// Please select motor type
#define MOTOR_60RPM 1
#define MOTOR_178RPM 2
#define MOTOR_TYPE MOTOR_60RPM

#define PRINT_VEL 0
#define PRINT_PIDERR 0
#define PRINT_AVGPWM 0
#define PRINT_INSPWM 1

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

// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;

// Time interval for measurements in milliseconds
const int INTERVAL = 100;
long previousMillis = 0;
long currentMillis = 0;

#if MOTOR_TYPE == MOTOR_60RPM
#define K_P 1153.0
#define K_b 10
#define PWM_MIN 44.0   // about 0.03 m/s
#define PWM_MAX 194.0  // about 0.16 m/s
#define TICKS_PER_REVOLUTION (1860.0)
#define K_Lbias 0.0
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
bool blinkState = false;

#define NUM_READ 40
float linearx = 0.0, angularz = 0.0;
int avgPWML[NUM_READ], avgPWMR[NUM_READ];
int readIndex = 0;
int totalL = 0, totalR = 0;

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
      left_wheel_tick_count = encoder_maximum;
    } else {
      left_wheel_tick_count--;
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
  //Serial.print("PWM:");
  //Serial.print(pwmReq);
  Serial.print("L:");
  Serial.print(velLeftWheel,3);
  Serial.print(",R:");
  Serial.println(velRightWheel,3);
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
  Serial.print("aL:");
  Serial.print(average);

  // subtract the last reading:
  totalR = totalR - avgR[readIndex];
  // read from the sensor:
  avgR[readIndex] = velRightWheel*1000;
  // add the reading to the total:
  totalR = totalR + avgR[readIndex];

  // calculate the average:
  average = totalR / NUM_READ;
  Serial.print(",aR:");
  Serial.println(average);

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
void calc_pwm_values(float linearx, float angularz) {
  //set target velocity
  vLeft = linearx - angularz * WHEEL_BASE / 2.0;
  vRight = linearx + angularz * WHEEL_BASE / 2.0;

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

  Serial.print("calPWM:");
  Serial.print(pwmLeftReq);
  Serial.print(":");
  Serial.println(pwmRightReq);
}

void set_pwm_values() {
  int average, pwm_inc;
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
  Serial.print("rL:");
  Serial.print(pwmLeftReq);
  Serial.print(",rR:");
  Serial.print(pwmRightReq);
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
  Serial.print(",oL:");
  Serial.print(pwmLeftOut);
  Serial.print(",oR:");
  Serial.println(pwmRightOut);
#endif

#if PRINT_AVGPWM == 1
  // subtract the last reading:
  totalL = totalL - avgPWML[readIndex];
  // read from the sensor:
  avgPWML[readIndex] = pwmLeftOut;
  // add the reading to the total:
  totalL = totalL + avgPWML[readIndex];

  // calculate the average:
  average = totalL / NUM_READ;
  Serial.print("aL:");
  Serial.print(average);

  // subtract the last reading:
  totalR = totalR - avgPWMR[readIndex];
  // read from the sensor:
  avgPWMR[readIndex] = pwmRightOut;
  // add the reading to the total:
  totalR = totalR + avgPWMR[readIndex];

  // calculate the average:
  average = totalR / NUM_READ;
  Serial.print(",aR:");
  Serial.println(average);
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= NUM_READ) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
#endif

  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

#if PRINT_PIDERR == 1
  Serial.print("sErr:");
  Serial.print(trackErrorL, 3);
  Serial.print(":");
  Serial.println(trackErrorR, 3);
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

void setup() {

  Serial.begin(115200);
  Serial.println("Motor vel_cmd test");
  pinMode(LED_BUILTIN, OUTPUT);

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

  ledcAttachPin(ENA, ENA_CH);
  ledcAttachPin(ENB, ENB_CH);

  // Set the motor speed
  ledcWrite(ENA_CH, 0);
  ledcWrite(ENB_CH, 0);

  trackPIDLeft.SetMode(AUTOMATIC);
  trackPIDLeft.SetSampleTime(200);
  trackPIDLeft.SetOutputLimits(-20, 20);
  trackPIDRight.SetMode(AUTOMATIC);
  trackPIDRight.SetSampleTime(200);
  trackPIDRight.SetOutputLimits(-20, 20);
}

void loop() {

  // Record the time
  currentMillis = millis();

  //input like "0.15 -0.5"
  if (Serial.available() > 0) {
    linearx = Serial.parseFloat();
    angularz = Serial.parseFloat();
    readIndex = 0;
    totalL = 0;
    totalR = 0;
    for (int i = 0; i < NUM_READ; i++) {
      avgPWML[i] = 0;
      avgPWMR[i] = 0;
    }

    //prints the received float number
    Serial.print("Lv:");
    Serial.print(linearx, 3);
    Serial.print(":Av,");
    Serial.println(angularz, 3);
    calc_pwm_values(linearx, angularz);
  }

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    // Calculate the velocity of the right and left wheels
    calc_vel();

    set_pwm_values();
  }
}
