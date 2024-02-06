/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Calculate the accumulated ticks for each wheel using the
 * built-in encoder (forward = positive; reverse = negative)
 * tick_counter.ino
 */
// Please select motor type
#define MOTOR_60RPM 1
#define MOTOR_178RPM 2
#define MOTOR_TYPE MOTOR_60RPM

#define PRINT_VEL 0
#define PRINT_AVGVEL 0
#define PRINT_ENC 1

// Encoder output to Arduino Interrupt pin. Tracks the tick count
#if MOTOR_TYPE == MOTOR_60RPM
#define ENC_IN_LEFT_A 36
#define ENC_IN_RIGHT_A 34
#define ENC_IN_LEFT_B 39
#define ENC_IN_RIGHT_B 35

#define PWM_MIN 40
#else
#define ENC_IN_LEFT_A 39
#define ENC_IN_RIGHT_A 35
#define ENC_IN_LEFT_B 36
#define ENC_IN_RIGHT_B 34

#define PWM_MIN 30
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
#define TICKS_PER_REVOLUTION (1860.0)
#else
#define TICKS_PER_REVOLUTION (620.0)
//left:620, right:580
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

int pwmReq = 0;

#define NUM_READ 40
int avgL[NUM_READ], avgR[NUM_READ];
int readIndex = 0;
int totalL = 0, totalR = 0;

bool blinkState = false;

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

void set_pwm_values() {
  // Set the direction of the motors
  if (pwmReq > 0) {  // Left wheel forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmReq < 0) {  // Left wheel reverse
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {  // Left wheel stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  if (pwmReq > 0) {  // Right wheel forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (pwmReq < 0) {  // Right wheel reverse
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {  // Right wheel stop
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  if (pwmReq > 0) {
    // Set the PWM value on the pins
    ledcWrite(ENA_CH, pwmReq);
    ledcWrite(ENB_CH, pwmReq);
  } else {
    // Set the PWM value on the pins
    ledcWrite(ENA_CH, -pwmReq);
    ledcWrite(ENB_CH, -pwmReq);
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("Motor econder test");

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

  ledcSetup(ENA_CH, 1000, 8);  //ENA, channel: 0, 100Hz, 8bits = 256(0 ~ 255)
  ledcSetup(ENB_CH, 1000, 8);  //enB, channel: 1, 100Hz, 8bits = 256(0 ~ 255)

  ledcAttachPin(ENA, ENA_CH);
  ledcAttachPin(ENB, ENB_CH);

  // Set the motor speed
  ledcWrite(ENA_CH, 0);
  ledcWrite(ENB_CH, 0);
}

void loop() {
  char rc = 0;
  // Record the time
  currentMillis = millis();

  //input a, s, d
  if (Serial.available() > 0) {
    rc = Serial.read();

    if (rc == 'a') {
      if (pwmReq == 0)
        pwmReq = PWM_MIN;
      else
        pwmReq += 10;
    } else if (rc == 'd') {
      if (pwmReq == 0)
        pwmReq = -PWM_MIN;
      else
        pwmReq -= 10;
    } else if (rc == 's')
      pwmReq = 0;

    Serial.print(rc);
    Serial.print(":");
    Serial.println(pwmReq);
  }

  set_pwm_values();

  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > INTERVAL) {

    previousMillis = currentMillis;
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    // Calculate the velocity of the right and left wheels
    calc_vel();

#if PRINT_ENC == 1
    Serial.print("L:");
    Serial.print(left_wheel_tick_count);
    Serial.print(",R:");
    Serial.println(right_wheel_tick_count);
#endif

  }
}
