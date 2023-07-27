/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Calculate the accumulated ticks for each wheel using the 
 * built-in encoder (forward = positive; reverse = negative) 
 * tick_counter.ino
 */

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5
#define ENC_IN_RIGHT_B 4

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

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;

// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

int pwmReq = 0;

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

void set_pwm_values() {
  // Set the direction of the motors
  if (pwmReq > 0) {  // Left wheel forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwmReq < 0) {  // Left wheel reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {  // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (pwmReq > 0) {  // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (pwmReq < 0) {  // Right wheel reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {  // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  // Set the PWM value on the pins
  analogWrite(enA, pwmReq);
  analogWrite(enB, pwmReq);
}

void setup() {
  Serial.begin(115200);

  Serial.println("Motor econder test");
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

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
}

void loop() {
  char rc = 0;
  // Record the time
  currentMillis = millis();

  //input a, s, d
  if (Serial.available() > 0) {
    rc = Serial.read();

    if (rc == 'a')
      pwmReq += 10;
    else if (rc == 'd')
      pwmReq -= 10;
    else if (rc == 's')
      pwmReq = 0;

    Serial.print(rc);
    Serial.print(":");
    Serial.println(pwmReq);
  }

  set_pwm_values();
  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    Serial.print("L:");
    Serial.print(left_wheel_tick_count);
    Serial.print(",R:");
    Serial.println(right_wheel_tick_count);
  }
}
