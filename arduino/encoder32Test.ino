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

// One-second interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

int pwmReq = 0;

#define PWM_MIN 40

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

  ledcSetup(ENA_CH, 500, 8);  //ENA, channel: 0, 500Hz, 8bits = 256(0 ~ 255)
  ledcSetup(ENB_CH, 500, 8);  //enB, channel: 1, 500Hz, 8bits = 256(0 ~ 255)

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
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    Serial.print("L:");
    Serial.print(left_wheel_tick_count);
    Serial.print(",R:");
    Serial.println(right_wheel_tick_count);
  }
}
