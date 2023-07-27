/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Calculate the accumulated ticks for each wheel using the 
 * built-in encoder (forward = positive; reverse = negative) 
 * tick_counter.ino
 */

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
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else if (pwmReq < 0) {  // Left wheel reverse
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  } else {  // Left wheel stop
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
  }

  if (pwmReq > 0) {  // Right wheel forward
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else if (pwmReq < 0) {  // Right wheel reverse
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  } else {  // Right wheel stop
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, LOW);
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
