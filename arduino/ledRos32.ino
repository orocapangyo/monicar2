#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_subscription_t ledSub;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define DEBUG 1

#define RXD2 16
#define TXD2 17

#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// LED control pins
#define LED_L 19
#define LED_R 18
#define LED_B 5

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
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
bool blinkState = false;

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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}


void subled_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  RGB(LEDBEHAV(msg->data));
}

void setup() {

#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif
  DEBUG_PRINTLN("LEDRos Starts");

  pinMode(LED_L, OUTPUT);  // RGB color lights red control pin configuration output
  pinMode(LED_R, OUTPUT);  // RGB color light green control pin configuration output
  pinMode(LED_B, OUTPUT);  // RGB color light blue control pin configuration output
  RGB(ALL_OFF);            // RGB LED all off

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // ROS Setup
  DEBUG_PRINTLN("ROS Starts");

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 108;
  RCCHECK(rclc_node_init_with_options(&node, "uros_arduino_node", "", &support, &node_ops));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &ledSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ledSub"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &ledSub, &msg, &subled_callback, ON_NEW_DATA));

  DEBUG_PRINTLN("ROS created");
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
