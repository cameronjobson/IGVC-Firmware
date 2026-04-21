#include <Arduino.h>
#include "sdkconfig.h"
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h> 

#define ONBOARD_LED_PIN 2

// UART Hardware definitions for micro-ROS (PC Connection)
#define UART_PORT_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_TXD 1
#define UART_RXD 3

// UART Hardware definitions for Sabertooth
#define UART_TXD1 17  
#define UART_RXD1 18  

// --- RECEIVER PINS ---
const int throttlePin = 4; // Right Joystick (Ch 2)
const int steeringPin = 5; // Left Joystick (Ch 4)

// Sabertooth settings
const uint8_t sabertoothAddress = 128;
const int SABER_MAX_SPEED = 127;
const uint8_t TOP_MOTOR_SPEED = 60; 

// Global Variables
int mode = 0; // 0 = ROS2 Control, 1 = RC Control
int32_t motor_command_buffer[2] = {0, 0};

// micro-ROS objects
rcl_subscription_t command_subscriber;
rcl_subscription_t mode_subscriber; 
std_msgs__msg__Int32MultiArray cmd_msg;
std_msgs__msg__Int32 mode_msg;      
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Function Prototypes
void sendCommand(uint8_t command, uint8_t value);
void motor1_forward(uint8_t speed);
void motor1_reverse(uint8_t speed);
void motor2_forward(uint8_t speed);
void motor2_reverse(uint8_t speed);
void stop_motors();
void drive_tank(int left_speed, int right_speed);

// --- CUSTOM TRANSPORT FUNCTIONS ---
bool my_open(struct uxrCustomTransport * transport) { return true; }
bool my_close(struct uxrCustomTransport * transport) { return true; }

size_t my_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err) {
    int written = uart_write_bytes(UART_PORT_NUM, buf, len);
    return (written < 0) ? 0 : (size_t)written;
}

size_t my_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err) {
    int read = uart_read_bytes(UART_PORT_NUM, buf, len, timeout / portTICK_PERIOD_MS);
    return (read < 0) ? 0 : (size_t)read;
}
// ---------------------------------------

void error_loop(){
  while(1){
    digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
    delay(100);
  }
}

// --- MICRO-ROS CALLBACKS ---
void command_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  int left_speed = 0;
  int right_speed = 0;

  if (msg->data.size >= 2) {
    left_speed = msg->data.data[0];
    right_speed = msg->data.data[1];
  }

  // Visual indicator that a command was received
  digitalWrite(ONBOARD_LED_PIN, (left_speed != 0 || right_speed != 0) ? HIGH : LOW);
  
  if (mode == 0) {
      drive_tank(left_speed, right_speed);
  }
}

void mode_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  mode = msg->data; 
  stop_motors(); // Safety stop when switching modes
}

void setup() {
  // --- SABERTOOTH BOOT SEQUENCE ---
  // Slow down to 9600 for maximum reliability
  Serial2.begin(9600, SERIAL_8N1, UART_RXD1, UART_TXD1);
  delay(2000); // Wait 2 seconds for Sabertooth to fully wake up         
  Serial2.write(0xAA); // Autobaud sync 1
  delay(50);
  Serial2.write(0xAA); // Autobaud sync 2
  delay(50);
  stop_motors();       

  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);

  digitalWrite(ONBOARD_LED_PIN, HIGH); 
  delay(500);
  digitalWrite(ONBOARD_LED_PIN, LOW);

  // --- INITIALIZE UART HARDWARE ---
  uart_config_t uart_config = {
      .baud_rate = UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0, 
      .source_clk = UART_SCLK_APB,
  };
  
  uart_driver_install(UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0);
  uart_param_config(UART_PORT_NUM, &uart_config);
  uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // --- SET MICRO-ROS TRANSPORT ---
  rmw_uros_set_custom_transport(true, NULL, my_open, my_close, my_write, my_read);

  // Setup the MultiArray payload array
  cmd_msg.layout.dim.data = NULL;
  cmd_msg.layout.dim.size = 0;
  cmd_msg.layout.dim.capacity = 0;
  cmd_msg.layout.data_offset = 0;
  cmd_msg.data.data = motor_command_buffer;
  cmd_msg.data.size = 2;
  cmd_msg.data.capacity = 2;

  // --- START MICRO-ROS ---
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) error_loop();
  if (rclc_node_init_default(&node, "esp32_serial_node", "", &support) != RCL_RET_OK) error_loop();
  
  // Subscriber 1: Motor Commands
  if (rclc_subscription_init_default(
    &command_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, cmd_msg, Int32MultiArray),
    "/robot_command") != RCL_RET_OK) error_loop();

  // Subscriber 2: Mode Switches
  if (rclc_subscription_init_default(
    &mode_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, mode_msg, Int32),
    "/robot_mode") != RCL_RET_OK) error_loop();

  // Executor with 2 handles for the 2 subscribers
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &command_subscriber, &cmd_msg, &command_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &mode_subscriber, &mode_msg, &mode_callback, ON_NEW_DATA);
}

// --- RC MOTOR LOGIC ---
int normalizeJoystick(int rawValue, int minVal, int midLow, int midHigh, int maxVal) {
  if (rawValue == 0) return 0; // Failsafe
  if (rawValue >= midLow && rawValue <= midHigh) return 0; 
  if (rawValue < midLow) {
    rawValue = max(rawValue, minVal); 
    return map(rawValue, minVal, midLow - 1, -255, -1);
  }
  if (rawValue > midHigh) {
    rawValue = min(rawValue, maxVal);
    return map(rawValue, midHigh + 1, maxVal, 1, 255);
  }
  return 0; 
}

void moveMotorsHelper(int leftSpeedForward, int leftSpeedBackward, int rightSpeedForward, int rightSpeedBackward) {
    if (leftSpeedForward > 0) motor1_forward(leftSpeedForward);
    else if (leftSpeedBackward > 0) motor1_reverse(leftSpeedBackward);
    else motor1_forward(0);

    if (rightSpeedForward > 0) motor2_forward(rightSpeedForward);
    else if (rightSpeedBackward > 0) motor2_reverse(rightSpeedBackward);
    else motor2_forward(0);
}

void moveMotors() {
    int rY = pulseIn(throttlePin, HIGH, 25000); 
    int lX = pulseIn(steeringPin, HIGH, 25000); 

    int throttle = normalizeJoystick(rY, 980, 1480, 1487, 1967);
    int steering = normalizeJoystick(lX, 987, 1480, 1487, 1969);

    int adj_throttle = (abs(throttle) / 255.0) * TOP_MOTOR_SPEED; 
    int adj_steering = (abs(steering) / 255.0) * TOP_MOTOR_SPEED;

    int motorAdjustment = max(0, adj_throttle - adj_steering); 
    bool forward = (throttle >= 30);
    bool backward = (throttle <= -30);
    bool left = (steering >= 30);
    bool right = (steering <= -30);

    if (forward) {
        if (left) moveMotorsHelper(motorAdjustment,0, adj_throttle, 0);
        else if (right) moveMotorsHelper(adj_throttle, 0, motorAdjustment, 0);
        else moveMotorsHelper(adj_throttle, 0, adj_throttle, 0);
    } else if (backward) {
        if (left) moveMotorsHelper(0, motorAdjustment, 0, adj_throttle);
        else if (right) moveMotorsHelper(0, adj_throttle, 0, motorAdjustment);
        else moveMotorsHelper(0, adj_throttle, 0, adj_throttle);
    } else { 
        moveMotorsHelper(0, 0, 0, 0);
    }
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  if (mode == 1) { 
      moveMotors(); 
  }
}

// --- SABERTOOTH SERIAL FUNCTIONS ---
void motor1_forward(uint8_t speed) { sendCommand(0, speed); }
void motor1_reverse(uint8_t speed) { sendCommand(1, speed); }
void motor2_forward(uint8_t speed) { sendCommand(4, speed); }
void motor2_reverse(uint8_t speed) { sendCommand(5, speed); }
void stop_motors() { motor1_forward(0); motor2_forward(0); }

void sendCommand(uint8_t command, uint8_t value) {
  uint8_t checksum = (sabertoothAddress + command + value) & 0x7F;
  Serial2.write(sabertoothAddress);
  Serial2.write(command);
  Serial2.write(value);
  Serial2.write(checksum);
}

void drive_tank(int left_speed, int right_speed) {
  left_speed = constrain(left_speed, -SABER_MAX_SPEED, SABER_MAX_SPEED);
  right_speed = constrain(right_speed, -SABER_MAX_SPEED, SABER_MAX_SPEED);

  if (left_speed > 0) motor1_forward(left_speed);
  else if (left_speed < 0) motor1_reverse(-left_speed);
  else motor1_forward(0);

  if (right_speed > 0) motor2_forward(right_speed);
  else if (right_speed < 0) motor2_reverse(-right_speed);
  else motor2_forward(0);
}

extern "C" void app_main() {
    initArduino();
    setup();
    while (true) {
        loop();
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}