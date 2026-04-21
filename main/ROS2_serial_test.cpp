#include <Arduino.h>
#include "sdkconfig.h"
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
// FIX 3: This header contains the rmw_uros_set_custom_transport declaration
#include <rmw_microros/custom_transport.h> 

#define ONBOARD_LED_PIN 2

// UART Hardware definitions
#define UART_PORT_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_TXD 1
#define UART_RXD 3

// micro-ROS objects
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- YOUR CUSTOM TRANSPORT FUNCTIONS ---
// FIX 1: We ignore the transport pointer and use UART_PORT_NUM directly
bool my_open(struct uxrCustomTransport * transport) {
    return true;
}

bool my_close(struct uxrCustomTransport * transport) {
    return true;
}

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

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(ONBOARD_LED_PIN, (msg->data == 1) ? HIGH : LOW);
}

void setup() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, HIGH); 
  delay(500);
  digitalWrite(ONBOARD_LED_PIN, LOW);

  // --- 1. INITIALIZE UART HARDWARE ---
  uart_config_t uart_config = {
      .baud_rate = UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0, // FIX 2: Satisfies the missing initializer warning
      .source_clk = UART_SCLK_APB,
  };
  
  uart_driver_install(UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0);
  uart_param_config(UART_PORT_NUM, &uart_config);
  uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // --- 2. SET MICRO-ROS TRANSPORT ---
  // We no longer need to pass the port via args, so we can pass NULL
  rmw_uros_set_custom_transport(true, NULL, my_open, my_close, my_write, my_read);

  // --- 3. START MICRO-ROS ---
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) error_loop();
  if (rclc_node_init_default(&node, "esp32_serial_node", "", &support) != RCL_RET_OK) error_loop();
  
  if (rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/robot_command") != RCL_RET_OK) error_loop();

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10); 
}

extern "C" void app_main() {
    initArduino();
    setup();
    while (true) {
        loop();
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}