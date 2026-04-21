#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define ONBOARD_LED_PIN 2

// Wi-Fi credentials
const char* ssid = "<WIFI_NAME>";
const char* password = "<WIFI_PASSWORD>";

// micro-ROS variables
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Error loop to blink rapidly if micro-ROS fails to initialize
void error_loop(){
  while(1){
    digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Callback function: This runs every time a message is received
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  if (msg->data == 1) {
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    Serial.println("Received command: 1 (LED ON)");
  } else if (msg->data == 0) {
    digitalWrite(ONBOARD_LED_PIN, LOW);
    Serial.println("Received command: 0 (LED OFF)");
  }
}

// micro-ROS logic runs isolated in its own task
void micro_ros_task(void * arg) {
  allocator = rcl_get_default_allocator();

  Serial.println("Connecting to micro-ROS Agent...");
  
  // Initialize Support (This connects to the Agent IP you put in menuconfig)
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
      Serial.println("Failed to connect to agent.");
      error_loop();
  }

  if (rclc_node_init_default(&node, "esp32_wifi_node", "", &support) != RCL_RET_OK) error_loop();

  if (rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/robot_command") != RCL_RET_OK) error_loop();

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  Serial.println("micro-ROS node initialized and listening on /robot_command");

  // Spin forever
  while(1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main() {
    initArduino();

    // Re-enable Serial Monitor so we can read the IP address and debug info!
    Serial.begin(115200);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    
    // Startup blink
    digitalWrite(ONBOARD_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(ONBOARD_LED_PIN, LOW);

    // 1. Connect to Wi-Fi BEFORE starting micro-ROS
    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());

    // 2. Launch the micro-ROS task
    xTaskCreate(micro_ros_task, "uros_task", 1024 * 8, NULL, 5, NULL);
}