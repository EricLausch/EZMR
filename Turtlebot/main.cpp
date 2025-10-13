// Standard Arduino-Funktionen
#include <Arduino.h>

// Micro-ROS PlatformIO-Anbindung
#include <micro_ros_platformio.h>

// ROS 2 Kernfunktionen
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <HardwareSerial.h>

#include <geometry_msgs/msg/twist.h>

// Sicherstellen, dass Arduino Serial Transport verwendet wird
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  #error This example is only available for Arduino framework with serial transport.
#endif

// Definition der ROS-Objekte
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

geometry_msgs__msg__Twist msg;

#define LED 2
const int MySerialRX = 16; // RX pin
const int MySerialTX = 17; // TX pin
HardwareSerial MySerial(2);

// dir = 0: Gegen Uhrzeigersinn
// mode = 1: Dauerbetrieb

uint8_t MotorL[] = {0xAA, 0x01, 0x1, 0x1, 0x0, 0x2};
uint8_t MotorR[] = {0xAA, 0x02, 0x1, 0x0, 0x0, 0x2};   // Startbyte / ID Motor Rechts / Modus / Drehrichtung / Parameter

// Makros zur Fehlerbehandlung
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    error_loop(); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) {} \
}

// Fehlerbehandlungsfunktion
void error_loop() {
  while (1) {
    delay(100);
  }
}
void cmd_vel_callback(const void *msgin);

// Timer-Callback-Funktion
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

  if (timer == NULL){
    return;
  }
}

void setup() {
  
  MySerial.begin(9600, SERIAL_8N1, MySerialRX, MySerialTX); // Initialize MySerial with a baud rate of 115200, 8 data bits, no parity, 1 stop bit
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);

  allocator = rcl_get_default_allocator();

  // Initialisierung des Micro-ROS Supports
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialisierung des ROS 2 Knotens
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Initialisierung des Publishers
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/turtle1/cmd_vel"
  ));

  const unsigned int timer_timeout = 100;

  // Initialisierung des Timers
  RCCHECK(rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback,
    true   // autostart: Timer startet automatisch
  ));

  // Initialisierung des Executors
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));

  // Hinzufügen des Timers zum Executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

}

void cmd_vel_callback(const void *msgin){

  auto *t = (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(LED, HIGH);
  MySerial.write(MotorL, sizeof(MotorL));
  MySerial.write(MotorR, sizeof(MotorR));
  delay(1000);
  digitalWrite(LED, LOW);
}

void loop() {
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  delay(1);
}

