// ====== Includes ======
#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>  //Empfangen der Geschwindigkeitsbefehle
#include <std_msgs/msg/u_int8.h>  //Senden der Bumper-Zustände

// ====== WLAN + Agent ======
#define WIFI_SSID      "ROS_Net"
#define WIFI_PASSWORD  "Aceu36NCmp!!nfuennvBVje"
#define AGENT_IP_STR   "10.42.0.1"
#define AGENT_PORT     8888

// ====== ROS Topics ======
#define CMD_VEL_TOPIC   "/cmd_vel"        // Host -> Robot
#define BUMPER_TOPIC    "/bumper/state"   // Robot -> Host

// ====== Pins & UART ======
#define LED_PIN        2
const int MY_SERIAL_RX = 16;
const int MY_SERIAL_TX = 17;
HardwareSerial MySerial(2);

// ====== Bumper ======
#define PIN_BUMPER_LEFT   12
#define PIN_BUMPER_RIGHT  14
#define BUMPER_PRESSED(level)  ((level) == LOW)  // Input-Pullups -> LOW = gedrückt

// ====== Motor/Protokoll ======
#define ID_LEFT    0x01
#define ID_RIGHT   0x02
#define MODE_RUN   0x01
#define MODE_STEP  0x00

#define PARAM_MIN_SLOW  0x10
#define PARAM_MAX_FAST  0x02

#define V_MAX_MPS   0.6f
#define EPS_STOP    0.01f
#define EPS_LIN     0.05f
#define TURN_GAIN   3.0f
#define TURN_MIN_P  0x06
const float B_HALF  = 0.084f;

// ====== ROS Objekte ======
rcl_subscription_t subscriber;
rcl_publisher_t    pub_bumper;        //Publisher für /bumper/state
rclc_executor_t    executor;
rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;
rcl_timer_t        timer;             // Polling-Timer für Bumper
std_msgs__msg__UInt8 bumper_msg;      // Msg-Puffer für Publisher

// ====== Utils Motor ======
static inline void send_frame(uint8_t id, uint8_t mode, uint8_t dir, uint8_t param_low) {
  uint8_t frame[6] = { 0xAA, id, mode, dir, 0x00, param_low };
  MySerial.write(frame, 6);
  Serial.printf("TX id=%u mode=%u dir=%u p=0x%02X -> [%02X %02X %02X %02X %02X %02X]\n",
                id, mode, dir, param_low, frame[0],frame[1],frame[2],frame[3],frame[4],frame[5]);
}

static inline void stop_hard() {
  send_frame(ID_RIGHT, MODE_STEP, 0x00, 0x00);
  send_frame(ID_LEFT,  MODE_STEP, 0x01, 0x00);
}

static inline uint8_t speed_to_param_low(float v_abs) {
  if (v_abs <= 0.0f) return PARAM_MIN_SLOW;
  float x = v_abs / V_MAX_MPS; if (x > 1.0f) x = 1.0f;
  uint8_t p = (uint8_t)(0x10 - (uint8_t)lroundf(x * 15.0f)); // 0x10..0x01
  if (p < PARAM_MAX_FAST) p = PARAM_MAX_FAST;
  if (p > PARAM_MIN_SLOW) p = PARAM_MIN_SLOW;
  return p;
}

// ====== cmd_vel Callback ======
void cmd_vel_callback(const void *msgin) {
  if (!msgin) return;
  const auto *t = (const geometry_msgs__msg__Twist*)msgin;

  digitalWrite(LED_PIN, HIGH);

  float v = t->linear.x;
  float w = t->angular.z;

  if (fabsf(v) < EPS_STOP && fabsf(w) < EPS_STOP) {
    stop_hard();
    Serial.println("STOP (cmd_vel)");
    digitalWrite(LED_PIN, LOW);
    return;
  }

  bool in_place = (fabsf(v) < EPS_LIN) && (fabsf(w) >= EPS_STOP);
  if (in_place) w *= TURN_GAIN;

  float v_r = v + w * B_HALF;
  float v_l = v - w * B_HALF;

  uint8_t dir_r = (v_r >= 0.0f) ? 0x00 : 0x01;
  uint8_t dir_l = (v_l >= 0.0f) ? 0x01 : 0x00;

  uint8_t p_lo_r = speed_to_param_low(fabsf(v_r));
  uint8_t p_lo_l = speed_to_param_low(fabsf(v_l));

  if (in_place) {
    if (p_lo_r > TURN_MIN_P) p_lo_r = TURN_MIN_P;
    if (p_lo_l > TURN_MIN_P) p_lo_l = TURN_MIN_P;
  }

  Serial.printf("RUN | R: v=%.3f dir=%u p=0x%02X | L: v=%.3f dir=%u p=0x%02X%s\n",
                v_r, dir_r, p_lo_r, v_l, dir_l, p_lo_l, in_place ? " [spin]" : "");

  send_frame(ID_RIGHT, MODE_RUN, dir_r, p_lo_r);
  send_frame(ID_LEFT,  MODE_RUN, dir_l, p_lo_l);

  digitalWrite(LED_PIN, LOW);
}

// ====== Bumper Polling (Timer) ======
uint8_t read_bumper_mask() {
  uint8_t l = digitalRead(PIN_BUMPER_LEFT);
  uint8_t r = digitalRead(PIN_BUMPER_RIGHT);
  uint8_t m = 0;
  if (BUMPER_PRESSED(l)) m |= 0x01;  // Bit0: Left
  if (BUMPER_PRESSED(r)) m |= 0x02;  // Bit1: Right
  return m;
}

// Timer-Callback publiziert nur bei Änderung
void timer_cb(rcl_timer_t*, int64_t) {
  static uint8_t last = 0xFF; // erzwinge erstes Publish
  uint8_t m = read_bumper_mask();
  if (m != last) {
    bumper_msg.data = m;
    rcl_publish(&pub_bumper, &bumper_msg, NULL);
    Serial.printf("BUMPER state: L=%d R=%d (mask=0x%02X)\n", (m&1)?1:0, (m&2)?1:0, m);
    last = m;
  }
}

// ====== Fehler-Makros ======
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { Serial.printf("ERR rc=%d @%s:%d\n", rc, __FILE__, __LINE__); while(1){ delay(100);} } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(200);

  // Motor-UART
  MySerial.begin(9600, SERIAL_8N1, MY_SERIAL_RX, MY_SERIAL_TX);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Bumper als active-LOW mit Pullups
  pinMode(PIN_BUMPER_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BUMPER_RIGHT, INPUT_PULLUP);

  // WLAN
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.printf("\nWiFi OK, ESP IP: %s\n", WiFi.localIP().toString().c_str());

  // micro-ROS Transport
  IPAddress agent_ip; agent_ip.fromString(AGENT_IP_STR);
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASSWORD, agent_ip, AGENT_PORT);

  // ROS Init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Publisher: /bumper/state
  RCCHECK(rclc_publisher_init_default(
    &pub_bumper, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    BUMPER_TOPIC
  ));

  // Subscriber: /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    CMD_VEL_TOPIC
  ));

  // Timer: 20 Hz Bumper-Polling
  RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(50), timer_cb, true));

  // Executor (Sub + Timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  static geometry_msgs__msg__Twist rx_msg;
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &rx_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Safety: initialer Stopp
  stop_hard();

  Serial.println("Setup fertig: /cmd_vel aktiv, /bumper/state Publisher aktiv.");
}

// ====== Loop ======
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  delay(1);
}
