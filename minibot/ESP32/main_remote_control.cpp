// ====== Includes ======
#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

// ====== WLAN + Agent ======
#define WIFI_SSID      "ROS_Net"
#define WIFI_PASSWORD  "Aceu36NCmp!!nfuennvBVje"
#define AGENT_IP_STR   "10.42.0.1"
#define AGENT_PORT     8888

// ====== ROS Topic ======
#define CMD_VEL_TOPIC  "/cmd_vel"   // bei Turtlesim ggf. "/turtle1/cmd_vel"

// ====== Pins & UART ======
#define LED_PIN        2
const int MY_SERIAL_RX = 16;
const int MY_SERIAL_TX = 17;
HardwareSerial MySerial(2);

// ====== Motor/Protokoll ======
// Frame: 0xAA, id, mode, dir, 0x00, param_low
// Richtungen (dein Mapping):
//  - Links:  vorwärts = 0x01, rückwärts = 0x00
//  - Rechts: vorwärts = 0x00, rückwärts = 0x01
#define ID_LEFT    0x01
#define ID_RIGHT   0x02
#define MODE_RUN   0x01   // Dauerlauf
#define MODE_STEP  0x00   // Schrittmodus

// ====== Speed-Skala ======
// param_low: 0x10 (nahe 0) … 0x02 (schnell)
#define PARAM_MIN_SLOW  0x10
#define PARAM_MAX_FAST  0x02

// Normierung/Empfindlichkeit
#define V_MAX_MPS   0.6f   // oberes Ende erwarteter |Radgeschwindigkeit|
#define EPS_STOP    0.01f  // „praktisch 0“ für Stop-Entscheidung

// Für schnelleres Drehen auf der Stelle (optional):
#define EPS_LIN     0.05f  // Schwelle „v≈0“
#define TURN_GAIN   3.0f   // Verstärkung von w bei Drehen-auf-der-Stelle
#define TURN_MIN_P  0x06   // optional: Obergrenze für p_lo beim Drehen (0x06..0x02)

// ====== Kinematik (einfach) ======
const float B_HALF  = 0.084f;  // halber Spurabstand (m)

// ====== ROS Objekte ======

// Speichert alle Informationen zu einem abonnierten Topic (z. B. /cmd_vel)
// Enthält den Nachrichtentyp, den Callback-Zeiger und den Status der Verbindung.
rcl_subscription_t subscriber;

// Verwalter für alle ROS-Aufgaben (z. B. Abonnements, Timer, Callbacks)
// Der Executor ruft automatisch die richtigen Callback-Funktionen auf,
// sobald neue Nachrichten empfangen werden oder Timer ablaufen.
rclc_executor_t executor;

// Grundlegende ROS-Unterstützungsstruktur – hält Kontextinformationen,
// die beim Starten des micro-ROS-Systems benötigt werden (Kommunikation, Speicher usw.)
// Wird mit rclc_support_init(...) initialisiert und an andere Funktionen weitergereicht.
rclc_support_t support;

// Zuständig für Speicherverwaltung in micro-ROS.
// Der Allocator definiert, wie Speicher angefordert, genutzt und freigegeben wird.
rcl_allocator_t allocator;

// Repräsentiert die eigentliche micro-ROS-Node auf dem ESP32.
// Die Node ist der "Teilnehmer" im ROS-Netzwerk – sie hat einen Namen
// und dient als Container für Publisher, Subscriber und Timer.
rcl_node_t node;

// ====== Utils ======
static inline void send_frame(uint8_t id, uint8_t mode, uint8_t dir, uint8_t param_low) {
  uint8_t frame[6] = { 0xAA, id, mode, dir, 0x00, param_low };
  MySerial.write(frame, 6);
  // Debug
  Serial.printf("TX id=%u mode=%u dir=%u p_lo=0x%02X -> [%02X %02X %02X %02X %02X %02X]\n",
                id, mode, dir, param_low,
                frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);
}

// Mappt |v| in [0..V_MAX_MPS] -> p_lo in [0x10..0x02]
static inline uint8_t speed_to_param_low(float v_abs) {
  if (v_abs <= 0.0f)
    return PARAM_MIN_SLOW;   // 0 m/s → 0x10 = Stillstand

  // Normierung: teile durch deine maximale erwartete Geschwindigkeit
  float x = v_abs / V_MAX_MPS;   // V_MAX_MPS = 0.6f
  if (x > 1.0f) x = 1.0f;        // Kappen auf 1, falls zu schnell

  // Umrechnung: 0..1 → 0x10..0x01. 15.0f ist max. diff zwischen 0x01 und 0x10. lround rundet auf float ohne Nachkommastellen. uint8_t wandelt in 8-Bit um.
  uint8_t p = (uint8_t)(0x10 - (uint8_t)lroundf(x * 15.0f));

  // Grenzen absichern
  if (p < PARAM_MAX_FAST) p = PARAM_MAX_FAST; // nie kleiner als 0x02
  if (p > PARAM_MIN_SLOW) p = PARAM_MIN_SLOW; // nie größer als 0x10
  return p;
}

// ====== Callback: sendet NUR bei Empfang ======
void cmd_vel_callback(const void *msgin) {
  if (!msgin) return;

  // Eingehende Nachricht als Twist interpretieren
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist*)msgin;

  digitalWrite(LED_PIN, HIGH);

  // 1) Eingänge aus der empfangenen Twist-Nachricht
  float v = twist_msg->linear.x;   // Vorwärts-/Rückwärtsgeschwindigkeit (m/s)
  float w = twist_msg->angular.z;  // Drehgeschwindigkeit (rad/s)

  // 2) Stillstand?
  if (fabsf(v) < EPS_STOP && fabsf(w) < EPS_STOP) {
    // Schrittmodus, Schrittweite 0 --> absoluter Stillstand
    send_frame(ID_RIGHT, MODE_STEP, 0x00, 0x00);
    send_frame(ID_LEFT,  MODE_STEP, 0x01, 0x00);
    Serial.println("STOP: mode=STEP, step=0");
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // 3) Drehen-auf-der-Stelle schneller machen (optional)
  bool in_place = (fabsf(v) < EPS_LIN) && (fabsf(w) >= EPS_STOP);
  if (in_place) {
    w *= TURN_GAIN;  // Winkel verstärken
  }

  // 4) Differential-Kinematik
  float v_r = v + w * B_HALF;
  float v_l = v - w * B_HALF;

  // 5) Richtung gemäß Mapping
  uint8_t dir_r = (v_r >= 0.0f) ? 0x00 : 0x01;  // rechts: vorwärts=0x00
  uint8_t dir_l = (v_l >= 0.0f) ? 0x01 : 0x00;  // links:  vorwärts=0x01

  // 6) Betrag -> p_lo in [0x10..0x02]
  uint8_t p_lo_r = speed_to_param_low(fabsf(v_r));
  uint8_t p_lo_l = speed_to_param_low(fabsf(v_l));

  // beim Drehen p_lo begrenzen (kleiner = schneller)
  if (in_place) {
    if (p_lo_r > TURN_MIN_P) p_lo_r = TURN_MIN_P;
    if (p_lo_l > TURN_MIN_P) p_lo_l = TURN_MIN_P;
  }

  // 7) Senden 
  send_frame(ID_RIGHT, MODE_RUN, dir_r, p_lo_r);
  send_frame(ID_LEFT,  MODE_RUN, dir_l, p_lo_l);

  digitalWrite(LED_PIN, LOW);
}

// ====== Fehler-Makros ======
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { while(1){ delay(100);} } } // Harte Prüfung: Node darf nur starten, wenn alles klappt
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }  // Weiche Prüfung: es ist kein Problem, wenn in dieser Schleife mal kein Callback ausgeführt wird

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(200);

  // Motor-UART
  MySerial.begin(9600, SERIAL_8N1, MY_SERIAL_RX, MY_SERIAL_TX);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // WLAN verbinden
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.printf("\nWiFi OK, ESP IP: %s\n", WiFi.localIP().toString().c_str());

  // micro-ROS über Wi-Fi
  IPAddress agent_ip; agent_ip.fromString(AGENT_IP_STR);
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASSWORD, agent_ip, AGENT_PORT);

  // ROS 2 Init
  allocator = rcl_get_default_allocator();  //legt fest, wie Speicher verwaltet wird
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));  //bootet die micro-ROS Runtime (Kontext, Middleware)
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));  //registriert Node im ROS-Graphen (Name + Namespace)

  // Subscriber initialisieren (RELIABLE)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    CMD_VEL_TOPIC
  ));

  // Executor (nur 1 Handle: Subscription)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Puffer, in den eingehende Nachrichten hineinkopiert werden. static, damit Speicher gesamte Laufzeit gültig bleibt
  static geometry_msgs__msg__Twist rx_msg;
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &rx_msg, &cmd_vel_callback, ON_NEW_DATA));

  Serial.println("Setup fertig (sendet nur bei Empfang).");
}

// ====== Loop ======
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  delay(1);
}
