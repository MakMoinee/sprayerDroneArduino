/*
  ESP32-S3 MAVLink Receiver + Access Point
  - ESP32-S3 creates its own Wi-Fi AP
  - Provides endpoints: /arm, /disarm, /takeoff, /land, /control
  - Talks MAVLink to APM 2.8 via Telemetry port (Serial)
*/

#include <WiFi.h>
#include <WebServer.h>
#include "MAVLink.h"

// === Wi-Fi Settings ===
const char* ssid = "ESP32S3-Drone";
const char* password = "12345678";

// === MAVLink & Drone Settings ===
#define SERIAL_PORT Serial1
#define MAVLINK_SYSID 255   // GCS system ID
#define MAVLINK_COMPID 190   // GCS component ID
#define TARGET_SYSID 1
#define TARGET_COMPID 1
volatile int roll;
volatile int pitch;
volatile int throttle;
volatile int yaw;

WebServer server(80);

// === Control Values ===
int16_t ch1_roll = 1500;
int16_t ch2_pitch = 1500;
int16_t ch3_throttle = 1000;
int16_t ch4_yaw = 1500;

// === Timers ===
unsigned long lastHeartbeat = 0;
unsigned long lastOverride = 0;

// === MAVLink Functions ===

// Send Heartbeat (required so APM accepts RC override)
void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    MAVLINK_SYSID, MAVLINK_COMPID, &msg,
    MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
    0, 0, MAV_STATE_ACTIVE
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
  SERIAL_PORT.flush();
}

void sendRCOverride(uint16_t ch1_roll, uint16_t ch2_pitch, uint16_t ch3_throttle, uint16_t ch4_yaw) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // The latest MAVLink expects 18 channel values.
  // We'll fill only the first 8, others set to 0.
  mavlink_msg_rc_channels_override_pack(
    MAVLINK_SYSID, MAVLINK_COMPID, &msg,
    TARGET_SYSID,
    TARGET_COMPID,
    ch1_roll, ch2_pitch, ch3_throttle, ch4_yaw,
    0, 0, 0, 0,   // CH5–CH8
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0   // CH9–CH18 (ignored)
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);

  Serial.printf("RC Override Sent -> Roll:%d Pitch:%d Throttle:%d Yaw:%d\n",
                ch1_roll, ch2_pitch, ch3_throttle, ch4_yaw);
}


// Send command (e.g., Arm, Disarm, Takeoff)
void sendCommand(uint16_t cmd, float p1 = 0, float p2 = 0, float p3 = 0,
                 float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    MAVLINK_SYSID, MAVLINK_COMPID, &msg,
    TARGET_SYSID, TARGET_COMPID, cmd, 0,
    p1, p2, p3, p4, p5, p6, p7
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
  SERIAL_PORT.flush();
}

// === HTTP Handlers ===
void armDrone() {
  sendCommand(MAV_CMD_COMPONENT_ARM_DISARM, 1);
  server.send(200, "text/plain", "Drone Armed");
}

void disarmDrone() {
  sendCommand(MAV_CMD_COMPONENT_ARM_DISARM, 0);
  server.send(200, "text/plain", "Drone Disarmed");
}

void takeoffDrone() {
  sendCommand(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 3); // 3m altitude
  Serial.println("Takeoff Command Sent");
  server.send(200, "text/plain", "Takeoff Command Sent");
}

void landDrone() {
  sendCommand(MAV_CMD_NAV_LAND);
  Serial.println("Land Command Sent");
  server.send(200, "text/plain", "Land Command Sent");
}

void controlDrone() {
  // Parse query params
  int p = server.hasArg("pitch") ? server.arg("pitch").toInt() : 1500;
  int r = server.hasArg("roll") ? server.arg("roll").toInt() : 1500;
  int t = server.hasArg("throttle") ? server.arg("throttle").toInt() : 1000;
  int y = server.hasArg("yaw") ? server.arg("yaw").toInt() : 1500;

  // Limit range 1000–2000 (PWM microseconds)
  p = constrain(p, 1000, 2000);
  r = constrain(r, 1000, 2000);
  t = constrain(t, 1000, 2000);
  y = constrain(y, 1000, 2000);

  ch1_roll = r;
  ch2_pitch = p;
  ch3_throttle = t;
  ch4_yaw = y;

  roll = r;
  pitch = p;
  throttle = t;
  yaw = y;

  String response = "Control set -> Roll=" + String(roll) +
                    ", Pitch=" + String(pitch) +
                    ", Throttle=" + String(throttle) +
                    ", Yaw=" + String(yaw);
  Serial.println(response);
  server.send(200, "text/plain", response);
}

// === Setup Web Routes ===
void setupRoutes() {
  server.on("/arm", armDrone);
  server.on("/disarm", disarmDrone);
  server.on("/takeoff", takeoffDrone);
  server.on("/land", landDrone);
  server.on("/control", controlDrone);
  server.on("/", []() {
    server.send(200, "text/plain", "ESP32-S3 Drone Controller Active");
  });
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting...");

  SERIAL_PORT.begin(57600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  // Start Wi-Fi in AP Mode
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(true);

  Serial.println("WiFi AP Started");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  setupRoutes();
  server.begin();
  Serial.println("Web Server Running...");
}

// === Loop ===
void loop() {
  server.handleClient();

  unsigned long now = millis();

  // Heartbeat every 1 second
  if (now - lastHeartbeat >= 1000) {
    sendHeartbeat();
    lastHeartbeat = now;
  }

  // RC override every 200 ms (5 Hz)
  if (now - lastOverride >= 200) {
    sendRCOverride(roll , pitch, throttle, yaw);
    lastOverride = now;
  }
}
