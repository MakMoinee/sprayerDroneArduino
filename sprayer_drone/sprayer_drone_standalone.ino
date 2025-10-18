/*
  ESP32-S3 MAVLink Receiver + Access Point (Standalone Version)
  - ESP32-S3 creates its own Wi-Fi AP
  - Provides endpoints: /arm, /disarm, /takeoff, /land, /control
  - Talks MAVLink to APM 2.8 via Telemetry port (Serial)
  - NO EXTERNAL LIBRARIES REQUIRED - All MAVLink code included
*/

#include <WiFi.h>
#include <WebServer.h>

// === MAVLink Protocol Definitions (Standalone) ===
#define MAVLINK_STX 0xFE
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_CORE_HEADER_LEN 5
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1)
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)

// MAVLink Message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70
#define MAVLINK_MSG_ID_COMMAND_LONG 76

// MAVLink Commands
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#define MAV_CMD_NAV_TAKEOFF 22
#define MAV_CMD_NAV_LAND 21
#define MAV_CMD_DO_SET_MODE 176

// MAVLink Types
#define MAV_TYPE_GCS 6
#define MAV_AUTOPILOT_INVALID 8
#define MAV_STATE_ACTIVE 4
#define MAV_MODE_MANUAL_ARMED 64

// === Wi-Fi Settings ===
const char* ssid = "ESP32S3-Drone";
const char* ssid_backup = "DroneController";  // Backup SSID
const char* password = "12345678";

// === MAVLink & Drone Settings ===
#define SERIAL_PORT Serial1
#define MAVLINK_SYSID 255   // GCS system ID
#define MAVLINK_COMPID 190   // GCS component ID
#define TARGET_SYSID 1
#define TARGET_COMPID 1

// Initialize with safe default values
volatile int roll = 1500;      // Center position
volatile int pitch = 1500;     // Center position  
volatile int throttle = 1000;  // Minimum throttle (safe)
volatile int yaw = 1500;       // Center position

WebServer server(80);

// === Control Values ===
int16_t ch1_roll = 1500;
int16_t ch2_pitch = 1500;
int16_t ch3_throttle = 1000;
int16_t ch4_yaw = 1500;

// === Timers ===
unsigned long lastHeartbeat = 0;
unsigned long lastOverride = 0;
unsigned long lastRCRequest = 0;
unsigned long lastWiFiCheck = 0;

// === MAVLink Packet Structure ===
typedef struct {
  uint8_t magic;
  uint8_t len;
  uint8_t seq;
  uint8_t sysid;
  uint8_t compid;
  uint8_t msgid;
  uint8_t payload[MAVLINK_MAX_PAYLOAD_LEN];
  uint8_t ck_a;
  uint8_t ck_b;
} mavlink_message_t;

// === MAVLink CRC Calculation ===
void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c) {
  msg->ck_a += c;
  msg->ck_b += msg->ck_a;
}

void mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, uint8_t length) {
  msg->magic = MAVLINK_STX;
  msg->len = length;
  msg->sysid = system_id;
  msg->compid = component_id;
  
  msg->ck_a = 0;
  msg->ck_b = 0;
  
  mavlink_update_checksum(msg, msg->len);
  mavlink_update_checksum(msg, msg->seq);
  mavlink_update_checksum(msg, msg->sysid);
  mavlink_update_checksum(msg, msg->compid);
  mavlink_update_checksum(msg, msg->msgid);
  
  for(int i = 0; i < length; i++) {
    mavlink_update_checksum(msg, msg->payload[i]);
  }
  
  // Add CRC_EXTRA for each message type
  uint8_t crc_extra = 0;
  switch(msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: crc_extra = 50; break;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: crc_extra = 124; break;
    case MAVLINK_MSG_ID_COMMAND_LONG: crc_extra = 152; break;
  }
  mavlink_update_checksum(msg, crc_extra);
}

uint16_t mavlink_msg_to_send_buffer(uint8_t* buffer, mavlink_message_t* msg) {
  buffer[0] = msg->magic;
  buffer[1] = msg->len;
  buffer[2] = msg->seq;
  buffer[3] = msg->sysid;
  buffer[4] = msg->compid;
  buffer[5] = msg->msgid;
  
  for(int i = 0; i < msg->len; i++) {
    buffer[6 + i] = msg->payload[i];
  }
  
  buffer[6 + msg->len] = msg->ck_a;
  buffer[7 + msg->len] = msg->ck_b;
  
  return 8 + msg->len;
}

// === MAVLink Message Builders ===
void mavlink_msg_heartbeat_pack(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id) {
  msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
  msg->seq = 0;
  
  msg->payload[0] = MAV_TYPE_GCS;
  msg->payload[1] = MAV_AUTOPILOT_INVALID;
  msg->payload[2] = 0; // base_mode
  msg->payload[3] = 0; // custom_mode (4 bytes)
  msg->payload[4] = 0;
  msg->payload[5] = 0;
  msg->payload[6] = 0;
  msg->payload[7] = MAV_STATE_ACTIVE; // system_status
  msg->payload[8] = 3; // mavlink_version
  
  mavlink_finalize_message(msg, system_id, component_id, 9);
}

void mavlink_msg_rc_channels_override_pack(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                          uint8_t target_system, uint8_t target_component,
                                          uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw,
                                          uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw,
                                          uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw,
                                          uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw,
                                          uint16_t chan17_raw, uint16_t chan18_raw) {
  msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
  msg->seq = 0;
  
  // Pack target system/component
  msg->payload[0] = target_system;
  msg->payload[1] = target_component;
  
  // Pack channels (little-endian)
  msg->payload[2] = chan1_raw & 0xFF; msg->payload[3] = (chan1_raw >> 8) & 0xFF;
  msg->payload[4] = chan2_raw & 0xFF; msg->payload[5] = (chan2_raw >> 8) & 0xFF;
  msg->payload[6] = chan3_raw & 0xFF; msg->payload[7] = (chan3_raw >> 8) & 0xFF;
  msg->payload[8] = chan4_raw & 0xFF; msg->payload[9] = (chan4_raw >> 8) & 0xFF;
  msg->payload[10] = chan5_raw & 0xFF; msg->payload[11] = (chan5_raw >> 8) & 0xFF;
  msg->payload[12] = chan6_raw & 0xFF; msg->payload[13] = (chan6_raw >> 8) & 0xFF;
  msg->payload[14] = chan7_raw & 0xFF; msg->payload[15] = (chan7_raw >> 8) & 0xFF;
  msg->payload[16] = chan8_raw & 0xFF; msg->payload[17] = (chan8_raw >> 8) & 0xFF;
  
  mavlink_finalize_message(msg, system_id, component_id, 18);
}

void mavlink_msg_command_long_pack(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                  uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation,
                                  float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
  msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
  msg->seq = 0;
  
  // Pack parameters (IEEE 754 little-endian)
  memcpy(&msg->payload[0], &param1, 4);
  memcpy(&msg->payload[4], &param2, 4);
  memcpy(&msg->payload[8], &param3, 4);
  memcpy(&msg->payload[12], &param4, 4);
  memcpy(&msg->payload[16], &param5, 4);
  memcpy(&msg->payload[20], &param6, 4);
  memcpy(&msg->payload[24], &param7, 4);
  
  // Pack command (little-endian)
  msg->payload[28] = command & 0xFF;
  msg->payload[29] = (command >> 8) & 0xFF;
  
  // Pack target system/component
  msg->payload[30] = target_system;
  msg->payload[31] = target_component;
  msg->payload[32] = confirmation;
  
  mavlink_finalize_message(msg, system_id, component_id, 33);
}

// === WiFi Functions ===
void checkWiFiStatus() {
  if (WiFi.getMode() != WIFI_AP) {
    Serial.println("⚠ WiFi mode changed! Restarting AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password, 1, 0, 4);
  }
  
  Serial.print("WiFi Status - Connected Clients: ");
  Serial.print(WiFi.softAPgetStationNum());
  Serial.print(" | AP IP: ");
  Serial.println(WiFi.softAPIP());
}

// === MAVLink Functions ===

// Request RC Override mode from APM 2.8
void requestRCOverride() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(&msg, MAVLINK_SYSID, MAVLINK_COMPID, TARGET_SYSID, TARGET_COMPID, 
                               MAV_CMD_DO_SET_MODE, 0, MAV_MODE_MANUAL_ARMED, 0, 0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
  SERIAL_PORT.flush();
  
  Serial.println("RC Override Mode Request Sent");
}

// Send Heartbeat (required so APM accepts RC override)
void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(&msg, MAVLINK_SYSID, MAVLINK_COMPID);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
  SERIAL_PORT.flush();
}

void sendRCOverride(uint16_t ch1_roll, uint16_t ch2_pitch, uint16_t ch3_throttle, uint16_t ch4_yaw) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_rc_channels_override_pack(&msg, MAVLINK_SYSID, MAVLINK_COMPID, TARGET_SYSID, TARGET_COMPID,
                                       ch1_roll, ch2_pitch, ch3_throttle, ch4_yaw,
                                       65535, 65535, 65535, 65535, // CH5-8 ignored
                                       65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535); // CH9-18 ignored

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
  SERIAL_PORT.flush();

  Serial.printf("RC Override Sent -> Roll:%d Pitch:%d Throttle:%d Yaw:%d\n",
                ch1_roll, ch2_pitch, ch3_throttle, ch4_yaw);
}

// Send command (e.g., Arm, Disarm, Takeoff)
void sendCommand(uint16_t cmd, float p1 = 0, float p2 = 0, float p3 = 0,
                 float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(&msg, MAVLINK_SYSID, MAVLINK_COMPID, TARGET_SYSID, TARGET_COMPID, 
                               cmd, 0, p1, p2, p3, p4, p5, p6, p7);

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
  int p = server.hasArg("pitch") ? server.arg("pitch").toInt() : pitch;
  int r = server.hasArg("roll") ? server.arg("roll").toInt() : roll;
  int t = server.hasArg("throttle") ? server.arg("throttle").toInt() : throttle;
  int y = server.hasArg("yaw") ? server.arg("yaw").toInt() : yaw;

  // Limit range 1000–2000 (PWM microseconds)
  p = constrain(p, 1000, 2000);
  r = constrain(r, 1000, 2000);
  t = constrain(t, 1000, 2000);
  y = constrain(y, 1000, 2000);

  // Update all control variables
  ch1_roll = r;
  ch2_pitch = p;
  ch3_throttle = t;
  ch4_yaw = y;

  roll = r;
  pitch = p;
  throttle = t;
  yaw = y;

  String response = "Control Updated -> Roll=" + String(roll) +
                    ", Pitch=" + String(pitch) +
                    ", Throttle=" + String(throttle) +
                    ", Yaw=" + String(yaw);
  Serial.println(response);
  server.send(200, "text/plain", response);
}

void statusDrone() {
  String response = "Current RC Values:\n";
  response += "Roll: " + String(roll) + " (Ch1)\n";
  response += "Pitch: " + String(pitch) + " (Ch2)\n";
  response += "Throttle: " + String(throttle) + " (Ch3)\n";
  response += "Yaw: " + String(yaw) + " (Ch4)\n";
  response += "ESP32 IP: " + WiFi.softAPIP().toString() + "\n";
  response += "MAVLink Status: Active (Standalone)";
  
  Serial.println("Status requested");
  server.send(200, "text/plain", response);
}

// === Setup Web Routes ===
void setupRoutes() {
  server.on("/arm", armDrone);
  server.on("/disarm", disarmDrone);
  server.on("/takeoff", takeoffDrone);
  server.on("/land", landDrone);
  server.on("/control", controlDrone);
  server.on("/status", statusDrone);
  
  // Main control interface
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><title>ESP32-S3 Drone Controller</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;margin:20px;} .btn{padding:10px 20px;margin:5px;background:#007cba;color:white;border:none;border-radius:5px;cursor:pointer;} .btn:hover{background:#005a87;} .control{margin:10px 0;} input{padding:5px;margin:5px;width:80px;}</style></head>";
    html += "<body><h1>ESP32-S3 Drone Controller</h1>";
    html += "<h2>Drone Commands</h2>";
    html += "<button class='btn' onclick=\"fetch('/arm')\">ARM</button>";
    html += "<button class='btn' onclick=\"fetch('/disarm')\">DISARM</button>";
    html += "<button class='btn' onclick=\"fetch('/takeoff')\">TAKEOFF</button>";
    html += "<button class='btn' onclick=\"fetch('/land')\">LAND</button><br>";
    html += "<h2>Manual Control</h2>";
    html += "<div class='control'>Roll: <input type='number' id='roll' min='1000' max='2000' value='1500'></div>";
    html += "<div class='control'>Pitch: <input type='number' id='pitch' min='1000' max='2000' value='1500'></div>";
    html += "<div class='control'>Throttle: <input type='number' id='throttle' min='1000' max='2000' value='1000'></div>";
    html += "<div class='control'>Yaw: <input type='number' id='yaw' min='1000' max='2000' value='1500'></div>";
    html += "<button class='btn' onclick='sendControl()'>UPDATE CONTROL</button>";
    html += "<button class='btn' onclick='centerControls()'>CENTER ALL</button><br>";
    html += "<button class='btn' onclick=\"window.open('/status', '_blank')\">VIEW STATUS</button>";
    html += "<script>";
    html += "function sendControl() {";
    html += "  var r = document.getElementById('roll').value;";
    html += "  var p = document.getElementById('pitch').value;";
    html += "  var t = document.getElementById('throttle').value;";
    html += "  var y = document.getElementById('yaw').value;";
    html += "  fetch('/control?roll=' + r + '&pitch=' + p + '&throttle=' + t + '&yaw=' + y);";
    html += "}";
    html += "function centerControls() {";
    html += "  document.getElementById('roll').value = 1500;";
    html += "  document.getElementById('pitch').value = 1500;";
    html += "  document.getElementById('throttle').value = 1000;";
    html += "  document.getElementById('yaw').value = 1500;";
    html += "  sendControl();";
    html += "}";
    html += "</script></body></html>";
    server.send(200, "text/html", html);
  });
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting...");

  SERIAL_PORT.begin(57600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  // Start Wi-Fi in AP Mode
  Serial.println("Starting WiFi Access Point...");
  
  // Configure WiFi for maximum compatibility and visibility
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Maximum power for better range
  WiFi.setSleep(false); // Keep WiFi active at all times
  
  // Start AP with explicit configuration
  bool apStarted = WiFi.softAP(ssid, password, 1, 0, 4); // Channel 1, not hidden, max 4 clients
  
  if (apStarted) {
    Serial.println("✓ WiFi AP Started Successfully!");
    Serial.print("SSID: "); Serial.println(ssid);
    Serial.print("Password: "); Serial.println(password);
    Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());
    Serial.print("MAC Address: "); Serial.println(WiFi.softAPmacAddress());
    Serial.print("Channel: 1");
    Serial.println("\n--- Connect your device to this network ---");
  } else {
    Serial.println("✗ Failed to start WiFi AP!");
    Serial.println("Trying backup SSID...");
    delay(1000);
    // Retry with backup SSID
    bool backupStarted = WiFi.softAP(ssid_backup, password, 6, 0, 4); // Try channel 6
    if (backupStarted) {
      Serial.println("✓ Backup WiFi AP Started!");
      Serial.print("SSID: "); Serial.println(ssid_backup);
      Serial.print("Password: "); Serial.println(password);
      Serial.print("IP: "); Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("✗ Both WiFi attempts failed!");
      Serial.println("Check ESP32 S3 WiFi capability...");
    }
  }

  setupRoutes();
  server.begin();
  Serial.println("✓ Web Server Running!");
  
  // Test WiFi AP immediately
  Serial.println("\n=== WiFi AP Test ===");
  delay(1000);
  checkWiFiStatus();
  Serial.println("Look for 'ESP32S3-Drone' or 'DroneController' in your WiFi list!");
  Serial.println("Connect with password: 12345678");
  Serial.println("Then open: http://192.168.4.1");
  Serial.println("======================\n");
  
  // Wait for telemetry connection to stabilize
  delay(2000);
  
  // Send initial heartbeat and request RC override mode
  sendHeartbeat();
  delay(500);
  requestRCOverride();
  
  Serial.println("MAVLink RC Override Mode Initialized (Standalone)");
}

// === Loop ===
void loop() {
  server.handleClient();

  unsigned long now = millis();

  // Check WiFi status every 30 seconds
  if (now - lastWiFiCheck >= 30000) {
    checkWiFiStatus();
    lastWiFiCheck = now;
  }

  // Heartbeat every 1 second
  if (now - lastHeartbeat >= 1000) {
    sendHeartbeat();
    lastHeartbeat = now;
  }

  // Request RC override mode every 10 seconds to ensure APM stays in override mode
  if (now - lastRCRequest >= 10000) {
    requestRCOverride();
    lastRCRequest = now;
  }

  // RC override every 100 ms (10 Hz) - APM 2.8 needs frequent updates
  if (now - lastOverride >= 100) {
    sendRCOverride(roll, pitch, throttle, yaw);
    lastOverride = now;
  }
}
