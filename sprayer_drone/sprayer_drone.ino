/*
  ESP32-S3 MAVLink Receiver + Access Point
  - ESP32-S3 creates its own Wi-Fi AP
  - Provides endpoints: /arm, /disarm, /takeoff, /land, /control
  - Talks MAVLink to APM 2.8 via Telemetry port (Serial)
*/

#include <WiFi.h>
#include <WebServer.h>
// MAVLink library - install from Library Manager: "MAVLink"
#include <mavlink.h>

// === Wi-Fi Settings ===
const char* ssid = "ESP32S3-Drone";
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

// === MAVLink Functions ===

// Request RC Override mode from APM 2.8
void requestRCOverride() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Send command to enable RC override
  mavlink_msg_command_long_pack(
    MAVLINK_SYSID, MAVLINK_COMPID, &msg,
    TARGET_SYSID, TARGET_COMPID, 
    MAV_CMD_DO_SET_MODE, 0,
    MAV_MODE_MANUAL_ARMED, 0, 0, 0, 0, 0, 0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
  SERIAL_PORT.flush();
  
  Serial.println("RC Override Mode Request Sent");
}

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

  // APM 2.8 standard channel mapping: Roll=CH1, Pitch=CH2, Throttle=CH3, Yaw=CH4
  // Set unused channels to UINT16_MAX to indicate they should be ignored
  mavlink_msg_rc_channels_override_pack(
    MAVLINK_SYSID, MAVLINK_COMPID, &msg,
    TARGET_SYSID, TARGET_COMPID,
    ch1_roll,           // Channel 1: Roll (Aileron) 
    ch2_pitch,          // Channel 2: Pitch (Elevator)
    ch3_throttle,       // Channel 3: Throttle
    ch4_yaw,            // Channel 4: Yaw (Rudder)
    UINT16_MAX,         // Channel 5: Aux1 (ignored)
    UINT16_MAX,         // Channel 6: Aux2 (ignored)  
    UINT16_MAX,         // Channel 7: Aux3 (ignored)
    UINT16_MAX,         // Channel 8: Aux4 (ignored)
    UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, 
    UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX // CH9-CH18 (ignored)
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
  SERIAL_PORT.flush(); // Ensure data is sent immediately

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
  int p = server.hasArg("pitch") ? server.arg("pitch").toInt() : pitch;     // Keep current if not specified
  int r = server.hasArg("roll") ? server.arg("roll").toInt() : roll;        // Keep current if not specified
  int t = server.hasArg("throttle") ? server.arg("throttle").toInt() : throttle; // Keep current if not specified
  int y = server.hasArg("yaw") ? server.arg("yaw").toInt() : yaw;          // Keep current if not specified

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
  response += "MAVLink Status: Active";
  
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
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(true);

  Serial.println("WiFi AP Started");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  setupRoutes();
  server.begin();
  Serial.println("Web Server Running...");
  
  // Wait for telemetry connection to stabilize
  delay(2000);
  
  // Send initial heartbeat and request RC override mode
  sendHeartbeat();
  delay(500);
  requestRCOverride();
  
  Serial.println("MAVLink RC Override Mode Initialized");
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
