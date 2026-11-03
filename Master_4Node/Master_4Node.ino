#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =====================================================
// SETTINGS
// =====================================================

#define CMD_SCAN 1
#define ESPNOW_CHANNEL 1
#define MAX_NETWORKS 50

#define RELAY_PIN 32

// Active-HIGH relay
#define RELAY_ON LOW
#define RELAY_OFF HIGH

const unsigned long RESULT_TIMEOUT_MS = 8000;
const unsigned long COOLDOWN_MS = 1000;

// =====================================================
// FOUR ANTENNA NODE MAC ADDRESSES
// =====================================================

uint8_t node1MAC[] = {
  0x20, 0x9B, 0xA9, 0x68, 0xC4, 0xB4
};

uint8_t node2MAC[] = {
  0x68, 0x09, 0x47, 0x9D, 0x90, 0x38
};

uint8_t node3MAC[] = {
  0x68, 0x09, 0x47, 0x9C, 0xDA, 0x20
};

uint8_t node4MAC[] = {
  0x68, 0x09, 0x47, 0x9E, 0x0E, 0x70
};

// =====================================================
// ESP2 PAN CONTROLLER MAC
// =====================================================

uint8_t panESP_MAC[] = {
  0x08, 0xB6, 0x1F, 0xB9, 0x7D, 0x84
};

// =====================================================
// PACKET STRUCTURES
// =====================================================

typedef struct {
  uint8_t command;
} CommandPacket;

typedef struct {
  uint8_t nodeID;
  char ssid[33];
  uint8_t bssid[6];
  int32_t rssi;
  bool endOfScan;
} WiFiPacket;

typedef struct {
  float angle;
} PanAnglePacket;

CommandPacket command = { CMD_SCAN };
PanAnglePacket panPacket;

// =====================================================
// NETWORK DATABASE
// =====================================================

struct NetworkData {
  bool used;
  char ssid[33];
  uint8_t bssid[6];
  int rssi[4];
};

NetworkData networks[MAX_NETWORKS];

bool finished[4];

// =====================================================
// MASTER STATE MACHINE
// =====================================================

enum MasterState {
  STATE_SEND_SCAN,
  STATE_WAIT_RESULTS,
  STATE_COOLDOWN
};

MasterState state = STATE_SEND_SCAN;
unsigned long stateTimer = 0;

// =====================================================
// SERIAL COMMAND BUFFER
// =====================================================

String serialInput = "";

// =====================================================
// CLEAR DATABASE
// =====================================================

void clearDatabase() {
  for (int i = 0; i < MAX_NETWORKS; i++) {
    networks[i].used = false;
    networks[i].ssid[0] = '\0';

    for (int j = 0; j < 4; j++) {
      networks[i].rssi[j] = -1000;
    }
  }

  for (int i = 0; i < 4; i++) {
    finished[i] = false;
  }
}

// =====================================================
// COMPARE BSSID
// =====================================================

bool sameBSSID(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }

  return true;
}

// =====================================================
// FIND NETWORK
// =====================================================

int findNetwork(const uint8_t *bssid) {
  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (networks[i].used &&
        sameBSSID(networks[i].bssid, bssid)) {
      return i;
    }
  }

  return -1;
}

// =====================================================
// CREATE NETWORK
// =====================================================

int createNetwork() {
  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (!networks[i].used) {
      networks[i].used = true;

      for (int j = 0; j < 4; j++) {
        networks[i].rssi[j] = -1000;
      }

      return i;
    }
  }

  return -1;
}

// =====================================================
// ADD ESP-NOW PEER
// =====================================================

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peer = {};

  memcpy(peer.peer_addr, mac, 6);

  peer.channel = 0;
  peer.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peer);

  if (result != ESP_OK) {
    Serial.println("PEER_ADD_FAILED");
  }
}

// =====================================================
// PRINT RSSI
// =====================================================

void printRSSI(int value) {
  if (value == -1000) {
    Serial.print("NA");
  } else {
    Serial.print(value);
  }
}

// =====================================================
// PRINT ANTENNA RESULTS
// =====================================================

void printResults() {
  int count = 0;

  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (networks[i].used) {
      count++;
    }
  }

  if (count == 0) {
    Serial.println("NO_WIFI_DATA_RECEIVED");
    return;
  }

  Serial.println(
    "SSID,RSSI_ANT1,RSSI_ANT2,RSSI_ANT3,RSSI_ANT4,BEST"
  );

  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (!networks[i].used) {
      continue;
    }

    int best = -1;

    for (int j = 0; j < 4; j++) {
      if (networks[i].rssi[j] == -1000) {
        continue;
      }

      if (best == -1 ||
          networks[i].rssi[j] > networks[i].rssi[best]) {
        best = j;
      }
    }

    Serial.print(networks[i].ssid);
    Serial.print(",");

    for (int j = 0; j < 4; j++) {
      printRSSI(networks[i].rssi[j]);
      Serial.print(",");
    }

    if (best >= 0) {
      Serial.print("ANT");
      Serial.println(best + 1);
    } else {
      Serial.println("NA");
    }
  }
}

// =====================================================
// ESP-NOW RECEIVE CALLBACK
// =====================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  (void)info;

  if (len != sizeof(WiFiPacket)) {
    return;
  }

  WiFiPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.nodeID < 1 || packet.nodeID > 4) {
    return;
  }

  if (packet.endOfScan) {
    finished[packet.nodeID - 1] = true;
    return;
  }

  if (packet.rssi < -100 || packet.rssi > 0) {
    return;
  }

  int idx = findNetwork(packet.bssid);

  if (idx == -1) {
    idx = createNetwork();

    if (idx == -1) {
      return;
    }

    strncpy(networks[idx].ssid, packet.ssid, 32);
    networks[idx].ssid[32] = '\0';

    memcpy(networks[idx].bssid, packet.bssid, 6);
  }

  int previousRSSI = networks[idx].rssi[packet.nodeID - 1];

  if (previousRSSI == -1000 ||
      packet.rssi > previousRSSI) {
    networks[idx].rssi[packet.nodeID - 1] = packet.rssi;
  }
}

// =====================================================
// SEND SCAN COMMAND
// =====================================================

void sendScan() {
  const uint8_t *nodeMACs[4] = {
    node1MAC,
    node2MAC,
    node3MAC,
    node4MAC
  };

  for (int i = 0; i < 4; i++) {
    esp_err_t result = esp_now_send(
      nodeMACs[i],
      (const uint8_t *)&command,
      sizeof(command)
    );

    if (result != ESP_OK) {
      Serial.print("SCAN_SEND_FAILED_NODE_");
      Serial.println(i + 1);
    }

    delay(20);
  }
}

// =====================================================
// SEND PAN ANGLE TO ESP2
// =====================================================

void sendPanAngle(float angle) {
  panPacket.angle = angle;

  esp_err_t result = esp_now_send(
    panESP_MAC,
    (const uint8_t *)&panPacket,
    sizeof(panPacket)
  );

  if (result == ESP_OK) {
    Serial.print("PAN_ANGLE_SENT:");
    Serial.println(angle);
  } else {
    Serial.println("PAN_SEND_FAILED");
  }
}

// =====================================================
// LASER CONTROL
// ONLY ON AND OFF CONTROL THE LASER
// =====================================================

void laserOn() {
  digitalWrite(RELAY_PIN, RELAY_ON);
  Serial.println("LASER_ON");
}

void laserOff() {
  digitalWrite(RELAY_PIN, RELAY_OFF);
  Serial.println("LASER_OFF");
}

// =====================================================
// PROCESS SERIAL COMMAND
//
// ON       = Laser ON
// OFF      = Laser OFF
// 0-359    = Pan angle to ESP2
// =====================================================

void processCommand(String input) {
  input.trim();

  // Only the text ON command activates the laser
  if (input.equalsIgnoreCase("ON")) {
    laserOn();
    return;
  }

  // Only the text OFF command deactivates the laser
  if (input.equalsIgnoreCase("OFF")) {
    laserOff();
    return;
  }

  // All numeric inputs are pan angles
  if (input.length() == 0) {
    return;
  }

  bool valid = true;
  bool decimalSeen = false;

  for (unsigned int i = 0; i < input.length(); i++) {
    char ch = input[i];

    if (ch == '.' && !decimalSeen) {
      decimalSeen = true;
    } else if (ch < '0' || ch > '9') {
      valid = false;
      break;
    }
  }

  if (!valid) {
    Serial.println(
      "INVALID_COMMAND: Use ON, OFF, or angle 0-359"
    );
    return;
  }

  float angle = input.toFloat();

  if (angle >= 0.0f && angle <= 359.0f) {
    sendPanAngle(angle);
  } else {
    Serial.println(
      "INVALID_ANGLE: Enter an angle from 0 to 359"
    );
  }
}

// =====================================================
// READ SERIAL COMMANDS
// =====================================================

void checkSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialInput.length() > 0) {
        processCommand(serialInput);
        serialInput = "";
      }
    } else if (c >= 32 && c <= 126) {
      if (serialInput.length() < 12) {
        serialInput += c;
      } else {
        serialInput = "";
        Serial.println("COMMAND_TOO_LONG");
      }
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  // Initialize laser relay in OFF state
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  WiFi.mode(WIFI_STA);

  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP_NOW_INIT_FAILED");

    while (true) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  addPeer(node1MAC);
  addPeer(node2MAC);
  addPeer(node3MAC);
  addPeer(node4MAC);
  addPeer(panESP_MAC);

  clearDatabase();

  Serial.println("ESP1_MASTER_READY");
  Serial.println("Laser: ON / OFF");
  Serial.println("Pan angle: 0-359");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  checkSerial();

  switch (state) {

    case STATE_SEND_SCAN:
      clearDatabase();

      sendScan();

      stateTimer = millis();
      state = STATE_WAIT_RESULTS;
      break;

    case STATE_WAIT_RESULTS: {
      bool allFinished =
        finished[0] &&
        finished[1] &&
        finished[2] &&
        finished[3];

      bool timedOut =
        millis() - stateTimer >= RESULT_TIMEOUT_MS;

      if (allFinished || timedOut) {
        printResults();

        stateTimer = millis();
        state = STATE_COOLDOWN;
      }

      break;
    }

    case STATE_COOLDOWN:
      if (millis() - stateTimer >= COOLDOWN_MS) {
        state = STATE_SEND_SCAN;
      }
      break;
  }
}