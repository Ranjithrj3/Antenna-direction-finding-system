
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define CMD_SCAN 1
#define NODE_ID 4
#define ESPNOW_CHANNEL 1

// Must be ESP1's actual Wi-Fi STA MAC address.
uint8_t masterMAC[] = {
  0x24, 0xDC, 0xC3, 0x44, 0x36, 0x6C
};

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

volatile bool scanRequested = false;
bool espNowReady = false;

void sendNetwork(
  const char* ssid,
  const uint8_t* bssid,
  int rssi
) {
  WiFiPacket packet = {};

  packet.nodeID = NODE_ID;
  strncpy(packet.ssid, ssid, sizeof(packet.ssid) - 1);
  memcpy(packet.bssid, bssid, 6);
  packet.rssi = rssi;
  packet.endOfScan = false;

  esp_err_t result = esp_now_send(
    masterMAC,
    (const uint8_t*)&packet,
    sizeof(packet)
  );

  if (result != ESP_OK) {
    Serial.printf("Network send error: %d\n", result);
  }

  // Allow time between queued transmissions.
  delay(20);
}

void sendEndPacket() {
  WiFiPacket packet = {};

  packet.nodeID = NODE_ID;
  packet.endOfScan = true;

  esp_err_t result = esp_now_send(
    masterMAC,
    (const uint8_t*)&packet,
    sizeof(packet)
  );

  if (result != ESP_OK) {
    Serial.printf("End packet error: %d\n", result);
  }

  // Give the radio time to process the queued packet.
  delay(50);
}

void scanWiFi() {
  Serial.printf("\nNode %d: Scanning...\n", NODE_ID);

  // Scan nearby access points.
  int n = WiFi.scanNetworks(false, true);

  Serial.printf("Networks found: %d\n", n);

  if (n < 0) {
    Serial.printf("Wi-Fi scan failed: %d\n", n);
    WiFi.scanDelete();

    // Restore ESP-NOW channel before notifying master.
    esp_wifi_set_channel(
      ESPNOW_CHANNEL,
      WIFI_SECOND_CHAN_NONE
    );

    sendEndPacket();
    return;
  }

  // Print and transmit the scan results.
  for (int i = 0; i < n; i++) {
    Serial.printf(
      "%s | RSSI: %d dBm\n",
      WiFi.SSID(i).c_str(),
      WiFi.RSSI(i)
    );

    sendNetwork(
      WiFi.SSID(i).c_str(),
      WiFi.BSSID(i),
      WiFi.RSSI(i)
    );
  }

  WiFi.scanDelete();

  // Return to the fixed ESP-NOW channel.
  esp_err_t result = esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (result != ESP_OK) {
    Serial.printf("Channel restore failed: %d\n", result);
    return;
  }

  sendEndPacket();

  Serial.println("Scan transmission completed.");
}

void OnDataRecv(
  const esp_now_recv_info_t* info,
  const uint8_t* data,
  int len
) {
  if (len != sizeof(CommandPacket)) {
    return;
  }

  CommandPacket cmd;
  memcpy(&cmd, data, sizeof(cmd));

  if (cmd.command == CMD_SCAN) {
    scanRequested = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);

  // Disconnect from access points without turning off Wi-Fi.
  WiFi.disconnect(false, true);

  delay(100);

  Serial.println("\n=== ANTENNA NODE STARTUP ===");
  Serial.printf("Node ID: %d\n", NODE_ID);
  Serial.print("Node STA MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Configured master MAC: ");
  Serial.printf(
    "%02X:%02X:%02X:%02X:%02X:%02X\n",
    masterMAC[0], masterMAC[1], masterMAC[2],
    masterMAC[3], masterMAC[4], masterMAC[5]
  );

  esp_err_t result = esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (result != ESP_OK) {
    Serial.printf("Channel setup failed: %d\n", result);
    return;
  }

  result = esp_now_init();

  if (result != ESP_OK) {
    Serial.printf("ESP-NOW init failed: %d\n", result);
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(masterMAC)) {
    result = esp_now_add_peer(&peer);

    if (result != ESP_OK) {
      Serial.printf("Adding master failed: %d\n", result);
      return;
    }
  }

  espNowReady = true;
  Serial.println("ESP-NOW READY");
  Serial.println("Waiting for scan command...");
}

void loop() {
  if (espNowReady && scanRequested) {
    scanRequested = false;
    scanWiFi();
  }
}
