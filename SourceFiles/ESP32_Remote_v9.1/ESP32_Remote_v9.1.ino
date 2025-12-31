
/*
 * JOE'S DRIVE - FINAL: S2S + PID BALANCE + 3-SEC CALIBRATION
 * ESP32 Remote v9.15 — enhanced ESPNOW mirroring with reliable send-queue
 * (Remote device)
 */


#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ===== Put the PRIMARY ESP32 STA MAC here (the device sending logs) =====
// Provided by you: C4:5B:BE:90:6A:24
uint8_t primaryMAC[] = {0xC4, 0x5B, 0xBE, 0x90, 0x6A, 0x24};

// OPTIONAL: If you want to send commands outbound to the primary
// set this to true and uncomment the "send a test command" inside loop.
const bool kRemoteSendsCommands = false;

// ----- Utilities (optional) -----
static inline void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Log ONLY send failures to avoid spam
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.print("[ESPNOW] Send FAIL to ");
    printMac(mac_addr);
    Serial.printf(" (status=%d)\n", (int)status);
  }
}

// Print incoming payloads exactly as sent by the primary

// Print incoming payloads exactly as sent by the primary (no extra newlines)
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Serial.print("[RX ");
  // printMac(mac);
  // Serial.print("] ");

  // Print payload verbatim, but skip carriage returns if you want clean output
  for (int i = 0; i < len; ++i) {
    char c = (char)incomingData[i];
    if (c != '\r') Serial.print(c);  // skip '\r' to match primary formatting    if (c != '\r') Serial.print(c);  // skip '\r' to match primary formatting
  }
  // ✅ Do NOT add another newline here
}
// void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//   // Prefix with sender MAC (optional), then raw payload
//   Serial.print("[RX ");
//   printMac(mac);
//   Serial.print("] ");

//   // Payload may include '\n' already; print bytes verbatim
//   for (int i = 0; i < len; ++i) {
//     Serial.print((char)incomingData[i]);
//   }

//   // If the last byte isn't a newline, add one for readability
//   if (len == 0 || incomingData[len - 1] != '\n') {
//     Serial.print('\n');
//   }
// }

void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  Serial.printf("[MAC] WiFi (STA): %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] Init Failed");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Add the PRIMARY as a peer (we only need one-way pairing to receive from it)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, primaryMAC, 6);
  peerInfo.channel = 0;     // use current STA channel
  peerInfo.encrypt = false; // must match sender
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.print("[ESPNOW] Failed to add PRIMARY peer: ");
    printMac(primaryMAC);
    Serial.println();
  } else {
    Serial.println("Remote ESPNOW ready. Type commands:");
  }
}


void loop() {
  // Bridge remote Serial input → ESPNOW → primary
  if (Serial.available()) {
    static char cmdBuf[256];
    size_t n = Serial.readBytesUntil('\n', cmdBuf, sizeof(cmdBuf) - 1);
    cmdBuf[n] = '\0';  // terminate

    // Send to PRIMARY; ensure primaryMAC[] is set to your primary STA MAC (C4:5B:BE:90:6A:24)
    esp_now_send(primaryMAC, (const uint8_t*)cmdBuf, n + 1);

    // Optional:    // Optional: local echo so you know it was sent
    Serial.printf("[TX → PRIMARY] %s\n", cmdBuf);
  }

  delay(1);
}