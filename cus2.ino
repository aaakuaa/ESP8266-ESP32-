#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <I2S.h>

const char* ssid = "AAA507";
const char* password = "13684289887";

WiFiUDP udpAudio;
WiFiUDP udpSync;

IPAddress serverIP(192, 168, 4, 1);
const int udpPortAudio = 8888;
const int udpPortSync = 9999;

const uint8_t clientID = 2;  // 1~3 根据客户端区分

const int sampleRate = 11025;
const int sampleCount = 221;

int16_t samples[sampleCount];

uint32_t t0 = 0;
bool synced = false;

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
}

void collectAndSend(uint32_t timestamp) {
  for (int i = 0; i < sampleCount; i++) {
    int16_t l, r;
    i2s_read_sample(&l, &r, true);
    samples[i] = l;
  }

  udpAudio.beginPacket(serverIP, udpPortAudio);
  udpAudio.write(&clientID, 1);
  udpAudio.write((uint8_t*)&timestamp, sizeof(timestamp));
  udpAudio.write((uint8_t*)samples, sizeof(samples));
  udpAudio.endPacket();

  Serial.printf("Sent %d samples (ID: %d, ts: %lu)\n", sampleCount, clientID, timestamp);
}

void setup() {
  system_update_cpu_freq(160);
  Serial.begin(115200);

  i2s_rxtx_begin(true, false);
  i2s_set_rate(sampleRate);

  connectWiFi();

  udpAudio.begin(udpPortAudio);
  udpSync.begin(udpPortSync);

  Serial.println("Client ready, waiting for sync...");
}

void loop() {
  if (udpSync.parsePacket()) {
    char c;
    udpSync.read(&c, 1);
    if (c == 'S') {
      uint32_t serverTime;
      udpSync.read((char*)&serverTime, sizeof(serverTime));
      if (!synced) {
        t0 = millis() - serverTime; // 计算本地与服务端时间差
        synced = true;
        Serial.printf("Synchronized. Time offset: %lu\n", t0);
      }
      if (synced) {
        uint32_t timestamp = millis() - t0;
        collectAndSend(timestamp);
      }
    }
  }
}
