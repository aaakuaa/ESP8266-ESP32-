#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/i2s.h>

const char* ssid = "AAA507";
const char* password = "13684289887";

WiFiUDP udpAudio;
WiFiUDP udpSync;

const int udpPortAudio = 8888;
const int udpPortSync = 9999;
IPAddress broadcastIP(192, 168, 4, 255);

const int sampleRate = 11025;
const int sampleCount = 221;

struct ClientBuffer {
  bool valid = false;
  uint32_t timestamp = 0;
  int16_t samples[sampleCount];
};

ClientBuffer clientBuffers[3];
unsigned long lastSync = 0;

// ====== I2S 配置 ======
void setupI2S() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = sampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num = 26,
    .ws_io_num = 25,
    .data_out_num = 22,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

// ====== 软限制函数 ======
int16_t softLimit(int32_t sample) {
  // 简化版 Soft Clipping：缩放到 -1 ~ 1 范围内，再乘以最大幅度
  const float maxAmp = 32767.0;
  float s = sample / maxAmp;

  // tanh-like soft clipping
  float out = s / (1 + fabs(s));
  return (int16_t)(out * maxAmp);
}

// ====== 均值滤波器 ======
void applyMovingAverage(int16_t* buffer) {
  int16_t temp[sampleCount];
  temp[0] = buffer[0];
  for (int i = 1; i < sampleCount - 1; i++) {
    temp[i] = (buffer[i - 1] + buffer[i] + buffer[i + 1]) / 3;
  }
  temp[sampleCount - 1] = buffer[sampleCount - 1];

  memcpy(buffer, temp, sizeof(temp));
}

// ====== 同步信号发送 ======
void sendSyncSignal() {
  uint32_t now = millis();
  udpSync.beginPacket(broadcastIP, udpPortSync);
  udpSync.write('S');
  udpSync.write((uint8_t*)&now, sizeof(now));
  udpSync.endPacket();
}

// ====== 初始化 ======
void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  udpAudio.begin(udpPortAudio);
  udpSync.begin(udpPortSync);
  setupI2S();
  Serial.println("Server started.");
}

// ====== 主循环 ======
void loop() {
  // 每 20ms 同步
  if (millis() - lastSync >= 20) {
    lastSync = millis();
    sendSyncSignal();
  }

  // 接收音频包
  while (udpAudio.parsePacket()) {
    uint8_t id;
    udpAudio.read(&id, 1);
    uint32_t timestamp;
    int bytesRead = udpAudio.read((uint8_t*)&timestamp, sizeof(timestamp));//读取时间戳

    int16_t tmp[sampleCount];
    bytesRead += udpAudio.read((uint8_t*)tmp, sizeof(tmp));//继续读取数据帧

    if (bytesRead == sizeof(timestamp) + sizeof(tmp) && id >= 1 && id <= 3) {   //检查数据完整性
      int index = id - 1;
      clientBuffers[index].valid = true;
      clientBuffers[index].timestamp = timestamp;
      memcpy(clientBuffers[index].samples, tmp, sizeof(tmp));
    }
  }

  // 选择最早 timestamp
  uint32_t baseTimestamp = UINT32_MAX;

  //遍历三个客户端的缓冲区，选出最早的时间戳。
  for (int i = 0; i < 3; i++) {    
    if (clientBuffers[i].valid && clientBuffers[i].timestamp < baseTimestamp) {
      baseTimestamp = clientBuffers[i].timestamp; 
    }
  }
  if (baseTimestamp == UINT32_MAX) return;  //数据失效跳出loop一次

  int present = 0;
  int32_t mixed[sampleCount] = {0};

  for (int i = 0; i < 3; i++) {
    if (clientBuffers[i].valid &&
        //容忍20ms的时间戳差值
        abs((int32_t)(clientBuffers[i].timestamp - baseTimestamp)) <= 20) {
      for (int j = 0; j < sampleCount; j++) {
        mixed[j] += clientBuffers[i].samples[j];
      }
      present++;
    }
  }

  if (present > 0) {
    float gain = 4.0;
    int16_t output[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      int32_t val = mixed[i] / present;       // 衰减混音

      val = (int32_t)(val * gain);

      output[i] = softLimit(val);             // 限幅
    }

    applyMovingAverage(output);               // 滤波

    // 创建立体声缓冲区（每个样本复制到左右声道）
    int16_t stereoOutput[sampleCount * 2];
    for (int i = 0; i < sampleCount; i++) {
      stereoOutput[i * 2] = output[i];     // 左声道
      stereoOutput[i * 2 + 1] = output[i]; // 右声道
    }

    size_t written;
    i2s_write(I2S_NUM_0, stereoOutput, sizeof(stereoOutput), &written, portMAX_DELAY);
  }

  // 清除标志
  for (int i = 0; i < 3; i++) {
    clientBuffers[i].valid = false;
  }
}
