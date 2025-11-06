#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>

// ========= Hardware Pins =========
const int PIN_SPI_CS = D1;
const int PIN_DRDY   = D0;

// ========= SPI Settings =========
SPISettings ads1299_spi_settings(1000000, MSBFIRST, SPI_MODE1);

// ========= ADS1299 Commands =========
const uint8_t CMD_RESET  = 0x06;
const uint8_t CMD_SDATAC = 0x11;
const uint8_t CMD_RDATAC = 0x10;
const uint8_t CMD_START  = 0x08;
const uint8_t CMD_RDATA  = 0x12;
const uint8_t CMD_WREG   = 0x40;
const uint8_t CMD_RREG   = 0x20;

// ========= Stream Parameters =========
#define SAMPLE_RATE_HZ      250
#define SAMPLES_PER_CHUNK   25      // 250SPS / 10Hz = 25
#define CH_MAX              8       // ADS1299最大8ch
#define DEVICE_NAME         "ADS1299_EEG_NUS"

// ========= BLE (NUS-like) UUIDs =========
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Notify
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Write

// ========= Packet Types =========
#define PKT_TYPE_DATA_CHUNK   0x66
#define PKT_TYPE_DEVICE_CFG   0xDD

// ========= Control Commands over RX =========
#define CMD_START_STREAMING   0xAA
#define CMD_STOP_STREAMING    0x5B

// ========= Packed Structures (LE) =========
struct __attribute__((packed)) ElectrodeConfig {
  char    name[8];    // e.g., "CH1", "Fp1" ...
  uint8_t type;       // reserved type (0=EEG)
  uint8_t reserved;
};

// IMUなし（将来拡張用に予約バイトを確保）
struct __attribute__((packed)) SampleData {
  int16_t signals[CH_MAX]; // ADS1299変換後（符号付き16bit, BE→int16化済）
  uint8_t trigger_state;   // GPIO下位4bit（0..15）
  uint8_t reserved[3];     // 将来IMUやフラグ等の拡張用
};

struct __attribute__((packed)) ChunkedSamplePacket {
  uint8_t  packet_type;      // 0x66
  uint16_t start_index;      // このチャンクの先頭サンプル番号（LE）
  uint8_t  num_samples;      // チャンク内サンプル数 (25)
  SampleData samples[SAMPLES_PER_CHUNK];
};

struct __attribute__((packed)) DeviceConfigPacket {
  uint8_t  packet_type;      // 0xDD
  uint8_t  num_channels;     // 実使用ch数（4/6/8）
  uint8_t  reserved[6];      // 将来用（Fs, format, flags等を載せても良い）
  ElectrodeConfig configs[CH_MAX];
};

// ========= Globals =========
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;

bool deviceConnected = false;
bool isStreaming     = false;

uint8_t  numChannels = 4;  // 起動時に検出
uint16_t sampleIndexCounter = 0;  // 総サンプル連番（wrap可）
unsigned long lastBleNotifyTime = 0;
const unsigned long BLE_NOTIFY_INTERVAL_MS = 100;  // ≒10Hz

// バッファ（25サンプル分）
SampleData sampleBuffer[SAMPLES_PER_CHUNK];
uint8_t    sampleBufferIndex = 0;

// 送信パケットをグローバル変数として確保し、スタックオーバーフローを回避
ChunkedSamplePacket chunkPacket;
// 設定情報パケットもグローバル変数として確保
DeviceConfigPacket deviceConfigPacket;

// BLEコールバックからメインループへ処理を依頼するためのフラグ
volatile bool g_send_config_packet = false;

// デフォルトの電極ラベル（将来的に可変）
const ElectrodeConfig defaultElectrodes[CH_MAX] = {
  {"CH1", 0, 0}, {"CH2", 0, 0}, {"CH3", 0, 0}, {"CH4", 0, 0},
  {"CH5", 0, 0}, {"CH6", 0, 0}, {"CH7", 0, 0}, {"CH8", 0, 0}
};

// ========= BLE Callbacks =========
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    Serial.println(">>> [BLE] Client connected");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    isStreaming = false;
    BLEDevice::startAdvertising();
    Serial.println(">>> [BLE] Client DISCONNECTED. Streaming stopped. Advertising restarted.");
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    std::string v = ch->getValue();
    if (v.empty()) return;
    uint8_t cmd = static_cast<uint8_t>(v[0]);

    if (cmd == CMD_START_STREAMING) {
      isStreaming = true;
      sampleIndexCounter = 0;
      sampleBufferIndex  = 0;
      g_send_config_packet = true;
    } else if (cmd == CMD_STOP_STREAMING) {
      isStreaming = false;
      Serial.println("[CMD] Stop streaming");
    }
  }
};

// ========= ADS1299 Helpers =========
uint8_t readRegister(uint8_t address) {
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_RREG | address);
  SPI.transfer(0x00); // read 1 reg
  delayMicroseconds(10);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void writeRegister(uint8_t address, uint8_t value) {
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_WREG | address);
  SPI.transfer(0x00); // write 1 reg
  SPI.transfer(value);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
}

uint8_t detectChannelCount() {
  uint8_t idReg = readRegister(0x00);
  uint8_t nu_ch = idReg & 0x03;
  switch (nu_ch) {
    case 0x00: return 4; // ADS1299-4
    case 0x01: return 6; // ADS1299-6
    case 0x02: return 8; // ADS1299-8
    default:   return 4;
  }
}

// ========= Setup =========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ADS1299 BLE Streamer (NUS-style, LE packed) ---");

  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started (NUS-like: TX=Notify, RX=Write)");

  pinMode(PIN_SPI_CS, OUTPUT);
  digitalWrite(PIN_SPI_CS, HIGH);
  pinMode(PIN_DRDY, INPUT);

  SPI.begin();

  Serial.println("ADS1299 init...");
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_SDATAC);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  delay(10);

  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_RESET);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  delay(100);

  numChannels = detectChannelCount();
  if (numChannels > CH_MAX) numChannels = CH_MAX;
  Serial.printf("Detected %u channels\n", numChannels);

  writeRegister(0x01, 0x96); // 250SPS
  writeRegister(0x03, 0xE0); // Internal reference
  writeRegister(0x15, 0x20); // SRB1 to all channels

  for (uint8_t ch = 0; ch < numChannels; ch++) {
    writeRegister(0x05 + ch, 0x60); // Gain 24x, normal input
  }
  for (uint8_t ch = numChannels; ch < 8; ch++) {
    writeRegister(0x05 + ch, 0x81); // Power down unused channels
  }

  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_START);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  delay(10);

  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_RDATAC);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();

  Serial.println("ADS1299 configured. Waiting for DRDY + stream commands...");
}

// ========= Reading one sample (from RDATAC) =========
bool readOneAds1299Sample(SampleData& outSample) {
  if (digitalRead(PIN_DRDY) != LOW) return false;

  const uint8_t maxBytes = 3 + 8 * 3;
  uint8_t data_buffer[maxBytes];

  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  const uint8_t dataBytes = 3 + (numChannels * 3);
  for (int i = 0; i < dataBytes; i++) {
    data_buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();

  const uint8_t gpio_status = data_buffer[2] & 0x0F;

  for (uint8_t ch = 0; ch < numChannels; ch++) {
    const uint8_t off = 3 + ch * 3;
    int32_t v24 = (int32_t)((data_buffer[off] << 24) |
                              (data_buffer[off + 1] << 16) |
                              (data_buffer[off + 2] << 8)) >> 8;
    int16_t v16 = (int16_t)(v24 >> 8);
    outSample.signals[ch] = v16;
  }
  for (uint8_t ch = numChannels; ch < CH_MAX; ch++) {
    outSample.signals[ch] = 0;
  }

  const uint8_t triggerNibble = (gpio_status & 0x0F);
  outSample.trigger_state = triggerNibble;
  outSample.reserved[0] = triggerNibble;
  outSample.reserved[1] = triggerNibble ? 0xA5 : 0x00;
  outSample.reserved[2] = 0x00;
  return true;
}

void loop() {
  static uint32_t loop_counter = 0;
  if (++loop_counter % 500000 == 0) {
      Serial.printf("[LOOP] Heartbeat. isStreaming: %d, deviceConnected: %d\n", isStreaming, deviceConnected);
  }
  
  if (g_send_config_packet) {
    g_send_config_packet = false; 

    deviceConfigPacket.packet_type  = PKT_TYPE_DEVICE_CFG;
    deviceConfigPacket.num_channels = numChannels;
    memset(deviceConfigPacket.reserved, 0, sizeof(deviceConfigPacket.reserved));
    memcpy(deviceConfigPacket.configs, defaultElectrodes, sizeof(defaultElectrodes));
    pTxCharacteristic->setValue((uint8_t*)&deviceConfigPacket, sizeof(deviceConfigPacket));
    pTxCharacteristic->notify();
    
    Serial.println("[CMD] Start streaming -> Sent DeviceConfigPacket from main loop");
    delay(10);
  }

  if (isStreaming && deviceConnected) {
    if (readOneAds1299Sample(sampleBuffer[sampleBufferIndex])) {
      sampleBufferIndex++;
      sampleIndexCounter++; 

      if (sampleBufferIndex >= SAMPLES_PER_CHUNK) {
        const unsigned long now = millis();
        if (now - lastBleNotifyTime >= BLE_NOTIFY_INTERVAL_MS) {
          chunkPacket.packet_type = PKT_TYPE_DATA_CHUNK;
          chunkPacket.start_index = (uint16_t)(sampleIndexCounter - SAMPLES_PER_CHUNK);
          chunkPacket.num_samples = SAMPLES_PER_CHUNK;
          memcpy(chunkPacket.samples, sampleBuffer, sizeof(sampleBuffer));
          
          pTxCharacteristic->setValue((uint8_t*)&chunkPacket, sizeof(chunkPacket));
          pTxCharacteristic->notify();

          sampleBufferIndex = 0;
          lastBleNotifyTime = now;
          delay(10); // BLEスタックが処理する時間を少し与える
        }
      }
    } else {
      // データがない場合、CPUを1msだけ他のタスク（BLE等）に譲る。
      delay(1);     }
  } else {
    delay(10);
  }
}
