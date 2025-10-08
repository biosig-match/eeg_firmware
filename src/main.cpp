#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- ピン定義 ---
const int PIN_SPI_CS = D1;
const int PIN_DRDY   = D0;

// --- SPI設定 ---
SPISettings ads1299_spi_settings(1000000, MSBFIRST, SPI_MODE1);

// --- ADS1299 コマンド定義 ---
const uint8_t CMD_RESET  = 0x06;
const uint8_t CMD_SDATAC = 0x11;
const uint8_t CMD_RDATAC = 0x10;
const uint8_t CMD_START  = 0x08;
const uint8_t CMD_RDATA  = 0x12;
const uint8_t CMD_WREG   = 0x40;
const uint8_t CMD_RREG   = 0x20;

// --- グローバル変数 ---
uint8_t numChannels = 4;  // デフォルト4チャンネル（起動時に自動検出）
uint8_t dataBytes = 15;   // 3(ステータス) + 4チャンネル * 3バイト

// --- BLE設定 ---
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321"
#define SAMPLES_PER_PACKET 25  // 1パケットに含めるサンプル数（250SPS ÷ 10Hz = 25サンプル）

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

// データバッファ（25サンプル分のチャンネルデータ + GPIO/Option）
uint8_t sampleBuffer[SAMPLES_PER_PACKET][18];  // 各サンプル18バイト
uint8_t sampleIndex = 0;

// シリアル出力の間引き設定
uint16_t serialPrintCounter = 0;
const uint16_t SERIAL_PRINT_INTERVAL = 50;  // 50サンプルに1回出力（250SPS÷50=5Hz）

// BLE送信タイミング
unsigned long lastBleNotifyTime = 0;
const unsigned long BLE_NOTIFY_INTERVAL_MS = 100;  // 100ms = 10Hz

// BLE接続状態管理
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected");
    BLEDevice::startAdvertising(); // 再度アドバタイズ開始
  }
};

/**
 * @brief ADS1299の単一レジスタから読み取りを行う
 * @param address 読み取るレジスタのアドレス
 * @return 読み取った値
 */
uint8_t readRegister(uint8_t address) {
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_RREG | address);
  SPI.transfer(0x00);  // 1レジスタのみ読み取り
  delayMicroseconds(10);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  return value;
}

/**
 * @brief ADS1299の単一レジスタに書き込みを行う
 * @param address 書き込むレジスタのアドレス
 * @param value 書き込む値
 */
void writeRegister(uint8_t address, uint8_t value) {
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_WREG | address);
  SPI.transfer(0x00);
  SPI.transfer(value);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
}

/**
 * @brief IDレジスタからチャンネル数を検出
 * @return チャンネル数 (4, 6, or 8)
 */
uint8_t detectChannelCount() {
  uint8_t idReg = readRegister(0x00);  // IDレジスタ読み取り
  uint8_t nu_ch = idReg & 0x03;        // 下位2ビット取得

  uint8_t channels;
  switch (nu_ch) {
    case 0x00: channels = 4; break;  // ADS1299-4
    case 0x01: channels = 6; break;  // ADS1299-6
    case 0x02: channels = 8; break;  // ADS1299
    default:   channels = 4; break;  // デフォルト
  }

  Serial.print("Detected ADS1299 with ");
  Serial.print(channels);
  Serial.println(" channels");

  return channels;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- ADS1299 4-Channel BLE Measurement Start ---");

  // BLE初期化
  Serial.println("Initializing BLE...");
  BLEDevice::init("ADS1299_EEG");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising started. Waiting for client...");

  // ピン初期化
  pinMode(PIN_SPI_CS, OUTPUT);
  digitalWrite(PIN_SPI_CS, HIGH);
  pinMode(PIN_DRDY, INPUT);

  // SPI初期化
  SPI.begin();

  // 1. デバイスのリセットとSDATAC
  Serial.println("Step 1: Initializing device...");

  // まずSDATACでモードをリセット
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_SDATAC);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  delay(10);

  // デバイスリセット
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_RESET);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  delay(100); // リセット後の安定化待機

  // チャンネル数を自動検出
  numChannels = detectChannelCount();
  dataBytes = 3 + (numChannels * 3);
  Serial.print("Data packet size: ");
  Serial.print(dataBytes);
  Serial.println(" bytes");

  // 2. ADS1299の設定
  Serial.print("Step 2: Configuring for ");
  Serial.print(numChannels);
  Serial.println("-channel measurement...");

  // CONFIG1: データレートを250SPSに設定
  writeRegister(0x01, 0x96);

  // CONFIG3: 内部基準電圧バッファを有効化（BIAS回路は使用しない）
  writeRegister(0x03, 0xE0); // PD_REFBUF=1, 予約=11, BIASREF_INT=0, PD_BIAS=0

  // MISC1: SRB1を全チャンネルのマイナス入力に接続
  writeRegister(0x15, 0x20);

  // 全チャンネルを有効化、ゲイン24倍、通常入力に設定
  for (uint8_t ch = 0; ch < numChannels; ch++) {
    writeRegister(0x05 + ch, 0x60);  // CHnSET
  }

  // 未使用チャンネルはパワーダウン
  for (uint8_t ch = numChannels; ch < 8; ch++) {
    writeRegister(0x05 + ch, 0x81);  // Power-down, Input short
  }

  // 3. 変換開始 & RDATACモードへ移行
  Serial.println("Step 3: Starting conversions and entering RDATAC mode...");

  // 変換開始
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_START);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();
  delay(10);

  // RDATACモードに入る（以降はコマンドなしでデータ読み出し可能）
  SPI.beginTransaction(ads1299_spi_settings);
  digitalWrite(PIN_SPI_CS, LOW);
  SPI.transfer(CMD_RDATAC);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.endTransaction();

  Serial.println("Configuration complete. Outputting CH data...");
  Serial.println("------------------------------------");
}

void loop() {
  // DRDYピンがLOWになったらデータを読み出す
  if (digitalRead(PIN_DRDY) == LOW) {

    // 動的サイズのバッファを使用
    uint8_t data_buffer[27];  // 最大27バイト（8ch版対応）

    // RDATACモードなのでコマンド不要、いきなりデータ受信
    SPI.beginTransaction(ads1299_spi_settings);
    digitalWrite(PIN_SPI_CS, LOW);
    for (int i = 0; i < dataBytes; i++) {
      data_buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();

    // ステータスバイト3からGPIO[4:1]を抽出（下位4ビット）
    uint8_t gpio_status = data_buffer[2] & 0x0F;

    // 1サンプル分のデータ（18バイト）を作成
    uint8_t sample_data[18];
    memset(sample_data, 0, 18);

    // 各チャンネルのデータを処理
    for (uint8_t ch = 0; ch < numChannels; ch++) {
      uint8_t offset = 3 + (ch * 3);

      // 24ビット符号付き整数に変換
      int32_t ch_24bit = (int32_t)((data_buffer[offset] << 24) |
                                    (data_buffer[offset + 1] << 16) |
                                    (data_buffer[offset + 2] << 8)) >> 8;

      // 24ビットから16ビットに変換
      int16_t ch_16bit = (int16_t)(ch_24bit >> 8);

      // サンプルデータに格納
      sample_data[ch * 2] = (ch_16bit >> 8) & 0xFF;
      sample_data[ch * 2 + 1] = ch_16bit & 0xFF;
    }

    // GPIO情報とオプション情報を追加
    sample_data[16] = gpio_status << 4;
    sample_data[17] = 0x00;

    // サンプルバッファに蓄積
    memcpy(sampleBuffer[sampleIndex], sample_data, 18);
    sampleIndex++;

    // シリアル出力（間引き：50サンプルに1回）
    serialPrintCounter++;
    if (serialPrintCounter >= SERIAL_PRINT_INTERVAL) {
      serialPrintCounter = 0;
      for (uint8_t ch = 0; ch < numChannels; ch++) {
        int16_t ch_16bit = (sample_data[ch * 2] << 8) | sample_data[ch * 2 + 1];
        Serial.print("CH");
        Serial.print(ch + 1);
        Serial.print(":");
        Serial.print(ch_16bit);
        Serial.print(" ");
      }
      Serial.print("GPIO:");
      Serial.print(gpio_status, HEX);
      Serial.print(" SmpIdx:");
      Serial.println(sampleIndex);
    }
  }

  // BLE送信処理（25サンプル溜まったら、100msごとに送信）
  if (deviceConnected && sampleIndex >= SAMPLES_PER_PACKET) {
    unsigned long currentTime = millis();
    if (currentTime - lastBleNotifyTime >= BLE_NOTIFY_INTERVAL_MS) {
      // 25サンプル × 18バイト = 450バイトを送信
      // BLEの最大MTUは通常512バイトなので問題なし
      pCharacteristic->setValue(sampleBuffer[0], SAMPLES_PER_PACKET * 18);
      pCharacteristic->notify();

      sampleIndex = 0;  // バッファリセット
      lastBleNotifyTime = currentTime;
    }
  }
}