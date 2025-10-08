# ADS1299 BLE Data Receiver

ESP32 (XIAO ESP32-S3) + ADS1299からBLE経由でEEGデータを受信し、リアルタイムで可視化するPythonアプリケーション。

## 機能

- BLE経由で18バイトのデータパケットを受信
- 8チャンネルの16bitデータをリアルタイム表示
- GPIO状態とオプション情報をビット形式で表示
- PyQt6ベースのGUI
- 約60 FPSでグラフ更新

## データフォーマット

```
バイト0-1:   CH1 (16bit符号付き整数)
バイト2-3:   CH2
バイト4-5:   CH3
バイト6-7:   CH4
バイト8-9:   CH5
バイト10-11: CH6
バイト12-13: CH7
バイト14-15: CH8
バイト16:    GPIO[4:1] (上位4bit) + 予約 (下位4bit)
バイト17:    オプション情報
```

## セットアップ

### 1. 仮想環境の作成と有効化

```bash
# 仮想環境を作成
python3 -m venv venv

# 仮想環境を有効化（macOS/Linux）
source venv/bin/activate

# 仮想環境を有効化（Windows）
venv\Scripts\activate
```

### 2. 依存パッケージのインストール

```bash
pip install -r requirements.txt
```

### 3. 実行

```bash
python ble_receiver.py
```

## 使い方

1. ESP32のファームウェアをアップロードして起動
2. Pythonスクリプトを実行
3. GUIの「Connect」ボタンをクリック
4. デバイス `ADS1299_EEG` が自動検出されて接続
5. 8チャンネルのEEGデータがリアルタイム表示される

## トラブルシューティング

### Bluetooth権限エラー（macOS）

macOSの場合、Bluetooth権限が必要です：
- システム環境設定 → セキュリティとプライバシー → プライバシー → Bluetooth
- Pythonまたはターミナルアプリに権限を付与

### デバイスが見つからない

- ESP32が起動していることを確認
- BLEアドバタイジングが有効になっていることを確認
- 近くに他のBLEデバイスが多い場合はスキャン時間を延長

### 依存パッケージのエラー

```bash
# PyQt6のインストールに失敗する場合
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

## カスタマイズ

### グラフ表示時間の変更

`ble_receiver.py`の`BUFFER_SIZE`を変更：

```python
BUFFER_SIZE = 1000  # 500 SPS × 2秒 = 1000サンプル
```

### UUID/デバイス名の変更

main.cppとble_receiver.pyの以下の定数を一致させる：

```python
SERVICE_UUID = "12345678-1234-1234-1234-123456789012"
CHARACTERISTIC_UUID = "87654321-4321-4321-4321-210987654321"
DEVICE_NAME = "ADS1299_EEG"
```

## 必要な環境

- Python 3.8以上
- Bluetooth対応のPC/Mac
- ESP32 (XIAO ESP32-S3) + ADS1299ファームウェア
