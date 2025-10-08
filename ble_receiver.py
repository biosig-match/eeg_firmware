#!/usr/bin/env python3
"""
ADS1299 BLE Data Receiver with Real-time Visualization
PyQt5を使用して8チャンネルのEEGデータをリアルタイム表示
"""

import sys
import struct
import asyncio
from collections import deque
import numpy as np
from bleak import BleakClient, BleakScanner
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLabel
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import pyqtgraph as pg

# BLE UUID設定（main.cppと一致させる）
SERVICE_UUID = "12345678-1234-1234-1234-123456789012"
CHARACTERISTIC_UUID = "87654321-4321-4321-4321-210987654321"
DEVICE_NAME = "ADS1299_EEG"

# データバッファサイズ（表示する時間窓）
BUFFER_SIZE = 500  # 250 SPS * 2秒分


class BLEThread(QThread):
    """BLE通信を別スレッドで実行"""
    data_received = pyqtSignal(list, int, int)  # (channel_data, gpio, option)
    connection_status = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = False
        self.client = None

    def run(self):
        """スレッドのメイン処理"""
        self.running = True
        asyncio.run(self.ble_main())

    async def ble_main(self):
        """BLE接続とデータ受信のメイン処理"""
        try:
            # デバイスをスキャン
            self.connection_status.emit("Scanning for devices...")
            devices = await BleakScanner.discover(timeout=5.0)

            target_device = None
            for device in devices:
                if device.name == DEVICE_NAME:
                    target_device = device
                    break

            if not target_device:
                self.connection_status.emit(f"Device '{DEVICE_NAME}' not found")
                return

            self.connection_status.emit(f"Connecting to {target_device.name}...")

            # BLE接続
            async with BleakClient(target_device.address) as client:
                self.client = client
                self.connection_status.emit(f"Connected to {target_device.name}")

                # 通知を有効化
                await client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)

                # 接続を維持
                while self.running:
                    await asyncio.sleep(0.1)

                await client.stop_notify(CHARACTERISTIC_UUID)

        except Exception as e:
            self.connection_status.emit(f"Error: {str(e)}")

    def notification_handler(self, sender, data: bytearray):
        """BLE通知受信時のコールバック"""
        # 新フォーマット: 25サンプル × 18バイト = 450バイト
        expected_size = 25 * 18
        if len(data) != expected_size:
            print(f"Warning: Expected {expected_size} bytes, got {len(data)}")
            return

        # 25サンプル分のデータをパース
        for sample_idx in range(25):
            offset = sample_idx * 18

            # 各サンプルの18バイトをパース
            sample_data = data[offset:offset+18]

            # 8チャンネルのデータ
            channel_data = []
            for i in range(8):
                # 16ビット符号付き整数（ビッグエンディアン）
                value = struct.unpack('>h', sample_data[i*2:(i*2)+2])[0]
                channel_data.append(value)

            # GPIO情報（バイト16の上位4ビット）
            gpio = (sample_data[16] >> 4) & 0x0F

            # オプション情報（バイト17）
            option = sample_data[17]

            # シグナルを発行（各サンプルごとに）
            self.data_received.emit(channel_data, gpio, option)

    def stop(self):
        """スレッドを停止"""
        self.running = False


class MainWindow(QMainWindow):
    """メインウィンドウ"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADS1299 EEG Real-time Monitor")
        self.setGeometry(100, 100, 1200, 800)

        # データバッファ（8チャンネル分）
        self.data_buffers = [deque(maxlen=BUFFER_SIZE) for _ in range(8)]
        self.time_buffer = deque(maxlen=BUFFER_SIZE)
        self.sample_count = 0

        # BLEスレッド
        self.ble_thread = BLEThread()
        self.ble_thread.data_received.connect(self.on_data_received)
        self.ble_thread.connection_status.connect(self.on_connection_status)

        # UI構築
        self.setup_ui()

    def setup_ui(self):
        """UI要素の配置"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # 接続状態ラベル
        self.status_label = QLabel("Status: Not connected")
        layout.addWidget(self.status_label)

        # GPIO/Option情報ラベル
        self.info_label = QLabel("GPIO: ---- | Option: --")
        layout.addWidget(self.info_label)

        # 接続ボタン
        btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.start_connection)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.stop_connection)
        self.disconnect_btn.setEnabled(False)

        btn_layout.addWidget(self.connect_btn)
        btn_layout.addWidget(self.disconnect_btn)
        layout.addLayout(btn_layout)

        # グラフウィジェット（8チャンネル分）
        self.plots = []
        self.curves = []

        # PyQtGraphの設定
        pg.setConfigOptions(antialias=True)

        for i in range(8):
            plot_widget = pg.PlotWidget()
            plot_widget.setLabel('left', f'CH{i+1}', units='LSB')
            plot_widget.setLabel('bottom', 'Samples')
            plot_widget.setYRange(-32768, 32767)
            plot_widget.showGrid(x=True, y=True)

            curve = plot_widget.plot(pen=pg.mkPen(color=(0, 150, 255), width=1))

            self.plots.append(plot_widget)
            self.curves.append(curve)
            layout.addWidget(plot_widget)

        # グラフ更新タイマー（60 FPS）
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(16)  # 約60 FPS

    def start_connection(self):
        """BLE接続開始"""
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.ble_thread.start()

    def stop_connection(self):
        """BLE接続終了"""
        self.ble_thread.stop()
        self.ble_thread.wait()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.status_label.setText("Status: Disconnected")

    def on_connection_status(self, status: str):
        """接続状態更新"""
        self.status_label.setText(f"Status: {status}")

    def on_data_received(self, channel_data: list, gpio: int, option: int):
        """データ受信時の処理"""
        # タイムスタンプ
        self.time_buffer.append(self.sample_count)
        self.sample_count += 1

        # 各チャンネルのデータをバッファに追加
        for i in range(8):
            self.data_buffers[i].append(channel_data[i])

        # GPIO/Option情報を表示（ビット表現）
        gpio_bits = format(gpio, '04b')
        option_bits = format(option, '08b')
        self.info_label.setText(f"GPIO: {gpio_bits} (0x{gpio:X}) | Option: {option_bits} (0x{option:02X})")

    def update_plots(self):
        """グラフ更新"""
        if len(self.time_buffer) == 0:
            return

        time_array = np.array(self.time_buffer)

        for i in range(8):
            if len(self.data_buffers[i]) > 0:
                data_array = np.array(self.data_buffers[i])
                self.curves[i].setData(time_array, data_array)

    def closeEvent(self, event):
        """ウィンドウを閉じる時の処理"""
        self.ble_thread.stop()
        self.ble_thread.wait()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
