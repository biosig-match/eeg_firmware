#!/usr/bin/env python3
"""
ADS1299 BLE Data Receiver with Real-time Visualization and CSV Logging.
Communicates with the NUS-like BLE service defined in main.cpp.
- Handles two packet types: Device Configuration and Chunked Data.
- Provides controls to start/stop data streaming.
- Updates plot labels based on received configuration.
- Saves all streaming data to a timestamped CSV file.
- Converts LSB data to microvolts (uV).
- Displays plots with autorange.
- Applies a 50Hz notch filter and a 0.5-30Hz bandpass filter.
"""

import sys
import struct
import asyncio
import csv
from datetime import datetime
from collections import deque
import numpy as np
from scipy import signal
from bleak import BleakClient, BleakScanner
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLabel, QGridLayout
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import pyqtgraph as pg

# ========= BLE Settings (must match main.cpp) =========
SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # For receiving notifications
CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # For writing commands
DEVICE_NAME = "ADS1299_EEG_NUS"

# ========= Control Commands (must match main.cpp) =========
CMD_START_STREAMING = b'\xAA'
CMD_STOP_STREAMING = b'\x5B'

# ========= Packet Types (must match main.cpp) =========
PKT_TYPE_DATA_CHUNK = 0x66
PKT_TYPE_DEVICE_CFG = 0xDD

# ========= Packet Sizes (must match main.cpp) =========
DEVICE_CONFIG_PACKET_SIZE = 88
CHUNKED_SAMPLE_PACKET_SIZE = 504

# Display buffer size (time window) and sampling rate
BUFFER_SIZE = 750  # 250 SPS * 3 seconds
SAMPLING_RATE = 250 # Samples per second

# --- ▼▼▼ μV変換のための定数 ▼▼▼ ---
V_REF = 4.5  # ADS1299のリファレンス電圧 (通常は4.5V)
# ★★★ ファームウェアでのPGAゲイン設定に合わせてこの値を変更してください ★★★
PGA_GAIN = 24.0
# ★★★ 注意: ファームウェアから送られるデータが16ビット整数('<8h..')であるため、
# 16ビットの分解能で計算します。もしファームウェアが24ビットデータを
# 送るように変更された場合は、(2**23 - 1) に変更してください。
LSB_TO_MICROVOLTS = (V_REF / PGA_GAIN / (2**15 - 1)) * 1_000_000
# --- ▲▲▲ ---

class BLEThread(QThread):
    """
    Handles BLE communication in a separate thread.
    This class has been refactored to use a more robust asyncio event loop
    management pattern (`run_forever`) and includes a buffer to handle
    fragmented BLE packets.
    """
    connection_status = pyqtSignal(str)
    config_received = pyqtSignal(dict)
    data_received = pyqtSignal(list)  # Emits a list of samples

    def __init__(self):
        super().__init__()
        self.client = None
        self.rx_char = None
        self.loop = None
        self.main_task = None
        self.is_running = False
        self.rx_buffer = bytearray() # Buffer for fragmented packets

    def run(self):
        """
        Main thread entry point.
        Initializes and runs the asyncio event loop.
        """
        self.is_running = True
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.main_task = self.loop.create_task(self.ble_main())
            self.loop.run_forever()
        finally:
            if self.main_task and not self.main_task.done():
                self.loop.run_until_complete(self.main_task)
            self.loop.close()

    async def ble_main(self):
        """Main BLE connection and data reception logic."""
        client = None
        try:
            self.connection_status.emit(f"Scanning for '{DEVICE_NAME}'...")
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

            if not device:
                self.connection_status.emit(f"Device '{DEVICE_NAME}' not found.")
                return

            self.connection_status.emit(f"Connecting to {device.name} ({device.address})...")
            
            client = BleakClient(device.address, timeout=20.0)
            await client.connect()
            self.client = client
            
            self.connection_status.emit(f"Connected to {device.name}.")

            self.rx_char = client.services.get_characteristic(CHARACTERISTIC_UUID_RX)
            if not self.rx_char:
                self.connection_status.emit("RX Characteristic not found!")
                return

            await client.start_notify(CHARACTERISTIC_UUID_TX, self.notification_handler)
            self.connection_status.emit("Ready. Waiting for streaming command.")

            while self.is_running:
                await asyncio.sleep(0.1)

        except asyncio.CancelledError:
            self.connection_status.emit("BLE task was cancelled.")
        except Exception as e:
            self.connection_status.emit(f"An error occurred: {e}")
        finally:
            if client and client.is_connected:
                await client.disconnect()
            self.connection_status.emit("BLE connection closed.")
            self.client = None
            self.rx_char = None


    def notification_handler(self, sender, data: bytearray):
        """
        Callback for incoming BLE notifications.
        This now buffers and reassembles fragmented packets.
        """
        if not data:
            return
        
        self.rx_buffer.extend(data) # Add new data to the buffer

        # Process buffer as long as it might contain complete packets
        while True:
            if len(self.rx_buffer) < 1:
                break # Buffer is empty

            packet_type = self.rx_buffer[0]
            
            expected_len = 0
            if packet_type == PKT_TYPE_DEVICE_CFG:
                expected_len = DEVICE_CONFIG_PACKET_SIZE
            elif packet_type == PKT_TYPE_DATA_CHUNK:
                expected_len = CHUNKED_SAMPLE_PACKET_SIZE
            else:
                print(f"Warning: Unknown packet type 0x{packet_type:02X} in buffer. Clearing buffer.")
                self.rx_buffer.clear()
                break
            
            # Check if we have a full packet in the buffer
            if len(self.rx_buffer) >= expected_len:
                # Extract the full packet
                packet_data = self.rx_buffer[:expected_len]
                # Remove the extracted packet from the buffer
                self.rx_buffer = self.rx_buffer[expected_len:]

                # Parse the complete packet
                if packet_type == PKT_TYPE_DEVICE_CFG:
                    self.parse_config_packet(packet_data)
                elif packet_type == PKT_TYPE_DATA_CHUNK:
                    self.parse_data_chunk_packet(packet_data)
            else:
                # Not enough data for a full packet, wait for more
                break


    def parse_config_packet(self, data: bytearray):
        """Parses the DeviceConfigPacket (0xDD)."""
        if len(data) < DEVICE_CONFIG_PACKET_SIZE:
            print(f"Warning: Config packet is too short ({len(data)} bytes)")
            return
        
        _, num_channels = struct.unpack('<BB', data[0:2])
        config = {'num_channels': num_channels, 'electrodes': []}
        offset = 8 # packet_type(1), num_channels(1), reserved(6)
        for i in range(8): # CH_MAX is 8
            chunk = data[offset + i*10 : offset + (i+1)*10]
            name_bytes, type_val, _ = struct.unpack('<8sBB', chunk)
            name = name_bytes.partition(b'\0')[0].decode('utf-8', 'ignore')
            config['electrodes'].append({'name': name, 'type': type_val})
        
        print(f"Received device config: {num_channels} channels.")
        self.config_received.emit(config)

    def parse_data_chunk_packet(self, data: bytearray):
        """Parses the ChunkedSamplePacket (0x66)."""
        if len(data) < 4:
            print("Warning: Data chunk packet is too short for header.")
            return
        _, start_index, num_samples = struct.unpack('<B H B', data[0:4])
        
        expected_size = 4 + num_samples * 20 # 20 bytes per SampleData
        if len(data) != expected_size:
            print(f"Warning: Data chunk size mismatch. Expected {expected_size}, got {len(data)}.")
            return

        samples_list = []
        offset = 4
        for i in range(num_samples):
            sample_chunk = data[offset : offset + 20]
            # '<8hB3x' = 8 signed shorts (16 bytes), 1 unsigned byte, 3 padding bytes
            unpacked_data = struct.unpack('<8hB3x', sample_chunk)
            sample = {
                'signals': list(unpacked_data[0:8]),
                'trigger': unpacked_data[8],
                'sample_index': start_index + i
            }
            samples_list.append(sample)
            offset += 20
        
        self.data_received.emit(samples_list)

    def _send_command(self, command: bytes):
        if self.client and self.rx_char and self.client.is_connected and self.loop:
            asyncio.run_coroutine_threadsafe(
                self.client.write_gatt_char(self.rx_char, command, response=False),
                self.loop
            )
        else:
            self.connection_status.emit("Not connected, cannot send command.")

    def start_streaming(self):
        print("Sending START_STREAMING command...")
        self._send_command(CMD_START_STREAMING)

    def stop_streaming(self):
        print("Sending STOP_STREAMING command...")
        self._send_command(CMD_STOP_STREAMING)

    def stop(self):
        """Stops the thread by safely stopping the asyncio event loop."""
        self.is_running = False
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADS1299 EEG Real-time Monitor")
        self.setGeometry(100, 100, 1200, 900)

        self.num_channels = 8
        self.data_buffers = [deque(maxlen=BUFFER_SIZE) for _ in range(self.num_channels)]
        self.time_buffer = deque(maxlen=BUFFER_SIZE)
        self.sample_count = 0

        self.ble_thread = BLEThread()
        self.ble_thread.connection_status.connect(self.on_connection_status)
        self.ble_thread.config_received.connect(self.on_config_received)
        self.ble_thread.data_received.connect(self.on_data_received)

        self.setup_ui()
        self.is_streaming = False
        
        self.csv_file = None
        self.csv_writer = None
        self.csv_header_written = False

        # Timeout timer to check for the first packet after starting stream
        self.first_packet_timer = QTimer(self)
        self.first_packet_timer.setSingleShot(True)
        self.first_packet_timer.timeout.connect(self.check_first_packet_timeout)
        self.first_packet_received = False
        
        # --- ▼▼▼ フィルタ設計 ▼▼▼ ---
        self.fs = SAMPLING_RATE  # サンプリング周波数

        # 50Hzノッチフィルタの設計
        f0_notch = 50.0  # 除去したい周波数 (Hz)
        Q_notch = 30.0   # クオリティファクタ
        self.b_notch, self.a_notch = signal.iirnotch(f0_notch, Q_notch, self.fs)
        self.notch_filter_states = [signal.lfilter_zi(self.b_notch, self.a_notch) for _ in range(self.num_channels)]
        
        # 0.5-30Hz バンドパスフィルタの設計 (2次バターワース)
        nyquist = 0.5 * self.fs
        low_cutoff = 0.5
        high_cutoff = 30.0
        low = low_cutoff / nyquist
        high = high_cutoff / nyquist
        self.b_band, self.a_band = signal.butter(2, [low, high], btype='band')
        self.bandpass_filter_states = [signal.lfilter_zi(self.b_band, self.a_band) for _ in range(self.num_channels)]
        # --- ▲▲▲ フィルタ設計ここまで ▲▲▲ ---

    def setup_ui(self):
        """Set up all UI elements."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        self.status_label = QLabel("Status: Not connected")
        self.info_label = QLabel("Trigger: - | File: -")
        self.connect_btn = QPushButton("Connect")
        self.start_btn = QPushButton("Start Streaming")
        self.stop_btn = QPushButton("Stop Streaming")

        control_layout.addWidget(self.connect_btn)
        control_layout.addWidget(self.start_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addStretch(1)
        control_layout.addWidget(self.status_label)
        control_layout.addWidget(self.info_label)
        main_layout.addWidget(control_panel)

        self.connect_btn.clicked.connect(self.toggle_connection)
        self.start_btn.clicked.connect(self.start_streaming)
        self.stop_btn.clicked.connect(self.stop_streaming)

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)

        self.plots = []
        self.curves = []
        pg.setConfigOptions(antialias=True)
        
        plot_grid = QGridLayout()
        main_layout.addLayout(plot_grid)

        for i in range(self.num_channels):
            plot_widget = pg.PlotWidget()
            plot_widget.setLabel('left', f'CH{i+1}', units='μV') 
            plot_widget.setLabel('bottom', 'Samples')
            # YRangeの固定を削除し、オートレンジを有効化
            plot_widget.showGrid(x=True, y=True)
            curve = plot_widget.plot(pen=pg.mkPen(color=(0, 150, 255), width=1))
            
            self.plots.append(plot_widget)
            self.curves.append(curve)
            plot_grid.addWidget(plot_widget, i // 2, i % 2)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(33)

    def toggle_connection(self):
        if not self.ble_thread.isRunning():
            self.connect_btn.setText("Disconnect")
            self.ble_thread.start()
        else:
            self.stop_streaming()
            self.ble_thread.stop()
            self.ble_thread.wait()
            self.connect_btn.setText("Connect")
            self.on_connection_status("Disconnected by user.")

    def start_streaming(self):
        if self.ble_thread.isRunning() and not self.is_streaming:
            for buf in self.data_buffers:
                buf.clear()
            self.time_buffer.clear()
            self.sample_count = 0
            
            # --- ▼▼▼ ストリーミング開始時に両方のフィルタ状態をリセット ▼▼▼ ---
            self.notch_filter_states = [signal.lfilter_zi(self.b_notch, self.a_notch) for _ in range(self.num_channels)]
            self.bandpass_filter_states = [signal.lfilter_zi(self.b_band, self.a_band) for _ in range(self.num_channels)]
            # --- ▲▲▲ ---
            
            filename = f"eeg_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            try:
                self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_header_written = False
                print(f"Opened CSV file for logging: {filename}")
                self.info_label.setText(f"Trigger: - | File: {filename}")
            except Exception as e:
                print(f"Error opening CSV file: {e}")
                self.status_label.setText("Status: Error creating log file!")
                return
            
            self.status_label.setText("Status: Sending start command...")
            self.first_packet_received = False
            self.first_packet_timer.start(3000) # 3-second timeout

            self.ble_thread.start_streaming()
            self.is_streaming = True
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)

    def stop_streaming(self):
        if self.is_streaming:
            if self.ble_thread.isRunning():
                self.ble_thread.stop_streaming()
            
            self.is_streaming = False
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.status_label.setText("Status: Connected. Stream stopped.")

            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                print("CSV file closed.")
                self.info_label.setText("Trigger: - | File: - (closed)")

    def on_connection_status(self, status: str):
        self.status_label.setText(f"Status: {status}")
        is_connected = "Connected" in status or "Ready" in status or "Streaming" in status
        self.start_btn.setEnabled(is_connected)
        self.stop_btn.setEnabled(self.is_streaming and is_connected)
        if "Disconnected" in status or "Error" in status or "not found" in status or "closed" in status:
            if self.is_streaming:
                self.stop_streaming()
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.connect_btn.setText("Connect")


    def on_config_received(self, config: dict):
        self.first_packet_timer.stop()
        if not self.first_packet_received:
            self.first_packet_received = True
            self.status_label.setText("Status: Streaming...")

        self.num_channels = config.get('num_channels', 8)
        electrodes = config.get('electrodes', [])
        print(f"Applying new configuration. Channels: {self.num_channels}")

        if self.csv_writer and not self.csv_header_written:
            ch_names = [e['name'] if e['name'] else f'CH{i+1}' for i, e in enumerate(electrodes)]
            header_ch_names = [f"{name} (uV)" for name in ch_names[:self.num_channels]]
            header = ['timestamp', 'sample_index'] + header_ch_names + ['trigger']
            self.csv_writer.writerow(header)
            self.csv_header_written = True
            print(f"CSV header written: {header}")

        for i in range(len(self.plots)):
            if i < len(electrodes) and i < self.num_channels:
                label_name = electrodes[i]['name'] if electrodes[i]['name'] else f'CH{i+1}'
                self.plots[i].setLabel('left', label_name, units='μV')
                self.plots[i].setVisible(True)
            else:
                self.plots[i].setVisible(False)
    
    def on_data_received(self, samples_list: list):
        """Process a list of incoming samples with cascaded filters."""
        if not self.first_packet_received:
            self.first_packet_timer.stop()
            self.first_packet_received = True
            self.status_label.setText("Status: Streaming...")
        
        if not samples_list:
            return

        # --- ▼▼▼ フィルタリング処理 ▼▼▼ ---
        # signals_by_channel の形状: (チャンネル数, 受信サンプル数)
        signals_by_channel = np.array([s['signals'] for s in samples_list]).T
        final_filtered_signals = np.zeros_like(signals_by_channel, dtype=float)

        # 各チャンネルにフィルタをカスケード適用
        for i in range(self.num_channels):
            # Stage 1: 50Hzノッチフィルタ
            notched_chunk, self.notch_filter_states[i] = signal.lfilter(
                self.b_notch, self.a_notch, signals_by_channel[i, :], zi=self.notch_filter_states[i]
            )
            
            # Stage 2: 0.5-30Hzバンドパスフィルタ
            bandpassed_chunk, self.bandpass_filter_states[i] = signal.lfilter(
                self.b_band, self.a_band, notched_chunk, zi=self.bandpass_filter_states[i]
            )
            
            final_filtered_signals[i, :] = bandpassed_chunk

        # CSV保存とプロットのために形状を戻す: (受信サンプル数, チャンネル数)
        filtered_samples = final_filtered_signals.T
        # --- ▲▲▲ フィルタリング処理ここまで ▲▲▲ ---

        last_trigger = 0
        
        # CSV書き込み処理 (フィルタ適用後のデータを使用)
        if self.csv_writer and self.csv_header_written:
            for idx, sample in enumerate(samples_list):
                timestamp = datetime.now().isoformat()
                signals_uV = [val * LSB_TO_MICROVOLTS for val in filtered_samples[idx][:self.num_channels]]
                row = [timestamp, sample['sample_index']] + signals_uV + [sample['trigger']]
                self.csv_writer.writerow(row)

        # プロット用バッファの更新 (フィルタ適用後のデータを使用)
        for idx, sample in enumerate(samples_list):
            self.time_buffer.append(self.sample_count)
            self.sample_count += 1
            for i in range(self.num_channels):
                value_uV = filtered_samples[idx, i] * LSB_TO_MICROVOLTS
                self.data_buffers[i].append(value_uV)
            last_trigger = sample['trigger']
        
        # UI更新
        trigger_bits = format(last_trigger, '04b')
        current_file_text = self.info_label.text().split("|")[-1].strip()
        self.info_label.setText(f"Trigger: {trigger_bits} | {current_file_text}")
        
    def update_plots(self):
        """Update graphs with new data from buffers."""
        if not self.time_buffer:
            return

        time_array = np.array(self.time_buffer)
        for i in range(self.num_channels):
            if self.data_buffers[i]:
                self.curves[i].setData(x=time_array, y=np.array(self.data_buffers[i]))

    def check_first_packet_timeout(self):
        """Called by QTimer if no packets are received after starting stream."""
        if self.is_streaming and not self.first_packet_received:
            self.status_label.setText("Status: No response from device. Check firmware/hardware.")

    def closeEvent(self, event):
        """Handle window close event."""
        if self.is_streaming:
            self.stop_streaming()
        if self.ble_thread.isRunning():
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

