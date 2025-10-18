#!/usr/bin/env python3
"""
ADS1299 BLE Data Receiver with Real-time Visualization
Communicates with the NUS-like BLE service defined in main.cpp.
- Handles two packet types: Device Configuration and Chunked Data.
- Provides controls to start/stop data streaming.
- Updates plot labels based on received configuration.
"""

import sys
import struct
import asyncio
from collections import deque
import numpy as np
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

# Display buffer size (time window)
BUFFER_SIZE = 750  # 250 SPS * 3 seconds

class BLEThread(QThread):
    """Handles BLE communication in a separate thread."""
    connection_status = pyqtSignal(str)
    config_received = pyqtSignal(dict)
    data_received = pyqtSignal(list)  # Emits a list of samples

    def __init__(self):
        super().__init__()
        self.running = False
        self.client = None
        self.rx_char = None
        self.loop = None

    def run(self):
        """Main thread entry point."""
        self.running = True
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ble_main())
        self.loop.close()

    async def ble_main(self):
        """Main BLE connection and data reception logic."""
        try:
            self.connection_status.emit(f"Scanning for '{DEVICE_NAME}'...")
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

            if not device:
                self.connection_status.emit(f"Device '{DEVICE_NAME}' not found.")
                return

            self.connection_status.emit(f"Connecting to {device.name} ({device.address})...")

            async with BleakClient(device.address) as client:
                self.client = client
                self.connection_status.emit(f"Connected to {device.name}.")

                # Find the RX characteristic for sending commands
                self.rx_char = client.services.get_characteristic(CHARACTERISTIC_UUID_RX)
                if not self.rx_char:
                    self.connection_status.emit("RX Characteristic not found!")
                    return

                await client.start_notify(CHARACTERISTIC_UUID_TX, self.notification_handler)
                self.connection_status.emit("Ready. Waiting for streaming command.")

                while self.running:
                    await asyncio.sleep(0.1)

                await client.stop_notify(CHARACTERISTIC_UUID_TX)
                self.connection_status.emit("Disconnected.")

        except Exception as e:
            self.connection_status.emit(f"Error: {e}")
        finally:
            self.client = None
            self.rx_char = None

    def notification_handler(self, sender, data: bytearray):
        """Callback for incoming BLE notifications."""
        if not data:
            return

        packet_type = data[0]

        if packet_type == PKT_TYPE_DEVICE_CFG:
            self.parse_config_packet(data)
        elif packet_type == PKT_TYPE_DATA_CHUNK:
            self.parse_data_chunk_packet(data)
        else:
            print(f"Warning: Received unknown packet type: 0x{packet_type:02X}")

    def parse_config_packet(self, data: bytearray):
        """Parses the DeviceConfigPacket (0xDD)."""
        if len(data) < 88: # Size of DeviceConfigPacket
            print(f"Warning: Config packet is too short ({len(data)} bytes)")
            return
        
        # struct DeviceConfigPacket format: <B B 6x (8s B B)*8
        # Unpack header
        _, num_channels = struct.unpack('<BB', data[0:2])
        
        config = {'num_channels': num_channels, 'electrodes': []}
        
        # Unpack electrode configs
        offset = 8 # Start of configs array
        for i in range(8):
            # Each ElectrodeConfig is 10 bytes: char[8], uint8_t, uint8_t
            chunk = data[offset + i*10 : offset + (i+1)*10]
            name_bytes, type_val, _ = struct.unpack('<8sBB', chunk)
            # Decode C-style null-terminated string
            name = name_bytes.partition(b'\0')[0].decode('utf-8', 'ignore')
            config['electrodes'].append({'name': name, 'type': type_val})
        
        print(f"Received device config: {num_channels} channels.")
        self.config_received.emit(config)

    def parse_data_chunk_packet(self, data: bytearray):
        """Parses the ChunkedSamplePacket (0x66)."""
        # Header: packet_type (1), start_index (2), num_samples (1)
        if len(data) < 4:
            print("Warning: Data chunk packet is too short for header.")
            return

        _, start_index, num_samples = struct.unpack('<B H B', data[0:4])
        
        # Each SampleData is 20 bytes: int16_t[8], uint8_t, uint8_t[3]
        expected_size = 4 + num_samples * 20
        if len(data) != expected_size:
            print(f"Warning: Data chunk size mismatch. Expected {expected_size}, got {len(data)}.")
            return

        samples_list = []
        offset = 4
        for _ in range(num_samples):
            sample_chunk = data[offset : offset + 20]
            # '<' for little-endian, '8h' for 8 int16, 'B' for trigger, '3x' for 3 reserved bytes
            unpacked_data = struct.unpack('<8hB3x', sample_chunk)
            
            sample = {
                'signals': list(unpacked_data[0:8]),
                'trigger': unpacked_data[8]
            }
            samples_list.append(sample)
            offset += 20
        
        self.data_received.emit(samples_list)

    def _send_command(self, command: bytes):
        if self.client and self.rx_char and self.client.is_connected:
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
        """Stops the thread."""
        self.running = False

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

    def setup_ui(self):
        """Set up all UI elements."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Top Control Panel ---
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        self.status_label = QLabel("Status: Not connected")
        self.info_label = QLabel("Trigger: -")
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

        # --- Plots ---
        self.plots = []
        self.curves = []
        pg.setConfigOptions(antialias=True)
        
        plot_grid = QGridLayout()
        main_layout.addLayout(plot_grid)

        for i in range(self.num_channels):
            plot_widget = pg.PlotWidget()
            plot_widget.setLabel('left', f'CH{i+1}', units='LSB')
            plot_widget.setLabel('bottom', 'Samples')
            plot_widget.setYRange(-32768, 32767)
            plot_widget.showGrid(x=True, y=True)
            curve = plot_widget.plot(pen=pg.mkPen(color=(0, 150, 255), width=1))
            
            self.plots.append(plot_widget)
            self.curves.append(curve)
            plot_grid.addWidget(plot_widget, i // 2, i % 2) # 2 columns of plots

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(33)  # ~30 FPS

    def toggle_connection(self):
        if not self.ble_thread.isRunning():
            self.connect_btn.setText("Disconnect")
            self.ble_thread.start()
        else:
            self.stop_streaming() # Ensure streaming is stopped before disconnect
            self.ble_thread.stop()
            self.ble_thread.wait()
            self.connect_btn.setText("Connect")
            self.on_connection_status("Disconnected by user.")

    def start_streaming(self):
        if self.ble_thread.isRunning() and not self.is_streaming:
            # Clear buffers on start
            for buf in self.data_buffers:
                buf.clear()
            self.time_buffer.clear()
            self.sample_count = 0
            
            self.ble_thread.start_streaming()
            self.is_streaming = True
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.status_label.setText("Status: Streaming...")

    def stop_streaming(self):
        if self.ble_thread.isRunning() and self.is_streaming:
            self.ble_thread.stop_streaming()
            self.is_streaming = False
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.status_label.setText("Status: Connected. Stream stopped.")

    def on_connection_status(self, status: str):
        self.status_label.setText(f"Status: {status}")
        is_connected = "Connected" in status or "Ready" in status or "Streaming" in status
        self.start_btn.setEnabled(is_connected)
        self.stop_btn.setEnabled(self.is_streaming and is_connected)
        if "Disconnected" in status or "Error" in status or "not found" in status:
            self.is_streaming = False
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.connect_btn.setText("Connect")


    def on_config_received(self, config: dict):
        self.num_channels = config.get('num_channels', 8)
        electrodes = config.get('electrodes', [])
        print(f"Applying new configuration. Channels: {self.num_channels}")
        for i in range(len(self.plots)):
            if i < len(electrodes) and i < self.num_channels:
                label_name = electrodes[i]['name'] if electrodes[i]['name'] else f'CH{i+1}'
                self.plots[i].setLabel('left', label_name, units='LSB')
                self.plots[i].setVisible(True)
            else:
                # Hide plots for unused channels
                self.plots[i].setVisible(False)
    
    def on_data_received(self, samples_list: list):
        """Process a list of incoming samples."""
        last_trigger = 0
        for sample in samples_list:
            self.time_buffer.append(self.sample_count)
            self.sample_count += 1
            for i in range(self.num_channels):
                self.data_buffers[i].append(sample['signals'][i])
            last_trigger = sample['trigger']
        
        trigger_bits = format(last_trigger, '04b')
        self.info_label.setText(f"Trigger: {trigger_bits} (0x{last_trigger:X})")
        
    def update_plots(self):
        """Update graphs with new data from buffers."""
        if not self.time_buffer:
            return

        time_array = np.array(self.time_buffer)
        for i in range(self.num_channels):
            if self.data_buffers[i]:
                self.curves[i].setData(x=time_array, y=np.array(self.data_buffers[i]))

    def closeEvent(self, event):
        """Handle window close event."""
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
