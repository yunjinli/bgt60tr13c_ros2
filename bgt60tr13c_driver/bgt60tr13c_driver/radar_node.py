#!/usr/bin/env python3
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
)

from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from bgt60tr13c_msgs.msg import DeviceConfig, RawFrame

# --- Infineon Avian SDK ---
from ifxAvian import Avian

import signal

def open_avian_with_retries(max_tries=100, delay_s=0.3, logger=None):
    last_err = None
    for i in range(1, max_tries+1):
        try:
            dev = Avian.Device()
            return dev
        except Exception as e:
            last_err = e
            if logger:
                logger.warn(f'Avian open attempt {i}/{max_tries} failed: {e}')
            time.sleep(delay_s)
    raise RuntimeError(f'Failed to open Avian device after {max_tries} tries: {last_err}')


def make_device_config():
    # Mirror your example config
    return Avian.DeviceConfig(
        sample_rate_Hz = 1_000_000,       # 1 MHz
        rx_mask = 5,                      # RX1 + RX3 (bitmask)
        tx_mask = 1,                      # TX1
        if_gain_dB = 33,
        tx_power_level = 31,
        start_frequency_Hz = 60e9,
        end_frequency_Hz = 61.5e9,
        num_chirps_per_frame = 128,
        num_samples_per_chirp = 64,
        chirp_repetition_time_s = 0.0005,
        frame_repetition_time_s = 0.15,
        mimo_mode = 'off'
    )

def rx_count_from_mask(mask: int) -> int:
    # popcount
    c = 0
    for i in range(32):
        if mask & (1 << i):
            c += 1
    return c

class BGT60TR13CNode(Node):
    def __init__(self):
        super().__init__('bgt60tr13c_node')
        
        # ---- Parameters ----
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sample_rate_Hz', 1_000_000),
                ('rx_mask', 5),
                ('tx_mask', 1),
                ('if_gain_dB', 33),
                ('tx_power_level', 31),
                ('start_frequency_Hz', 60e9),
                ('end_frequency_Hz', 61.5e9),
                ('num_chirps_per_frame', 128),
                ('num_samples_per_chirp', 64),
                ('chirp_repetition_time_s', 0.0005),
                ('frame_repetition_time_s', 0.15),
                ('mimo_mode', 'off'),
                # driver parameters
                ('publish_magnitude', True),
                ('topic_prefix', '/ifx_bgt_device/bgt60tr13c'),
                ('qos_depth', 5),
            ]
        )
        p = self.get_parameter
        self.topic_prefix = p('topic_prefix').get_parameter_value().string_value
        qos_depth = p('qos_depth').get_parameter_value().integer_value
        
        # Raw data QoS (best effort for throughput)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=max(1, qos_depth)
        )

        # CONFIG QoS (latched)
        config_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_cfg = self.create_publisher(
            DeviceConfig, f'{self.topic_prefix}/device_config', config_qos
        )
        self.pub_raw = self.create_publisher(
            RawFrame, f'{self.topic_prefix}/raw_frame', sensor_qos
        )

        # Build Avian config from params
        self.cfg = Avian.DeviceConfig(
            sample_rate_Hz = int(p('sample_rate_Hz').value),
            rx_mask = int(p('rx_mask').value),
            tx_mask = int(p('tx_mask').value),
            if_gain_dB = int(p('if_gain_dB').value),
            tx_power_level = int(p('tx_power_level').value),
            start_frequency_Hz = float(p('start_frequency_Hz').value),
            end_frequency_Hz = float(p('end_frequency_Hz').value),
            num_chirps_per_frame = int(p('num_chirps_per_frame').value),
            num_samples_per_chirp = int(p('num_samples_per_chirp').value),
            chirp_repetition_time_s = float(p('chirp_repetition_time_s').value),
            frame_repetition_time_s = float(p('frame_repetition_time_s').value),
            mimo_mode = str(p('mimo_mode').value),
        )
        self.publish_magnitude = bool(p('publish_magnitude').value)
        self.declare_parameter('republish_config_period_s', 0.0)
        period = float(self.get_parameter('republish_config_period_s').value)
        self._cfg_timer = None
        if period > 0.0:
            self._cfg_timer = self.create_timer(period, self.publish_config_once)
        
        # Open device + set config
        self.get_logger().info('Opening Avian device...')
        self.dev = open_avian_with_retries(logger=self.get_logger())

        # Apply config
        self.dev.set_config(self.cfg)

        # Optional: short settle + throw away the first frame (often junk / stale)
        time.sleep(0.2)

        metrics = self.dev.metrics_from_config(self.cfg)
        self.get_logger().info(f'Metrics: {metrics}')
        self.publish_config_once()  # now publishes DeviceConfig msg

        self._closed = False
        # Ensure shutdown gets called for Ctrl+C / SIGTERM as well
        signal.signal(signal.SIGINT,  lambda *_: self._sig_shutdown())
        signal.signal(signal.SIGTERM, lambda *_: self._sig_shutdown())
        
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self.capture_loop, daemon=True)
        self._thread.start()
        
        
        
    def publish_config_once(self):
        msg = DeviceConfig()
        msg.sample_rate_hz = self.cfg.sample_rate_Hz
        msg.rx_mask = self.cfg.rx_mask
        msg.tx_mask = self.cfg.tx_mask
        msg.if_gain_db = self.cfg.if_gain_dB
        msg.tx_power_level = self.cfg.tx_power_level
        msg.start_frequency_hz = float(self.cfg.start_frequency_Hz)
        msg.end_frequency_hz = float(self.cfg.end_frequency_Hz)
        msg.num_chirps_per_frame = self.cfg.num_chirps_per_frame
        msg.num_samples_per_chirp = self.cfg.num_samples_per_chirp
        msg.chirp_repetition_time_s = float(self.cfg.chirp_repetition_time_s)
        msg.frame_repetition_time_s = float(self.cfg.frame_repetition_time_s)
        msg.mimo_mode = str(self.cfg.mimo_mode)
        self.pub_cfg.publish(msg)
        self.get_logger().info('Published device_config')

    def publish_config_once(self):
        msg = DeviceConfig()
        msg.sample_rate_hz = self.cfg.sample_rate_Hz
        msg.rx_mask = self.cfg.rx_mask
        msg.tx_mask = self.cfg.tx_mask
        msg.if_gain_db = self.cfg.if_gain_dB
        msg.tx_power_level = self.cfg.tx_power_level
        msg.start_frequency_hz = float(self.cfg.start_frequency_Hz)
        msg.end_frequency_hz = float(self.cfg.end_frequency_Hz)
        msg.num_chirps_per_frame = self.cfg.num_chirps_per_frame
        msg.num_samples_per_chirp = self.cfg.num_samples_per_chirp
        msg.chirp_repetition_time_s = float(self.cfg.chirp_repetition_time_s)
        msg.frame_repetition_time_s = float(self.cfg.frame_repetition_time_s)
        msg.mimo_mode = str(self.cfg.mimo_mode)
        self.pub_cfg.publish(msg)

    def capture_loop(self):
        num_rx = rx_count_from_mask(self.cfg.rx_mask)
        ns = self.cfg.num_samples_per_chirp
        nc = self.cfg.num_chirps_per_frame

        while not self._stop.is_set():
            try:
                frame = self.dev.get_next_frame()
                if self.publish_magnitude:
                    data = np.abs(frame).astype(np.float32)
                else:
                    data = np.real(frame).astype(np.float32)

                raw = RawFrame()
                raw.stamp = self.get_clock().now().to_msg()
                raw.num_rx = num_rx
                raw.num_samples_per_chirp = ns
                raw.num_chirps_per_frame = nc
                raw.data = data.ravel().tolist()
                self.pub_raw.publish(raw)
            except Exception as e:
                self.get_logger().error(f'Capture error: {e}')
                time.sleep(0.01)

    def _sig_shutdown(self):
        # Called from signal handler (non-ROS thread)
        try:
            self.on_shutdown()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def on_shutdown(self):
        if self._closed:
            return
        self._closed = True

        self.get_logger().info('Shutting down node…')
        try:
            if hasattr(self, '_stop'):
                self._stop.set()
        except Exception:
            pass
        try:
            if hasattr(self, '_thread') and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass

        # Try to stop/close device gracefully
        try:
            # If the SDK has an explicit stop, call it (uncomment if available)
            # self.dev.stop_acquisition()
            pass
        except Exception:
            pass
        try:
            if hasattr(self, 'dev') and self.dev is not None:
                # Prefer an explicit close if provided by the SDK
                if hasattr(self.dev, 'close'):
                    self.dev.close()
                # Fallback: delete reference
                self.dev = None
        except Exception:
            pass

        # Tiny delay helps some USB stacks fully release
        time.sleep(0.2)
        self.get_logger().info('Shutdown complete.')

def main():
    rclpy.init()
    node = BGT60TR13CNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
