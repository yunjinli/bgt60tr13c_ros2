#!/usr/bin/env python3
import math
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from bgt60tr13c_msgs.msg import RawFrame

# Optional: OpenCV for fast grayscale display
import cv2


def normalize_to_uint8(img: np.ndarray) -> np.ndarray:
    """
    Robust per-frame normalization to [0,255] for display.
    Handles constant/NaN frames gracefully.
    """
    finite = np.isfinite(img)
    if not finite.any():
        return np.zeros_like(img, dtype=np.uint8)

    vals = img[finite]
    vmin = np.percentile(vals, 1.0)   # clip a bit of outliers
    vmax = np.percentile(vals, 99.0)
    if vmax <= vmin:
        vmax = vmin + 1e-6

    img = np.clip(img, vmin, vmax)
    img = (img - vmin) / (vmax - vmin) * 255.0
    return img.astype(np.uint8)


class RawFrameViz(Node):
    def __init__(self):
        super().__init__('raw_frame_viz')

        # Parameters
        self.declare_parameter('topic', '/ifx_bgt_device/bgt60tr13c/raw_frame')
        self.declare_parameter('scale', 3)           # UI upscaling (integer)
        self.declare_parameter('single_window', True) # grid in one window vs. per-channel windows

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.scale = int(self.get_parameter('scale').value)
        self.single_window = bool(self.get_parameter('single_window').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(RawFrame, topic, self.cb, qos)
        self.get_logger().info(f'Subscribing to: {topic}')

        self.window_name = 'BGT60TR13C RawFrame (per-channel)'
        if self.single_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Handle Ctrl+C cleanly (especially for cv2 windows)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, *_):
        self.get_logger().info('Shutting down…')
        cv2.destroyAllWindows()
        rclpy.shutdown()

    def cb(self, msg: RawFrame):
        # Reconstruct shape: (num_rx, num_samples_per_chirp, num_chirps_per_frame)
        nrx, ns, nc = int(msg.num_rx), int(msg.num_samples_per_chirp), int(msg.num_chirps_per_frame)
        try:
            arr = np.frombuffer(np.array(msg.data, dtype=np.float32), dtype=np.float32)
            arr = arr.reshape((nrx, ns, nc))
        except Exception as e:
            self.get_logger().error(f'Bad frame shape: rx={nrx}, ns={ns}, nc={nc}, len(data)={len(msg.data)} ({e})')
            return

        # Build images per channel
        imgs = []
        for i in range(nrx):
            ch = arr[i, :, :]  # shape (ns, nc)

            # If your data is real already, this is fine; if it's magnitude, even better.
            # If you were publishing real/imag elsewhere, adapt here.
            img_u8 = normalize_to_uint8(ch)

            # OpenCV shows (rows, cols) -> (ns, nc). Optionally flip vertical:
            # img_u8 = np.flipud(img_u8)

            # Upscale for visibility
            if self.scale > 1:
                img_u8 = cv2.resize(img_u8, (nc * self.scale, ns * self.scale), interpolation=cv2.INTER_NEAREST)

            imgs.append(img_u8)

        # Display
        if self.single_window:
            grid = self._tile_images(imgs)
            cv2.imshow(self.window_name, grid)
        else:
            for i, im in enumerate(imgs):
                cv2.imshow(f'raw_frame_channel_{i}', im)

        cv2.waitKey(1)  # keeps UI responsive

    @staticmethod
    def _tile_images(images):
        """
        Tile a list of grayscale images (same HxW) into a roughly square grid.
        """
        if not images:
            return None
        h, w = images[0].shape[:2]
        cols = math.ceil(math.sqrt(len(images)))
        rows = math.ceil(len(images) / cols)

        # pad last row if needed
        padded = images + [np.zeros((h, w), dtype=np.uint8)] * (rows * cols - len(images))
        rows_list = []
        for r in range(rows):
            row = np.hstack(padded[r * cols:(r + 1) * cols])
            rows_list.append(row)
        grid = np.vstack(rows_list)
        return grid


def main():
    rclpy.init()
    node = RawFrameViz()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
