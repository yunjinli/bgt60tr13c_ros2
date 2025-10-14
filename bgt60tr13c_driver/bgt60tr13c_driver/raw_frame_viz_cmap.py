#!/usr/bin/env python3
import math
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from bgt60tr13c_msgs.msg import RawFrame
import cv2


COLORMAPS = {
    "turbo": cv2.COLORMAP_TURBO,
    "viridis": cv2.COLORMAP_VIRIDIS,
    "magma": cv2.COLORMAP_MAGMA,
    "plasma": cv2.COLORMAP_PLASMA,
    "inferno": cv2.COLORMAP_INFERNO,
    "jet": cv2.COLORMAP_JET,
    "gray": None,  # special-case: no color map, grayscale
}


def to_uint8_linear(img: np.ndarray, vmin: float, vmax: float) -> np.uint8:
    """Map img linearly to [0,255] using vmin/vmax (no percentile tricks)."""
    if not np.isfinite(vmin) or not np.isfinite(vmax) or vmax <= vmin:
        # robust fallback
        vmin = np.nanmin(img)
        vmax = np.nanmax(img) if np.isfinite(np.nanmax(img)) else (vmin + 1.0)
        if vmax <= vmin:
            vmax = vmin + 1.0
    img = np.clip(img, vmin, vmax)
    img = (img - vmin) / (vmax - vmin) * 255.0
    return img.astype(np.uint8)


class RawFrameVizCmap(Node):
    def __init__(self):
        super().__init__('raw_frame_viz_cmap')

        # ---- Parameters ----
        self.declare_parameters('', [
            ('topic', '/ifx_bgt_device/bgt60tr13c/raw_frame'),
            ('scale', 3),                         # integer upscaling factor
            ('single_window', True),              # tile all channels into one window
            ('mode', 'raw'),                      # 'raw' or 'normalized'
            ('colormap', 'turbo'),                # see COLORMAPS keys; 'gray' for grayscale
            ('clip_min', float('nan')),           # used in 'raw' mode; if NaN -> auto from frame min
            ('clip_max', float('nan')),           # used in 'raw' mode; if NaN -> auto from frame max
            ('normalize_low_pct', 1.0),           # used in 'normalized' mode
            ('normalize_high_pct', 99.0),         # used in 'normalized' mode
            ('flip_vertical', False),             # flip image vertically for preferred orientation
            ('text_overlay', True),               # draw vmin/vmax and channel idx
        ])

        p = self.get_parameter
        topic            = p('topic').get_parameter_value().string_value
        self.scale       = int(p('scale').value)
        self.single_win  = bool(p('single_window').value)
        self.mode        = str(p('mode').value).lower()
        self.cmap_name   = str(p('colormap').value).lower()
        self.clip_min    = float(p('clip_min').value)
        self.clip_max    = float(p('clip_max').value)
        self.low_pct     = float(p('normalize_low_pct').value)
        self.high_pct    = float(p('normalize_high_pct').value)
        self.flip_vert   = bool(p('flip_vertical').value)
        self.text_overlay = bool(p('text_overlay').value)

        if self.cmap_name not in COLORMAPS:
            self.get_logger().warn(f"Unknown colormap '{self.cmap_name}', falling back to 'turbo'")
            self.cmap_name = 'turbo'

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(RawFrame, topic, self.cb, qos)
        self.get_logger().info(f'Subscribing to: {topic} | mode={self.mode} | cmap={self.cmap_name}')

        self.window = 'RawFrame (per-channel)'
        if self.single_win:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)

        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

    def _sig(self, *_):
        cv2.destroyAllWindows()
        rclpy.shutdown()

    def cb(self, msg: RawFrame):
        nrx, ns, nc = int(msg.num_rx), int(msg.num_samples_per_chirp), int(msg.num_chirps_per_frame)
        try:
            arr = np.asarray(msg.data, dtype=np.float32).reshape((nrx, ns, nc))
        except Exception as e:
            self.get_logger().error(f'Bad frame shape: rx={nrx}, ns={ns}, nc={nc}, len={len(msg.data)} ({e})')
            return

        # per-frame vmin/vmax (raw mode) or percentiles (normalized mode)
        if self.mode == 'raw':
            vmin = self.clip_min if np.isfinite(self.clip_min) else np.nanmin(arr)
            vmax = self.clip_max if np.isfinite(self.clip_max) else np.nanmax(arr)
        else:  # normalized
            finite = arr[np.isfinite(arr)]
            if finite.size == 0:
                vmin, vmax = 0.0, 1.0
            else:
                vmin = np.percentile(finite, self.low_pct)
                vmax = np.percentile(finite, self.high_pct)
                if vmax <= vmin:
                    vmax = vmin + 1e-6

        # build images per channel
        tiles = []
        for ch_idx in range(nrx):
            img = arr[ch_idx, :, :]  # (ns, nc)
            if self.flip_vert:
                img = np.flipud(img)

            img8 = to_uint8_linear(img, vmin, vmax)

            # colorize (or grayscale)
            cmap = COLORMAPS[self.cmap_name]
            if cmap is None:
                vis = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)  # keep 3-ch for uniform stacking
            else:
                vis = cv2.applyColorMap(img8, cmap)

            # if self.text_overlay:
            #     cv2.putText(vis, f'ch={ch_idx} vmin={vmin:.3g} vmax={vmax:.3g}',
            #                 (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.1, (255, 255, 255), 1, cv2.LINE_AA)

            if self.scale > 1:
                vis = cv2.resize(vis, (vis.shape[1] * self.scale, vis.shape[0] * self.scale),
                                 interpolation=cv2.INTER_NEAREST)
            tiles.append(vis)

        disp = self._tile(tiles) if self.single_win else None
        if self.single_win:
            cv2.imshow(self.window, disp)
        else:
            for i, im in enumerate(tiles):
                cv2.imshow(f'raw_frame_ch_{i}', im)
        cv2.waitKey(1)

    @staticmethod
    def _tile(images):
        """Tile equally-sized images into a roughly square grid."""
        if not images:
            return None
        h, w = images[0].shape[:2]
        n = len(images)
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)
        blank = np.zeros_like(images[0])
        padded = images + [blank] * (rows * cols - n)
        row_imgs = [np.hstack(padded[r*cols:(r+1)*cols]) for r in range(rows)]
        return np.vstack(row_imgs)


def main():
    rclpy.init()
    node = RawFrameVizCmap()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
