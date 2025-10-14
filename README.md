# BGT60TR13C ROS2

ROS 2 (Humble) workspace for Infineon BGT60TR13C:

- `bgt60tr13c_msgs`: ROS interfaces (messages)
- `bgt60tr13c_driver`: Python driver, YAML-config, and visualizers

## Prereqs

- Ubuntu 22.04 + ROS 2 Humble (sourced)
- Python deps:
  - Infineon Radar SDK 3.3.1 (`ifxAvian`): `pip install <ifxAvian.whl>`
  - OpenCV Python: installed via rosdep as `python3-opencv`

## Clone & Build

```bash
## Go to your ROS2 workspace
cd ~/ros_ws/src
git clone https://github.com/yunjinli/bgt60tr13c_ros2.git
cd ~/ros_ws
# Build
colcon build --symlink-install
source install/setup.bash
```

## Run

### Configuration

Edit bgt60tr13c_driver/config/radar.yaml or pass your file with the following format.

```yaml
## bgt60tr13c_driver/config/radar.yaml
bgt60tr13c_node:
  ros__parameters:
    # Infineon Avian DeviceConfig
    sample_rate_Hz: 2000000
    rx_mask: 7
    tx_mask: 1
    if_gain_dB: 33
    tx_power_level: 31
    start_frequency_Hz: 5.9e10
    end_frequency_Hz: 6.1e10
    num_chirps_per_frame: 32
    num_samples_per_chirp: 32
    chirp_repetition_time_s: 0.0005
    frame_repetition_time_s: 0.1
    mimo_mode: "off"

    # Driver options
    publish_magnitude: true # if false, publishes real part
    topic_prefix: "/ifx_bgt_device/bgt60tr13c"
    qos_depth: 5

    republish_config_period_s: 2.0
```

### Driver only (with packaged YAML)

```bash
ros2 launch bgt60tr13c_driver radar.launch.py params:=/path/to/my.yaml
```

### Visualize the raw radar frames

```bash
ros2 run bgt60tr13c_driver raw_frame_viz_cmap

```

### Default Topics

```bash
/ifx_bgt60_device/bgt60tr13c/device_config ## (latched)
/ifx_bgt60_device/bgt60tr13c/raw_frame
```
