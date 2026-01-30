# Radar Configuration

- [radar.yaml](./radar.yaml) (default): Shape (3, 128, 128), the configuration I did the surface detection back in PSS. It's a definitely working configuration.

- [radar_medium.yaml](./radar_medium.yaml) (default): Shape (3, 64, 64), the configuration I did the surface detection back in PSS. The performance is slightly worse than 128 x 128.
- [radar_small.yaml](./radar_small.yaml) (default): Shape (3, 32, 32), the configuration we did at RTF when collecting data from the model car with a covered box. This is not an optimal configuration, either go for [radar.yaml](./radar.yaml) (safest) or [radar_medium.yaml](./radar_medium.yaml).
