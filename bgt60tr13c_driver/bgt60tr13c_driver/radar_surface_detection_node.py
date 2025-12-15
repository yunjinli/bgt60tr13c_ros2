#!/usr/bin/env python3
import signal
import numpy as np
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from bgt60tr13c_msgs.msg import RawFrame
from std_msgs.msg import String  # Using String for easy JSON transport
from collections import deque

## Radar surface detection model
import torch

class SimpleCNNv2(torch.nn.Module):
    def __init__(self, input_channels, num_classes):
        super(SimpleCNNv2, self).__init__()
        self.conv1 = self.conv1 = torch.nn.Conv2d(input_channels, 16, kernel_size=3, stride=1, padding=1)
        self.conv2 = torch.nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1)
        self.conv3 = torch.nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        # self.conv4 = torch.nn.Conv2d(64, 128, kernel_size=3, stride=1, paddin>
        self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)

        self.fc = torch.nn.Linear(64, num_classes)
        self.relu = torch.nn.ReLU()

    def forward(self, x):
        x = self.pool(self.relu(self.conv1(x)))
        x = self.pool(self.relu(self.conv2(x)))
        x = self.pool(self.relu(self.conv3(x)))
        # x = self.pool(self.relu(self.conv4(x)))
        x = torch.mean(x, dim=[2, 3])  # Global average pooling
        x = self.fc(x)
        return x

class SimpleCNN(torch.nn.Module):
    def __init__(self, input_channels, num_classes):
        super(SimpleCNN, self).__init__()
        self.conv1 = torch.nn.Conv2d(input_channels, 16, kernel_size=3, stride=1, padding=1)
        self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.conv2 = torch.nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1)
        self.fc1 = torch.nn.Linear(32, 64)
        self.fc2 = torch.nn.Linear(64, 128)
        self.fc3 = torch.nn.Linear(128, num_classes)
        self.relu = torch.nn.ReLU()

    def forward(self, x):
        x = self.pool(self.relu(self.conv1(x)))
        x = self.pool(self.relu(self.conv2(x)))
        x = torch.mean(x, dim=[2, 3])  # Global average pooling
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.fc3(x)
        return x
    
class NormalizeMinMax:
    def __call__(self, sample):
        min_val = torch.min(sample)
        max_val = torch.max(sample)
        normalized_sample = (sample - min_val) / (max_val - min_val)
        return normalized_sample
     
class RadarSurfaceDetection(Node):
    def __init__(self):
        super().__init__('radar_surface_detection')

        # Parameters
        self.declare_parameter('input_topic', '/ifx_bgt_device/bgt60tr13c/raw_frame')
        self.declare_parameter('output_topic', '/surface_prediction')
        self.declare_parameter('model_path', 'best_model.pth')
        self.declare_parameter('moving_avg_window', 5) # Number of frames to smooth over
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.window_size = self.get_parameter('moving_avg_window').get_parameter_value().integer_value
        
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(RawFrame, input_topic, self.cb, qos_best_effort)
        self.pub = self.create_publisher(String, output_topic, qos_reliable)
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')
        
        ## Load pre-trained model
        self.label_map = {'grass': 0, 'asphalt': 1, 'dirt': 2, 'pavement': 3}
        self.idx_to_label = {v: k for k, v in self.label_map.items()}
        # self.model = SimpleCNN(input_channels=3, num_classes=len(self.label_map))
        self.model = SimpleCNNv2(input_channels=3, num_classes=len(self.label_map))
        try:
            self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
            self.get_logger().info(f"Loaded model from {model_path}")
        except FileNotFoundError:
             self.get_logger().error(f"Model file not found at {model_path}")
        self.model.eval()
        self.transform = NormalizeMinMax()
        self.probs_buffer = deque(maxlen=self.window_size)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, *_):
        self.get_logger().info('Shutting down…')
        rclpy.shutdown()

    def cb(self, msg: RawFrame):
        # Reconstruct shape: (num_rx, num_samples_per_chirp, num_chirps_per_frame)
        nrx, ns, nc = int(msg.num_rx), int(msg.num_samples_per_chirp), int(msg.num_chirps_per_frame)
        try:
            arr = np.frombuffer(np.array(msg.data, dtype=np.float32), dtype=np.float32)
            arr = arr.reshape((nrx, nc, ns))
        except Exception as e:
            self.get_logger().error(f'Bad frame shape: rx={nrx}, ns={ns}, nc={nc}, len(data)={len(msg.data)} ({e})')
            return
        
        # frame = torch.from_numpy(arr).unsqueeze(0)  # Shape: (1, num_rx, num_chirps, num_samples)
        frame_tensor = torch.from_numpy(arr).unsqueeze(0)  # Shape: (1, num_rx, num_chirps, num_samples)
        # Normalize
        # frame = self.transform(frame).float()
        # Inference
        # with torch.no_grad():
        #     output_string = ""
        #     outputs = self.model(frame)
        #     outputs = torch.nn.functional.softmax(outputs, dim=1)
        #     for k, v in self.label_map.items():
        #         output_string += f"{k}: {outputs[0][v]:.4f} "
        #     _, predicted = torch.max(outputs, 1)
        #     # self.get_logger().info(f"")
        #     predicted_label = list(self.label_map.keys())[list(self.label_map.values()).index(predicted.item())]
        #     # self.get_logger().info(f'Predicted surface: {predicted_label} (class {predicted.item()})')
        #     output_string += f" --> Predicted surface: {predicted_label} (class {predicted.item()})"
        #     self.get_logger().info(f"Model outputs: {output_string}")
        with torch.no_grad():
            logits = self.model(frame_tensor)
            current_probs = torch.nn.functional.softmax(logits, dim=1) # Shape (1, num_classes)
            
            self.probs_buffer.append(current_probs[0])
            
            avg_probs = torch.mean(torch.stack(list(self.probs_buffer)), dim=0)

            predicted_idx = torch.argmax(avg_probs).item()
            predicted_label = self.idx_to_label.get(predicted_idx, "unknown")
            confidence = avg_probs[predicted_idx].item()

            result_dict = {
                "predicted_label": predicted_label,
                "confidence": round(confidence, 4),
                "probabilities": {k: round(avg_probs[v].item(), 4) for k, v in self.label_map.items()}
            }
            
            msg_out = String()
            msg_out.data = json.dumps(result_dict)
            self.pub.publish(msg_out)

            output_string = " | ".join([f"{k}: {v:.2f}" for k, v in result_dict['probabilities'].items()])
            self.get_logger().info(f"[{predicted_label.upper()}] {output_string}")
            

def main():
    rclpy.init()
    node = RadarSurfaceDetection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
