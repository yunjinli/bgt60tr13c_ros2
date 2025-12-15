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
import onnxruntime as ort

def softmax(x):
    """Compute softmax values for each set of scores in x."""
    e_x = np.exp(x - np.max(x)) # subtract max for numerical stability
    return e_x / e_x.sum(axis=1, keepdims=True)

class RadarSurfaceDetectionOnnx(Node):
    def __init__(self):
        super().__init__('radar_surface_detection_onnx')

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
        self.label_map = {'grass': 0, 'asphalt': 1, 'dirt': 2, 'pavement': 3} ## Currently hardcoded, perhaps we have to change that in the future.
        self.idx_to_label = {v: k for k, v in self.label_map.items()}
        try:
            self.ort_session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
            self.get_logger().info(f"Loaded ONNX model from {model_path}")
            
            self.input_name = self.ort_session.get_inputs()[0].name
            self.output_name = self.ort_session.get_outputs()[0].name
        except Exception as e:
             self.get_logger().error(f"Failed to load ONNX model from {model_path}: {e}")
             self.ort_session = None
             
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
        
        input_tensor = arr[np.newaxis, ...].astype(np.float32)
        
        try:
            logits = self.ort_session.run([self.output_name], {self.input_name: input_tensor})[0]
            current_probs = softmax(logits)
            
            self.probs_buffer.append(current_probs[0])
            
            avg_probs = np.mean(np.stack(list(self.probs_buffer)), axis=0)

            predicted_idx = np.argmax(avg_probs)
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
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")

def main():
    rclpy.init()
    node = RadarSurfaceDetectionOnnx()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
