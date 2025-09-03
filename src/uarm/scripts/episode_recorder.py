#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import pickle
import os
import datetime
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from threading import Lock
import signal
import sys

import tensorflow as tf
import cv2
import pyttsx3
from num2words import num2words
import re
import subprocess

exit_flag = False

def center_crop_and_resize(image, crop_h, crop_w, out_h=448, out_w=448):
    h, w = image.shape[:2]
    start_y = max((h - crop_h) // 2, 0)
    start_x = max((w - crop_w) // 2, 0)
    cropped = image[start_y:start_y+crop_h, start_x:start_x+crop_w]
    resized = cv2.resize(cropped, (out_w, out_h))
    return resized


def handle_shutdown(signum, frame):
    global exit_flag
    exit_flag = True
    print("\n[Shutdown] Ctrl+C pressed. Cleaning up...")
    # ROS2不需要显式的signal_shutdown

signal.signal(signal.SIGINT, handle_shutdown)

def rlds_signature(example: dict):
    def nested_spec(d):
        if isinstance(d, dict):
            return {k: nested_spec(v) for k, v in d.items()}
        return d
    return nested_spec(example)

def speak(text: str):
    try:
        def replace_numbers(match):
            return num2words(int(match.group()))
        text = re.sub(r'\b\d+\b', replace_numbers, text)
        subprocess.run(["espeak-ng", "-v", "en", "-s", "150", text])
    except Exception as e:
        print(f"Speech failed: {e}")

class Recorder(Node):
    def __init__(self):
        super().__init__('episode_recorder')
        self.bridge = CvBridge()
        self.lock = Lock()

        self.latest_image_1 = None  # /cam_1
        self.latest_image_2 = None  # /cam_2
        self.preview_image_1 = None
        self.preview_image_2 = None
        self.latest_state = None
        self.latest_action = None
        self.episode = []
        self.recording = False

        self.subscription1 = self.create_subscription(Image, '/cam_1', self.image1_callback, 10)
        self.subscription2 = self.create_subscription(Image, '/cam_2', self.image2_callback, 10)
        self.subscription3 = self.create_subscription(Float64MultiArray, '/robot_state', self.state_callback, 10)
        self.subscription4 = self.create_subscription(Float64MultiArray, '/robot_action', self.action_callback, 10)

    def image1_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # print(cv_image.size)
            cropped_resized = center_crop_and_resize(cv_image, crop_h=720, crop_w=720)  # you can adjust this
        except Exception as e:
            self.get_logger().error(f"Failed to convert cam_1 image: {e}")
            return
        with self.lock:
            self.latest_image_1 = cropped_resized
            self.preview_image_1 = cropped_resized.copy()
            self.try_record()

    def image2_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cropped_resized = center_crop_and_resize(cv_image, crop_h=720, crop_w=720)
        except Exception as e:
            self.get_logger().error(f"Failed to convert cam_2 image: {e}")
            return
        with self.lock:
            self.latest_image_2 = cropped_resized
            self.preview_image_2 = cropped_resized.copy()
            self.try_record()

    def state_callback(self, msg):
        with self.lock:
            self.latest_state = np.array(msg.data, dtype=np.float64)
            self.try_record()

    def action_callback(self, msg):
        with self.lock:
            self.latest_action = np.array(msg.data, dtype=np.float64)
            self.try_record()

    def try_record(self):
        if not self.recording:
            return

        if (self.latest_image_1 is not None and
            self.latest_image_2 is not None and
            self.latest_state is not None and
            self.latest_action is not None):

            ts = {
                "image_1": self.latest_image_1.copy(),
                "image_2": self.latest_image_2.copy(),
                "state": self.latest_state.copy(),
                "action": self.latest_action.copy()
            }
            self.episode.append(ts)
            self.get_logger().info(f"Recorded step {len(self.episode)}")
            self.latest_state = None
            self.latest_action = None

    def get_last_frames(self):
        with self.lock:
            img1 = self.preview_image_1.copy() if self.preview_image_1 is not None else None
            img2 = self.preview_image_2.copy() if self.preview_image_2 is not None else None
            return img1, img2

    def save_episode(self, episode_id: int):
        if not self.episode:
            self.get_logger().warn("No data recorded, skipping save.")
            return

        save_dir = os.path.expanduser("~/ros_save_data/put_stick_into_hole0731")
        os.makedirs(save_dir, exist_ok=True)

        tfrecord_path = os.path.join(save_dir, f"lerobot_episode_{episode_id}")
        try:
            rlds_episode = []
            for i, ts in enumerate(self.episode):
                img1_encoded = cv2.imencode('.jpg', ts["image_1"])[1].tobytes()
                img2_encoded = cv2.imencode('.jpg', ts["image_2"])[1].tobytes()

                rlds_episode.append({
                    "observation": {
                        "image_1": img1_encoded,
                        "image_2": img2_encoded,
                        "state": ts["state"]
                    },
                    "action": ts["action"],
                    "is_first": i == 0,
                    "is_last": i == len(self.episode) - 1,
                    "is_terminal": i == len(self.episode) - 1,
                })

            ds = tf.data.Dataset.from_generator(
                lambda: (step for step in rlds_episode),
                output_signature=rlds_signature({
                    "observation": {
                        "image_1": tf.TensorSpec(shape=(), dtype=tf.string),
                        "image_2": tf.TensorSpec(shape=(), dtype=tf.string),
                        "state": tf.TensorSpec(shape=(len(self.episode[0]['state']),), dtype=tf.float64),
                    },
                    "action": tf.TensorSpec(shape=(len(self.episode[0]['action']),), dtype=tf.float64),
                    "is_first": tf.TensorSpec(shape=(), dtype=tf.bool),
                    "is_last": tf.TensorSpec(shape=(), dtype=tf.bool),
                    "is_terminal": tf.TensorSpec(shape=(), dtype=tf.bool),
                })
            )

            tf.data.experimental.save(ds, tfrecord_path)
            self.get_logger().info(f"✅ RLDS TFRecord episode saved to {tfrecord_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save RLDS TFRecord: {e}")

        self.episode = []

def get_next_episode_id(save_dir: str) -> int:
    existing = [f for f in os.listdir(save_dir) if f.startswith("lerobot_episode_")]
    episode_ids = []
    for name in existing:
        try:
            num = int(name.split("_")[-1])
            episode_ids.append(num)
        except:
            continue
    return max(episode_ids, default=0) + 1

def main(args=None):
    global exit_flag
    rclpy.init(args=args)
    recorder = Recorder()

    save_dir = os.path.expanduser("~/ros_save_data/put_stick_into_hole0731")
    os.makedirs(save_dir, exist_ok=True)
    episode_id = get_next_episode_id(save_dir)

    cv2.namedWindow("Preview: cam_1", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Preview: cam_2", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Preview: cam_1", 640, 360)
    cv2.resizeWindow("Preview: cam_2", 640, 360)

    start_time = time.time()

    recorder.get_logger().info(f"🔴 Recording episode {episode_id}. Press Ctrl+C to stop and save.")
    recorder.recording = True

    try:
        while rclpy.ok() and not exit_flag:
            rclpy.spin_once(recorder, timeout_sec=0.01)
            frame1, frame2 = recorder.get_last_frames()
            if frame1 is not None:
                cv2.putText(frame1, "Recording...", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("Preview: cam_1", frame1)
            if frame2 is not None:
                cv2.putText(frame2, "Recording...", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("Preview: cam_2", frame2)
            cv2.waitKey(10)
    except KeyboardInterrupt:
        pass

    recorder.recording = False
    recorder.get_logger().info("⏹️ Stopped recording. Saving episode...")
    recorder.save_episode(episode_id)
    # ✅ 录制后打印花费时间
    duration = time.time() - start_time
    recorder.get_logger().info(f"🕒 Episode {episode_id} recorded in {duration:.2f} seconds.")
    recorder.get_logger().info("✅ Episode saved. Exiting.")

    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
