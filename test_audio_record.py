#!/usr/bin/env python3
"""
Test script: Record audio from /audio/stream to WAV file
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import wave
import numpy as np
import sys


class AudioRecorder(Node):
    def __init__(self, filename, duration):
        super().__init__('audio_recorder')
        
        self.filename = filename
        self.duration = duration
        self.sample_rate = 16000
        self.channels = 2
        
        self.audio_data = []
        self.start_time = self.get_clock().now()
        
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/audio/stream',
            self.audio_callback,
            10
        )
        
        self.get_logger().info(f'Recording {duration}s to {filename}...')
    
    def audio_callback(self, msg):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if elapsed >= self.duration:
            self.save_audio()
            rclpy.shutdown()
            return
        
        # Convert uint8 list to bytes, then to int16 array
        audio_bytes = bytes(msg.data)
        audio_chunk = np.frombuffer(audio_bytes, dtype=np.int16)
        
        self.audio_data.append(audio_chunk)
    
    def save_audio(self):
        if not self.audio_data:
            self.get_logger().warn('No audio data')
            return
        
        audio_array = np.concatenate(self.audio_data)
        audio_array = audio_array.reshape(-1, self.channels)
        
        with wave.open(self.filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_array.tobytes())
        
        self.get_logger().info(f'Saved {len(audio_array)} samples to {self.filename}')


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 test_audio_record.py <file.wav> <seconds>")
        sys.exit(1)
    
    filename = sys.argv[1]
    duration = float(sys.argv[2])
    
    rclpy.init()
    
    try:
        node = AudioRecorder(filename, duration)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
