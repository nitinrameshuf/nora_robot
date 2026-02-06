#!/usr/bin/env python3
# Copyright 2026 nanostation
# Licensed under the Apache License, Version 2.0

"""
Pico Bridge Node for Nora
Sends expression commands to Raspberry Pi Pico via USB serial
Controls NeoPixel LED animations
"""

import rclpy
from rclpy.node import Node
from nora_head.msg import ExpressionCommand
import serial
import serial.tools.list_ports
import time


class PicoBridgeNode(Node):
    def __init__(self):
        super().__init__('pico_bridge')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('auto_reconnect', True)
        
        # Get parameters
        self.device = self.get_parameter('device').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value
        
        # Serial connection
        self.serial_port = None
        self.connect_serial()
        
        # Subscribe to expression commands
        self.subscription = self.create_subscription(
            ExpressionCommand,
            '/expression/command',
            self.expression_callback,
            10
        )
        
        # Reconnection timer (if auto_reconnect enabled)
        if self.auto_reconnect:
            self.reconnect_timer = self.create_timer(5.0, self.check_connection)
        
        self.get_logger().info('Pico bridge initialized')
    
    def connect_serial(self):
        """Open serial connection to Pico"""
        try:
            self.serial_port = serial.Serial(
                port=self.device,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            # Wait for Pico to initialize
            time.sleep(2.0)
            
            # Clear any startup messages
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.get_logger().info(f'Connected to Pico on {self.device}')
            
            # Send initial neutral expression
            self.send_command('neutral', 0.5, 1.0)
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Pico: {e}')
            self.serial_port = None
    
    def check_connection(self):
        """Check and reconnect if disconnected"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn('Serial connection lost, attempting reconnect...')
            self.connect_serial()
    
    def expression_callback(self, msg):
        """Handle expression command messages"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn('Cannot send command: Pico not connected')
            return
        
        # Use defaults if values are -1
        brightness = msg.brightness if msg.brightness >= 0 else 1.0
        speed = msg.speed if msg.speed >= 0 else 1.0
        
        self.send_command(msg.expression, brightness, speed)
        
        self.get_logger().info(
            f'Sent expression: {msg.expression} '
            f'(brightness={brightness:.2f}, speed={speed:.2f}, priority={msg.priority})'
        )
    
    def send_command(self, expression, brightness=1.0, speed=1.0):
        """Send command to Pico via serial"""
        try:
            # Protocol: "EXPR:expression_name\n"
            # Pico firmware handles brightness/speed internally
            command = f"EXPR:{expression}\n"
            
            self.serial_port.write(command.encode('utf-8'))
            self.serial_port.flush()
            
            # Optional: Read response from Pico (if implemented)
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    self.get_logger().debug(f'Pico response: {response}')
        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial_port = None
        
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.serial_port and self.serial_port.is_open:
            # Send neutral expression before closing
            try:
                self.send_command('neutral', 0.3, 1.0)
                time.sleep(0.5)
            except:
                pass
            
            self.serial_port.close()
            self.get_logger().info('Pico serial connection closed')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PicoBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
