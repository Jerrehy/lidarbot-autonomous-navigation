#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self.publish_diag)

    def publish_diag(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # CPU
        cpu_status = DiagnosticStatus(name='system:cpu', level=DiagnosticStatus.OK, message='OK')
        cpu_status.values = [KeyValue(key='cpu_percent', value=f"{psutil.cpu_percent()}%"),
                             KeyValue(key='ram_percent', value=f"{psutil.virtual_memory().percent}%")]
        msg.status.append(cpu_status)
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(SystemMonitor())
    rclpy.shutdown()
