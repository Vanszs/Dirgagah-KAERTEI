#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import RTCM

try:
    import serial
except ImportError:
    serial = None

class RTKSerialBridge(Node):
    """Read RTCM/NMEA bytes from a serial RTK receiver and forward to MAVROS.

    Parameters:
    - serial_port (string): default '/dev/ttyUSB0'
    - baud_rate (int): default 115200
    - chunk_size (int): bytes per publish (default 512)
    - frame_id (string): optional frame id
    """

    def __init__(self):
        super().__init__('rtk_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('chunk_size', 512)
        self.declare_parameter('frame_id', '')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Topic expected by MAVROS gps_rtk plugin
        self.rtcm_pub = self.create_publisher(RTCM, '/mavros/gps_rtk/send_rtcm', qos)

        if serial is None:
            self.get_logger().error('pyserial not available. Install python3-serial')
            return

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f'RTKSerialBridge connected: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.ser = None

        self.timer = self.create_timer(0.02, self.read_and_publish)  # ~50 Hz read

    def read_and_publish(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(self.chunk_size)
            if data:
                msg = RTCM()
                msg.data = list(data)
                self.rtcm_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'RTK read/publish error: {e}')

    def destroy_node(self):
        try:
            if getattr(self, 'ser', None):
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTKSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

