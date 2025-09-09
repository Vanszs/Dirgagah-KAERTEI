import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.msg import State

class StateEcho(Node):
    def __init__(self):
        super().__init__('state_echo')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.create_subscription(State, '/mavros_node/state', self.cb, qos)

    def cb(self, msg):
        self.get_logger().info(f"Connected={msg.connected}, Armed={msg.armed}, Mode={msg.mode}")

def main():
    rclpy.init()
    node = StateEcho()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
