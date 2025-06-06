import rclpy
from rclpy.node import Node
from slambot_interfaces.msg import EncoderTicks

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")
        self.encoder_ticks_subscriber_ = self.create_subscription(EncoderTicks, "encoder_ticks", self.callback_calculate_pose, 10)
        self.get_logger().info("Running odometry publisher node")
    
    def callback_calculate_pose(self, msg: EncoderTicks):
        left = msg.left_encoder
        right = msg.right_encoder
        self.get_logger().info(f"Left: {left}, Right: {right}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()