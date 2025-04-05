import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Manipulator(Node):
    def __init__(self):
        super().__init__("manipulator")
        self.publisher_ = self.create_publisher(String, "location", 10)
        self.subscriber_ = self.create_subscription(String, "feedback", self.sub_callback, 10)
        self.positions = ["initial", "home", "final"]
        self.i = 0
        self.get_logger().info("Manipulator initialized and ready.")
        self.publish_next_position()

    def publish_next_position(self):
        if self.i < len(self.positions):
            msg = String()
            msg.data = self.positions[self.i]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published location: {msg.data}")
        else:
            self.get_logger().info("All positions have been sent.")

    def sub_callback(self, msg):
        feedback = msg.data.strip().lower()
        self.get_logger().info(f"Received feedback: {feedback}")
        if feedback == "done":
            self.i += 1
            self.publish_next_position()

def main():
    rclpy.init()
    node = Manipulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
