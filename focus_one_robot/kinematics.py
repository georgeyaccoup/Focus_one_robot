import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

class KinematicsNode(Node):
    def __init__(self):
        super().__init__("kinematics_node")
        self.subscription = self.create_subscription(String, "location", self.location_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, "angles", 10)
        self.get_logger().info("Kinematics Node started and waiting for location commands.")

    def location_callback(self, msg):
        location = msg.data.strip().lower()
        self.get_logger().info(f"Received location: {location}")

        angles = Float64MultiArray()

        if location == "initial":
            # Mocked IK results based on your matrix
            angles.data = [0.2, 0.5, -0.3, 0.6, -0.1, 0.25]  # [θ1, θ2, θ3, θ4, θ5, prismatic]
        elif location == "home":
            angles.data = [0.4, -0.6, 0.2, -0.1, 0.3, 0.32]
        elif location == "final":
            angles.data = [1.0, 0.1, -0.5, 0.4, -0.2, 0.3]
        else:
            self.get_logger().warn("Unknown location command received.")
            return

        self.publisher.publish(angles)
        self.get_logger().info(f"Published joint values: {angles.data}")

def main():
    rclpy.init()
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
