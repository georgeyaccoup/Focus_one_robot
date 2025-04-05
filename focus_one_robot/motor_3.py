import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String
import time

class Joint1Controller(Node):
    def __init__(self):
        super().__init__('joint_3_controller')

        # Publisher to send "done" feedback
        self.feedback_publisher = self.create_publisher(String, 'joint_3_feedback', 10)

        # Subscriber to receive joint_1 target angle
        self.angle_subscriber = self.create_subscription(
            Float64,
            'joint_1',
            self.angle_callback,
            10
        )

        self.get_logger().info("Joint 2 Controller Node Initialized")

    def angle_callback(self, msg):
        target_angle = msg.data  # Target angle for joint 1
        self.get_logger().info(f"Received target angle: {target_angle}")

        # Move the motor to the target angle (simulated, replace with real motor control)
        self.move_motor_to_angle(target_angle)

        # After moving, check if motor reached the desired angle
        if self.check_motor_position(target_angle):
            # Wait for 5 seconds before publishing feedback
            self.wait_and_publish_feedback()

    def move_motor_to_angle(self, target_angle):
        # Here you would interface with your motor controller
        # For example: motor_controller.set_position(target_angle)
        # Simulating motor movement with a sleep
        self.get_logger().info(f"Moving motor to {target_angle} degrees...")
        time.sleep(2)  # Simulating motor movement time

    def check_motor_position(self, target_angle):
        # Simulate a position check. Replace this with actual sensor feedback.
        # For real systems, you'd read from an encoder or motor sensor.
        current_angle = target_angle  # Assume it reaches the desired position
        self.get_logger().info(f"Motor reached position: {current_angle} degrees")

        # Check if the motor reached the target angle (allowing some tolerance)
        tolerance = 0.01  # Degree tolerance
        if abs(current_angle - target_angle) <= tolerance:
            return True
        return False

    def wait_and_publish_feedback(self):
        # Wait for 5 seconds before publishing "done"
        self.get_logger().info("Waiting for 5 seconds before publishing feedback...")
        time.sleep(5)

        # Publish "done" to indicate that the motor has reached the target position
        msg = String()
        msg.data = "done"
        self.feedback_publisher.publish(msg)
        self.get_logger().info("Published feedback: done")

def main(args=None):
    rclpy.init(args=args)
    node = Joint1Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

