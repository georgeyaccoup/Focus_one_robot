import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from functools import partial

class FeedbackMonitor(Node):
    def __init__(self):
        super().__init__('feedback_monitor')

        # Initialize feedback received flags for all joints
        self.feedback_received = {
            "joint_1": False,
            "joint_2": False,
            "joint_3": False,
            "joint_4": False,
            "joint_5": False,
            "joint_6": False,
        }

        # Publisher to send "Done" to the feedback topic
        self.feedback_publisher = self.create_publisher(String, 'feedback', 10)

        # Subscribe to each joint feedback topic with partial to pass joint name
        self.create_subscription(String, 'joint_1_feedback',
                                 partial(self.feedback_callback, joint_name='joint_1'), 10)
        self.create_subscription(String, 'joint_2_feedback',
                                 partial(self.feedback_callback, joint_name='joint_2'), 10)
        self.create_subscription(String, 'joint_3_feedback',
                                 partial(self.feedback_callback, joint_name='joint_3'), 10)
        self.create_subscription(String, 'joint_4_feedback',
                                 partial(self.feedback_callback, joint_name='joint_4'), 10)
        self.create_subscription(String, 'joint_5_feedback',
                                 partial(self.feedback_callback, joint_name='joint_5'), 10)
        self.create_subscription(String, 'joint_6_feedback',
                                 partial(self.feedback_callback, joint_name='joint_6'), 10)

        self.get_logger().info("✅ Feedback Monitor Node Initialized and Subscribed to All Joints")

    def feedback_callback(self, msg, joint_name):
        if msg.data.lower() == "done":
            self.feedback_received[joint_name] = True
            self.get_logger().info(f"✅ {joint_name} feedback received: done")

        # If all joints are done, publish "Done" once
        if all(self.feedback_received.values()):
            self.publish_final_feedback()

    def publish_final_feedback(self):
        final_msg = String()
        final_msg.data = "Done"
        self.feedback_publisher.publish(final_msg)
        self.get_logger().info("🎯 All joints completed. Published 'Done' to 'feedback' topic.")

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
