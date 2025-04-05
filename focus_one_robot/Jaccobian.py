import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import pinocchio as pin
from pinocchio import buildModelFromUrdf
from pinocchio.utils import zero

import numpy as np
import os

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'velocity', 10)

        # === Load URDF and Build Model ===
        urdf_path = '/home/george/new_ws/src/focus_one_robot/urdf/focus_one.urdf'  
        self.model = buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        self.q = np.zeros(self.model.nq)  # Joint positions
        self.v = np.zeros(self.model.nv)  # Joint velocities

        # Timer callback to simulate and publish velocity
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # === Fake desired end-effector twist ===
        v_desired = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.1])  # Linear + angular twist

        # Compute Jacobian at end-effector
        end_effector_frame = self.model.frames[-1].name
        frame_id = self.model.getFrameId(end_effector_frame)

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.computeJointJacobian(self.model, self.data, self.q, self.model.njoints - 1)
        J = pin.computeFrameJacobian(self.model, self.data, self.q, frame_id, pin.LOCAL)

        # Compute joint velocities via pseudoinverse
        dq = np.linalg.pinv(J) @ v_desired  # (nv,)

        # Split angular (first 5 joints) and prismatic (last joint)
        velocity_array = np.array([*dq[:5], dq[5]])  # [ω1, ω2, ..., v6]

        msg = Float64MultiArray()
        msg.data = velocity_array.tolist()
        self.publisher_.publish(msg)

        self.get_logger().info(f"Published velocity: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
