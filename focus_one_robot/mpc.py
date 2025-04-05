import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy as np
import cvxopt

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # Create Subscribers for "velocity" and "angles"
        self.velocity_subscriber = self.create_subscription(
            Float64MultiArray,
            'velocity',
            self.velocity_callback,
            10
        )

        self.angles_subscriber = self.create_subscription(
            Float64MultiArray,
            'angles',
            self.angles_callback,
            10
        )

        # Create Publishers for each joint (Joint_1 to Joint_6)
        self.joint_1_pub = self.create_publisher(Float64, 'joint_1', 10)
        self.joint_2_pub = self.create_publisher(Float64, 'joint_2', 10)
        self.joint_3_pub = self.create_publisher(Float64, 'joint_3', 10)
        self.joint_4_pub = self.create_publisher(Float64, 'joint_4', 10)
        self.joint_5_pub = self.create_publisher(Float64, 'joint_5', 10)
        self.joint_6_pub = self.create_publisher(Float64, 'joint_6', 10)

        # Initialize state and control variables
        self.velocity = np.zeros(6)
        self.angles = np.zeros(6)

    def velocity_callback(self, msg):
        # Update velocity (joint velocities)
        self.velocity = np.array(msg.data)

    def angles_callback(self, msg):
        # Update current joint angles (including prismatic joint)
        self.angles = np.array(msg.data)

    def solve_mpc(self):
        # Define MPC parameters
        N = 10  # Horizon length (number of steps for prediction)

        # Cost matrices (quadratic cost function)
        Q = np.eye(6)  # State cost matrix
        R = np.eye(6)  # Control cost matrix

        # Initialize optimization variables
        H = np.block([[Q, np.zeros((6, 6))], [np.zeros((6, 6)), R]])
        f = np.zeros(12)

        # Define equality constraints (desired joint positions and velocities)
        A = np.zeros((12, 12))  # Simple constraint matrix for now
        b = np.zeros(12)        # Constraints bounds

        # Set up the quadratic program using cvxopt
        P = cvxopt.matrix(H)
        q = cvxopt.matrix(f)

        # Correctly create the G matrix with the right dimensions (12x12)
        # G is the matrix that contains inequality constraints
        G = np.block([
            [np.eye(6), np.zeros((6, 6))],
            [np.zeros((6, 6)), -np.eye(6)]
        ])
        G = cvxopt.matrix(G)  # Convert to cvxopt.matrix
        h = np.ones(12)  # Set the upper and lower bounds for the control inputs
        h = cvxopt.matrix(h)  # Convert to cvxopt.matrix

        # Solve the quadratic problem with a manual KKT solver (custom solver)
        options = {'kktsolver': 'ldl'}
        solution = cvxopt.solvers.qp(P, q, G, h, options=options)

        # Extract the optimal joint angles and prismatic position from the solution
        control_inputs = np.array(solution['x']).flatten()
        joint_angles = control_inputs[:5]  # 5 angular joints
        prismatic_position = control_inputs[5]  # 1 prismatic joint

        return joint_angles, prismatic_position

    def control_loop(self):
        # Solve MPC to compute the next control input (joint angles and prismatic position)
        joint_angles, prismatic_position = self.solve_mpc()

        # Publish the computed joint positions (angular and prismatic)
        self.joint_1_pub.publish(Float64(data=joint_angles[0]))
        self.joint_2_pub.publish(Float64(data=joint_angles[1]))
        self.joint_3_pub.publish(Float64(data=joint_angles[2]))
        self.joint_4_pub.publish(Float64(data=joint_angles[3]))
        self.joint_5_pub.publish(Float64(data=joint_angles[4]))
        self.joint_6_pub.publish(Float64(data=prismatic_position))

    def timer_callback(self):
        # Run the control loop every 0.1 seconds
        self.control_loop()

def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPCController()

    # Create a timer to run the MPC control loop every 0.1 seconds
    mpc_node.create_timer(0.1, mpc_node.timer_callback)

    rclpy.spin(mpc_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
