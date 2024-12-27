import rclpy
from rclpy.node import Node
from scipy.linalg import expm
from scipy.linalg import logm
from itertools import permutations
import time
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

import numpy as np
from youbot_kinematics.youbotKineStudent import YoubotKinematicStudent
from youbot_kinematics.target_data import TARGET_JOINT_POSITIONS


class YoubotTrajectoryPlanning(Node):
    def __init__(self):
        # Initialize node
        super().__init__('youbot_trajectory_planner')

        # Save question number for check in main run method
        self.kdl_youbot = YoubotKinematicStudent()

        # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
        self.traj_pub = self.create_publisher(JointTrajectory, '/EffortJointInterface_trajectory_controller/command',
                                        5)
        self.checkpoint_pub = self.create_publisher(Marker, "checkpoint_positions", 100)

    def run(self):
        """This function is the main run function of the class. When called, it runs question 6 by calling the q6()
        function to get the trajectory. Then, the message is filled out and published to the /command topic.
        """
        print("run q6a")
        self.get_logger().info("Waiting 5 seconds for everything to load up.")
        time.sleep(2.0)
        traj = self.q6()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.traj_pub.publish(traj)

    def q6(self):
        """ This is the main q6 function. Here, other methods are called to create the shortest path required for this
        question. Below, a general step-by-step is given as to how to solve the problem.
        Returns:
            traj (JointTrajectory): A list of JointTrajectory points giving the robot joint positions to achieve in a
            given time period.
        """
        # TODO: implement this
        # Steps to solving Q6.
        # 1. Load in targets from the bagfile (checkpoint data and target joint positions).
        # 2. Compute the shortest path achievable visiting each checkpoint Cartesian position.
        # 3. Determine intermediate checkpoints to achieve a linear path between each checkpoint and have a full list of
        #    checkpoints the robot must achieve. You can publish them to see if they look correct. Look at slides 39 in lecture 7
        # 4. Convert all the checkpoints into joint values using an inverse kinematics solver.
        # 5. Create a JointTrajectory message.

        # Your code starts here ------------------------------
        target_cart_tf, target_joint_positions = self.load_targets()
        self.get_logger().info(f"Loaded {target_cart_tf.shape[2]} checkpoints (including start).")

        sorted_order, min_dist = self.get_shortest_path(target_cart_tf)
        self.get_logger().info(
            f"Shortest path visiting these checkpoints in order: {sorted_order}. Dist={min_dist:.3f}"
        )

        num_points_between = 6
        full_checkpoint_tfs = self.intermediate_tfs(sorted_order, target_cart_tf, num_points_between)
        self.get_logger().info(f"Full # of poses after interpolation: {full_checkpoint_tfs.shape[2]}")

        self.publish_traj_tfs(full_checkpoint_tfs)
        init_joint_position = target_joint_positions[:, 0]  
        q_checkpoints = self.full_checkpoints_to_joints(full_checkpoint_tfs, init_joint_position)
        self.get_logger().info(f"Computed joint solutions for all {q_checkpoints.shape[1]} interpolated steps!")
        traj = JointTrajectory()
        time_per_step = 1.0  # 1s per checkpoint
        cur_t = 0.0
        for i in range(q_checkpoints.shape[1]):
            point = JointTrajectoryPoint()
            point.positions = q_checkpoints[:, i].tolist()
            cur_t += time_per_step
            point.time_from_start.sec = int(cur_t)
            point.time_from_start.nanosec = int((cur_t - int(cur_t))*1e9)
            traj.points.append(point)

        return traj
        # Your code ends here ------------------------------

        assert isinstance(traj, JointTrajectory)
        return traj

    def load_targets(self):
        """This function loads the checkpoint data from the TARGET_JOINT_POSITIONS variable. In this variable you will find each
        row has target joint positions. You need to use forward kinematics to get the goal end-effector position.
        Returns:
            target_cart_tf (4x4x5 np.ndarray): The target 4x4 homogenous transformations of the checkpoints found in the
            bag file. There are a total of 5 transforms (4 checkpoints + 1 initial starting cartesian position).
            target_joint_positions (5x5 np.ndarray): The target joint values for the 4 checkpoints + 1 initial starting
            position.
        """
        num_target_positions = len(TARGET_JOINT_POSITIONS)
        self.get_logger().info(f"{num_target_positions} target positions")
        # Initialize arrays for checkpoint transformations and joint positions
        target_joint_positions = np.zeros((5, num_target_positions + 1))
        # Create a 4x4 transformation matrix, then stack 6 of these matrices together for each checkpoint
        target_cart_tf = np.repeat(np.identity(4), num_target_positions + 1, axis=1).reshape((4, 4, num_target_positions + 1))

        # Get the current starting position of the robot
        target_joint_positions[:, 0] = self.kdl_youbot.current_joint_position
        # Initialize the first checkpoint as the current end effector position
        target_cart_tf[:, :, 0] = self.kdl_youbot.forward_kinematics(target_joint_positions[:, 0].tolist())

        # TODO: populate the transforms in the target_cart_tf object
        # populate the joint positions in the target_joint_positions object
        # Your code starts here ------------------------------
        for i in range(num_target_positions):
            # 第 i 个目标关节角
            q_target = TARGET_JOINT_POSITIONS[i, :]
            target_joint_positions[:, i+1] = q_target
            # Forward kinematics
            target_cart_tf[:, :, i+1] = self.kdl_youbot.forward_kinematics(
                q_target.tolist()
            )

        return target_cart_tf, target_joint_positions

        # Your code ends here ------------------------------

        self.get_logger().info(f"{target_cart_tf.shape} target poses")
        assert isinstance(target_cart_tf, np.ndarray)
        assert target_cart_tf.shape == (4, 4, num_target_positions + 1)
        assert isinstance(target_joint_positions, np.ndarray)
        assert target_joint_positions.shape == (5, num_target_positions + 1)

        return target_cart_tf, target_joint_positions
        

    def get_shortest_path(self, checkpoints_tf):
        """This function takes the checkpoint transformations and computes the order of checkpoints that results
        in the shortest overall path.
        Args:
            checkpoints_tf (np.ndarray): The target checkpoints transformations as a 4x4x5 numpy ndarray.
        Returns:
            sorted_order (np.array): An array of size 5 indicating the order of checkpoint
            min_dist:  (float): The associated distance to the sorted order giving the total estimate for travel
            distance.
        """
        num_checkpoints = checkpoints_tf.shape[2]
        # TODO: implement this method. Make it flexible to accomodate different numbers of targets.
        # Your code starts here ------------------------------
        # Sort the shortest distance and permutation here
        indices_to_visit = range(1, num_checkpoints)

        def dist(a, b):
            pa = a[0:3, 3]
            pb = b[0:3, 3]
            return np.linalg.norm(pa - pb)

        min_dist = float('inf')
        best_order = None

        for perm in permutations(indices_to_visit):
            cur_dist = 0.0
            prev_idx = 0
            for idx in perm:
                cur_dist += dist(checkpoints_tf[:, :, prev_idx], checkpoints_tf[:, :, idx])
                prev_idx = idx
            
            if cur_dist < min_dist:
                min_dist = cur_dist
                best_order = [0] + list(perm)

        sorted_order = np.array(best_order)
        return sorted_order, float(min_dist)

        # Your code ends here ------------------------------

        assert isinstance(sorted_order, np.ndarray)
        assert sorted_order.shape == (num_checkpoints,)
        assert isinstance(min_dist, float)

        return sorted_order, min_dist

    def publish_traj_tfs(self, tfs):
        """This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
        Cartesian path of the robot end-effector.
        Args:
            tfs (np.ndarray): A array of 4x4xn homogenous transformations specifying the end-effector trajectory.
        """
        id = 0
        for i in range(0, tfs.shape[2]):
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0 + id * 0.05
            marker.color.b = 1.0 - id * 0.05
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = tfs[0, -1, i]
            marker.pose.position.y = tfs[1, -1, i]
            marker.pose.position.z = tfs[2, -1, i]
            self.checkpoint_pub.publish(marker)

    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        """This function takes the target checkpoint transforms and the desired order based on the shortest path sorting, 
        and calls the decoupled_rot_and_trans() function.
        Args:
            sorted_checkpoint_idx (list): List describing order of checkpoints to follow.
            target_checkpoint_tfs (np.ndarray): the state of the robot joints. In a youbot those are revolute
            num_points (int): Number of intermediate points between checkpoints.
        Returns:
            full_checkpoint_tfs: 4x4x(4xnum_points + 5) homogeneous transformations matrices describing the full desired
            poses of the end-effector position.
        """
        # TODO: implement this
        # Your code starts here ------------------------------
        big_tf_list = []
        for i in range(len(sorted_checkpoint_idx) - 1):
            idxA = sorted_checkpoint_idx[i]
            idxB = sorted_checkpoint_idx[i+1]
            A_tf = target_checkpoint_tfs[:, :, idxA]
            B_tf = target_checkpoint_tfs[:, :, idxB]

            # 对这一段做插值
            tfs_segment = self.decoupled_rot_and_trans(A_tf, B_tf, num_points)
            # tfs_segment.shape = (4,4, num_points+1)，其中包含起点、终点

            # 若不是第一段的第一点，就把起点去重
            # 以免和上一次段的终点重复
            if i > 0:
                tfs_segment = tfs_segment[:, :, 1:]

            big_tf_list.append(tfs_segment)
        
        full_checkpoint_tfs = np.concatenate(big_tf_list, axis=2)
        return full_checkpoint_tfs
        # Your code ends here ------------------------------
       
        return full_checkpoint_tfs

    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        """This function takes two checkpoint transforms and computes the intermediate transformations
        that follow a straight line path by decoupling rotation and translation.
        Args:
            checkpoint_a_tf (np.ndarray): 4x4 transformation describing pose of checkpoint a.
            checkpoint_b_tf (np.ndarray): 4x4 transformation describing pose of checkpoint b.
            num_points (int): Number of intermediate points between checkpoint a and checkpoint b.
        Returns:
            tfs: 4x4x(num_points) homogeneous transformations matrices describing the full desired
            poses of the end-effector position from checkpoint a to checkpoint b following a linear path.
        """
        self.get_logger().info("checkpoint a")
        self.get_logger().info(str(checkpoint_a_tf))
        self.get_logger().info("checkpoint b")
        self.get_logger().info(str(checkpoint_b_tf))
        # TODO: implement this
        # Your code starts here ------------------------------
        pA = checkpoint_a_tf[0:3, 3]
        pB = checkpoint_b_tf[0:3, 3]
        # 提取旋转
        RA = checkpoint_a_tf[0:3, 0:3]
        RB = checkpoint_b_tf[0:3, 0:3]

        # 用对数映射做旋转插值(基本示例)
        RA_inv = np.linalg.inv(RA)
        R_rel = RA_inv @ RB
        # 计算旋转差的对数：Log(R_rel)
        log_R_rel = logm(R_rel)

        tfs = np.zeros((4, 4, num_points + 1))
        for i in range(num_points + 1):
            alpha = i / float(num_points)  # 0~1
            # 插值平移
            p_i = pA + alpha * (pB - pA)
            # 插值旋转：R_i = RA * exp(alpha * log(R_rel))
            R_i = RA @ expm(log_R_rel * alpha)

            # 组合成 4x4
            T_i = np.eye(4)
            T_i[0:3, 0:3] = R_i
            T_i[0:3, 3] = p_i
            tfs[:, :, i] = T_i

        return tfs
        # Your code ends here ------------------------------
        return tfs

    def full_checkpoints_to_joints(self, full_checkpoint_tfs, init_joint_position):
        """This function takes the full set of checkpoint transformations, including intermediate checkpoints, 
        and computes the associated joint positions by calling the ik_position_only() function.
        Args:
            full_checkpoint_tfs (np.ndarray, 4x4xn): 4x4xn transformations describing all the desired poses of the end-effector
            to follow the desired path.
            init_joint_position (np.ndarray):A 5x1 array for the initial joint position of the robot.
        Returns:
            q_checkpoints (np.ndarray, 5xn): For each pose, the solution of the position IK to get the joint position
            for that pose.
        """
        # TODO: Implement this
        # Your code starts here ------------------------------
        N   = full_checkpoint_tfs.shape[2]
        q_checkpoints = np.zeros((5, N))
        q_prev = init_joint_position.copy()  
        for i in range(N):
            pose_i = full_checkpoint_tfs[:, :, i]
            q_sol, err = self.ik_position_only(pose_i, q_prev)
            q_checkpoints[:, i] = q_sol
            q_prev = q_sol  

        return q_checkpoints
        # Your code ends here ------------------------------

        return q_checkpoints

    def ik_position_only(self, pose, q0, lam=0.25, num=500):
        """This function implements position only inverse kinematics.
        Args:
            pose (np.ndarray, 4x4): 4x4 transformations describing the pose of the end-effector position.
            q0 (np.ndarray, 5x1):A 5x1 array for the initial starting point of the algorithm.
        Returns:
            q (np.ndarray, 5x1): The IK solution for the given pose.
            error (float): The Cartesian error of the solution.
        """

        # TODO: Implement this
        # Some useful notes:
        # We are only interested in position control - take only the position part of the pose as well as elements of the
        # Jacobian that will affect the position of the error.

        # Your code starts here ------------------------------

        q = q0.copy()
        threshold = 1e-3 
        p_d = pose[0:3, 3]
        for i in range(num):
            # forward kin
            T_curr = self.kdl_youbot.forward_kinematics(q.tolist())
            p_curr = T_curr[0:3, 3]
            e = p_d - p_curr
            err_norm = np.linalg.norm(e)
            if err_norm < threshold:
                break
            J_full = self.kdl_youbot.get_jacobian(q.tolist())  # 6x5
            J_pos = J_full[0:3, :]  # 3x5

            # (J^T J + λ^2 I)^(-1) J^T e

            A = J_pos.T @ J_pos + (lam**2)*np.eye(5)  
            temp = J_pos.T @ e  
            dq = np.linalg.inv(A) @ temp

            q = q + dq  

        error= np.linalg.norm(p_d - p_curr)
        return q, error
        
        # Your code ends here ------------------------------

        return q, error


def main(args=None):
    rclpy.init(args=args)

    youbot_planner = YoubotTrajectoryPlanning()

    youbot_planner.run()

    rclpy.spin(youbot_planner)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    youbot_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()