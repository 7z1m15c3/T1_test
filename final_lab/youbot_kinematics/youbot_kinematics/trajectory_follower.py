import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import TransformStamped
# import the utils from the transform_helpers package
from transform_helpers.utils import rotmat2q
import tf2_ros
import numpy as np
import time


from youbot_kinematics.youbotKineStudent import YoubotKinematicStudent

class YoubotTrajectoryFollower(Node):
    def __init__(self):
        super().__init__('youbot_trajectory_follower')
        self.get_logger().info("Starting YoubotTrajectoryFollower node...")


        self.subscription = self.create_subscription(
            JointTrajectory, 
            '/EffortJointInterface_trajectory_controller/command',  
            self.trajectory_callback, 
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.kine = YoubotKinematicStudent()
        self.is_executing = False  

    def trajectory_callback(self, msg):
        """
        当监听到 JointTrajectory 消息时，被回调。
        """
        if self.is_executing:
            self.get_logger().warn("Already executing a trajectory, ignoring new one!")
            return
        
        self.is_executing = True
        self.get_logger().info(f"Received a trajectory with {len(msg.points)} points.")
        
        joint_names = msg.joint_names  # ["arm_joint_1", ..., "arm_joint_5"]
    
        prev_time = 0.0
        for point_idx, pt in enumerate(msg.points):
            joints = pt.positions  
            T = self.kine.forward_kinematics(list(joints))  # 4x4 numpy
            
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = "base_link"
            transform_stamped.child_frame_id = "youbot_end_effector"
            

            transform_stamped.transform.translation.x = T[0,3]
            transform_stamped.transform.translation.y = T[1,3]
            transform_stamped.transform.translation.z = T[2,3]

            rot_mat = T[0:3, 0:3]
            quat = rotmat2q(rot_mat)
            transform_stamped.transform.rotation.x = quat.x
            transform_stamped.transform.rotation.y = quat.y
            transform_stamped.transform.rotation.z = quat.z
            transform_stamped.transform.rotation.w = quat.w
            
            self.tf_broadcaster.sendTransform(transform_stamped)
            current_time = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = current_time - prev_time
            prev_time = current_time
            # sleep
            if dt > 0.0:
                self.get_logger().info(f"Trajectory point {point_idx}: sleeping {dt} sec.")
                time.sleep(dt)
        
        self.get_logger().info("Trajectory execution finished!")
        self.is_executing = False


def main(args=None):
    rclpy.init(args=args)

    node = YoubotTrajectoryFollower()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''
conda activate roboenv-py3.10
t1: ros2 launch youbot_kinematics bringup.launch.py
t2: ros2 run youbot_kinematics plan_trajectory
t3: ros2 run youbot_kinematics trajectory_follower
'''