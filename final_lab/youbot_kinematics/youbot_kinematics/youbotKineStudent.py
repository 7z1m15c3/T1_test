import rclpy
import time
import threading

import numpy as np
from youbot_kinematics.youbotKineBase import YoubotKinematicBase
from youbot_kinematics.target_data import TARGET_JOINT_POSITIONS

class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super(YoubotKinematicStudent, self).__init__(tf_suffix='student')

        # Set the offset for theta
        youbot_joint_offsets = [170.0 * np.pi / 180.0,
                                -65.0 * np.pi / 180.0,
                                146 * np.pi / 180,
                                -102.5 * np.pi / 180,
                                -167.5 * np.pi / 180]

        # Apply joint offsets to dh parameters
        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        # Joint reading polarity signs
        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint}
                w.r.t the base of the robot.
        """
        assert isinstance(self.dh_params, dict)
        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)
        assert up_to_joint >= 0
        assert up_to_joint <= len(self.dh_params['a'])

        T = np.identity(4)

        # Apply offset and polarity to joint readings (found in URDF file)
        joints_readings = [sign * angle for sign, angle in zip(self.youbot_joint_readings_polarity, joints_readings)]

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)
            
        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"
        return T

    def get_jacobian(self, joint):
        """Given the joint values of the robot, compute the Jacobian matrix.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5 which is the Jacobian matrix.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # TODO: create the jacobian matrix

        # Your code starts here ----------------------------

        # For your solution to match the KDL Jacobian, z0 needs to be set [0, 0, -1] instead of [0, 0, 1], since that is how its defined in the URDF.
        # Both are correct.
        # Your code starts here ----------------------------
        joint = [sign * angle for sign, angle in zip(
            self.youbot_joint_readings_polarity, joint
        )]

        T_mats = [np.eye(4)]
        for i in range(5):
            A_i = self.standard_dh(
                self.dh_params['a'][i],
                self.dh_params['alpha'][i],
                self.dh_params['d'][i],
                self.dh_params['theta'][i] + joint[i]
            )
            T_next = T_mats[i] @ A_i
            T_mats.append(T_next)

        p = []
        z = []
        for i in range(6):
            T0i = T_mats[i]
            p_i = T0i[0:3, 3]
            p.append(p_i)
            R0i = T0i[0:3, 0:3]
            # 注意题目说要与 KDL 一致，需要 z0 = [0, 0, -1]
            # 对所有关节都统一用 R0i*[0,0,-1].

            if i == 0:
                # 第0关节：z轴 [0, 0, -1]
                z_i = R0i @ np.array([0, 0, -1.0])
            else:
                # 其它关节：z轴 [0, 0, 1]
                z_i = R0i @ np.array([0, 0, 1.0])

            z.append(z_i)

        p_end = p[5]

        jacobian = np.zeros((6, 5))
        for i in range(5):
            # v_i = z_{i} x (p_end - p_i)
            v_i = np.cross(z[i], (p_end - p[i]))
            w_i = z[i]
            jacobian[0:3, i] = v_i
            jacobian[3:6, i] = w_i

        # Your code ends here ------------------------------
        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """Check for singularity condition given robot joints. Coursework 2 Question 4c.
        Reference Lecture 5 slide 30.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            singularity (bool): True if in singularity and False if not in singularity.

        """
        assert isinstance(joint, list)
        assert len(joint) == 5
        # TODO: Implement this
        # Your code starts here ----------------------------
        jacobian = self.get_jacobian(joint)

        # check the rack
        rank = np.linalg.matrix_rank(jacobian)

        # if rank < 6, then the robot is in a singularity
        singularity = rank < 6

        return singularity
        # Your code ends here ------------------------------
        assert isinstance(singularity, bool)
        return singularity


def main(args=None):
    rclpy.init(args=args)

    kinematic_student = YoubotKinematicStudent()

    for i in range(TARGET_JOINT_POSITIONS.shape[0]):
        target_joint_angles = TARGET_JOINT_POSITIONS[i]
        target_joint_angles = target_joint_angles.tolist()
        pose = kinematic_student.forward_kinematics(target_joint_angles)
        singularity = kinematic_student.check_singularity(target_joint_angles)
        # we would probably compute the jacobian at our current joint angles, not the target
        # but this is just to check your work
        jacobian = kinematic_student.get_jacobian(target_joint_angles)
        print("target joint angles")
        print(target_joint_angles)
        print("pose")
        print(pose)
        print("jacobian")
        print(jacobian)
        print("Is Singular:", singularity)

    rclpy.spin(kinematic_student)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kinematic_student.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()