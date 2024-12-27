# from setuptools import find_packages, setup

# package_name = 'youbot_kinematics'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/config', ['config/viz.rviz']),
#         ('share/' + package_name + '/launch', ['launch/bringup.launch.py'])
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='Eddie Edwards',
#     maintainer_email='eddie.edwards@ucl.ac.uk',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [ 'main_kdl = youbot_kinematics.youbotKineKDL:main',
#                              'main_student = youbot_kinematics.youbotKineStudent:main',
#                              'plan_trajectory = youbot_kinematics.plan_trajectory:main',
#         ],
#     },
# )

from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'youbot_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=['youbot_kinematics',
        'youbot_kinematics.xml_reflection', ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/viz.rviz']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py'])
        (os.path.join('share', package_name, 'youbot_description', 'urdfs'), 
        glob('youbot_description/urdfs/*.urdf')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Kinematics for youbot ...',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'main_kdl = youbot_kinematics.youbotKineKDL:main',
                             'main_student = youbot_kinematics.youbotKineStudent:main',
                             'plan_trajectory = youbot_kinematics.plan_trajectory:main',
                             'trajectory_follower = youbot_kinematics.trajectory_follower:main',
        ],
    },
)

