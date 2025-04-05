from setuptools import setup
from glob import glob

package_name = 'focus_one_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
  	('share/' + package_name+'/urdf/', glob('urdf/*')),
    ('share/' + package_name+'/world/', glob('world/*')),
  	('share/' + package_name+'/rviz/', glob('rviz/*')),
  	('share/' + package_name+'/meshes/collision/', glob('meshes/collision/*')),
  	('share/' + package_name+'/meshes/visual/', glob('meshes/visual/*')),
    ('share/' + package_name+'/config/', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='TODO:',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feedback_monitor = focus_one_robot.feedback:main',
            'joint_1_controller = focus_one_robot.motor_1:main',
            'joint_2_controller = focus_one_robot.motor_2:main',
            'joint_3_controller = focus_one_robot.motor_3:main',
            'joint_4_controller = focus_one_robot.motor_4:main',
            'joint_5_controller = focus_one_robot.motor_5:main',
            'joint_6_controller = focus_one_robot.motor_6:main',
            'mpc_controller = focus_one_robot.mpc:main',
            'manipulator_controller = focus_one_robot.minpultor:main',
            'kinematics_controller = focus_one_robot.kinematics:main',
            'piston_controller = focus_one_robot.piston:main',
        ],
    },
)
