from setuptools import find_packages, setup

package_name = 'drone_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seatech',
    maintainer_email='tatayoyoh@mymail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'traj_ctrl = drone_trajectory.trajectory_control:main',
            'square_traj = drone_trajectory.square_trajectory:main',
            'circle_traj = drone_trajectory.circle_trajectory:main',
            'seq_traj = drone_trajectory.sequence_trajectory:main',
            'waypoint_follower = drone_trajectory.waypoint_follow:main',
            'waypoint_follower2 = drone_trajectory.waypoint_follow2:main',
            'waypoint_follower3 = drone_trajectory.waypoint_follow3:main',
            'sonar_graph = drone_trajectory.sonar_graph:main',
        ],
    },
)
