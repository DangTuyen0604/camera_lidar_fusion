from setuptools import find_packages, setup

package_name = 'kitti_ros2_player'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['PyYAML', 'setuptools'],
    zip_safe=True,
    maintainer='tuyen',
    maintainer_email='tuyen@example.com',
    description='ROS 2 player for KITTI camera and LiDAR data.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitti_player = '
            'kitti_ros2_player.kitti_player_node:main',
        ],
    },
)
