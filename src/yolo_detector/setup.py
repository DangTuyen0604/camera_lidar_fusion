from setuptools import find_packages, setup


package_name = 'yolo_detector'


setup(
    name=package_name,
    version='0.1.0',
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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nguyen Dang Tuyen',
    maintainer_email='nguyendangtuyen062004@gmail.com',
    description='YOLO 2D detection and KITTI camera-LiDAR fusion.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = '
            'yolo_detector.yolo_detector_node:main',
            'simulation_color_detector = '
            'yolo_detector.simulation_color_detector_node:main',
        ],
    },
)
