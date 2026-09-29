from glob import glob

from setuptools import setup

package_name = 'warehouse_mujoco'

setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/mjcf', glob('mjcf/*.xml')),
    ],
    install_requires=['setuptools', 'mujoco>=3.2'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Nguyen Dang Tuyen', maintainer_email='nguyendangtuyen062004@gmail.com',
    description='MuJoCo warehouse simulator for the OpenAMRobot AMR', license='Apache-2.0',
    entry_points={'console_scripts': [
        'mujoco_sim = warehouse_mujoco.ros_node:main']},
)
