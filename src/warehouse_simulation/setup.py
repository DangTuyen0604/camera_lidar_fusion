from glob import glob
import os

from setuptools import setup

package_name = 'warehouse_simulation'
model_files = []
for model in ('shelf', 'pallet', 'cargo_box', 'worker', 'conveyor', 'docking_station'):
    for path in glob(os.path.join('models', model, '*')):
        model_files.append((os.path.join('share', package_name, 'models', model), [path]))

setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ] + model_files,
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Nguyen Dang Tuyen', maintainer_email='nguyendangtuyen062004@gmail.com',
    description='Deterministic warehouse simulation', license='Apache-2.0',
    entry_points={'console_scripts': [
        'scenario_runner = warehouse_simulation.scenario_runner:main']},
)
