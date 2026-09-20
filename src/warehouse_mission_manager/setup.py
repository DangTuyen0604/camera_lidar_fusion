from glob import glob

from setuptools import setup

package_name = 'warehouse_mission_manager'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/config', glob('config/*.yaml')),
                  ('share/' + package_name + '/launch', glob('launch/*.py'))],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      tests_require=['pytest'],
      maintainer='Nguyen Dang Tuyen', maintainer_email='nguyendangtuyen062004@gmail.com',
      description='Warehouse mission manager', license='Apache-2.0',
      entry_points={'console_scripts': [
          'mission_manager = warehouse_mission_manager.mission_manager:main']})
