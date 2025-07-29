from setuptools import setup
import os
from glob import glob

package_name = 'multi_robot_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🔄 加入launch檔案
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Multi robot exploration package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = multi_robot_exploration.path_planner:main',
            # 🔄 加入新的執行檔
            'robot_command_dispatcher = multi_robot_exploration.robot_command_dispatcher:main',
        ],
    },
)