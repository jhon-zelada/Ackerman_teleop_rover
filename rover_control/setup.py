from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhon',
    maintainer_email='jhon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'teleop_keyboard_ackermann_executable = rover_control.teleop_keyboard_ackermann:main',
            'teleop_joystick_ackermann_executable = rover_control.teleop_joystick_ackermann:main',
        ],
    },
)
