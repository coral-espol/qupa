from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qupa_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Torres',
    maintainer_email='davatorr@espol.edu.ec',
    description='Hardware drivers for QUPA robot',
    license='APACHE-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_scanner         = qupa_hardware.ir_scanner_node:main',
            'motor_driver       = qupa_hardware.motor_driver_node:main',
            'camera             = qupa_hardware.camera_node:main',
            'camera_calibration = qupa_hardware.camera_calibration_node:main',
            'floor_sensor       = qupa_hardware.floor_sensor_node:main',
            'led                = qupa_hardware.led_node:main',
        ],
    },
)
