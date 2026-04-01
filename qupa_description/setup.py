from setuptools import setup
import os
from glob import glob

package_name = 'qupa_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/**/*.xacro', recursive=True)),
        (os.path.join('share', package_name, 'urdf/bases'),
            glob('urdf/bases/*.xacro')),
        (os.path.join('share', package_name, 'urdf/sensors'),
            glob('urdf/sensors/*.xacro')),
        (os.path.join('share', package_name, 'urdf/wheels'),
            glob('urdf/wheels/*.xacro')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Torres',
    maintainer_email='davatorr@espol.edu.ec',
    description='URDF/xacro description for the QUPA robot',
    license='MIT',
)
