from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'field_robotics_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amvdeus',
    maintainer_email='emmanuelablo01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_circle_controller = field_robotics_assignment.controller_node:main',
            'frontier_explorer = field_robotics_assignment.frontier_explorer:main',
        ],
    },
)