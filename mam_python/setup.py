from setuptools import setup, find_packages
from glob import glob
import os

setup(
    name='mam_python',
    version='0.1.0',
    packages=find_packages(exclude=['tests', 'launch']),
    package_data={'mam_python': []},
    data_files=[
        (os.path.join('share', 'mam_python', 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'ros2_node=mam_python.ros2_node:main',
        ],
    },
    zip_safe=False,
)
