from setuptools import setup, find_packages

setup(
    name='mam_python',
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    author='Nest Masters',
    description='MAM Robot - Squirrel Cup Eurobot 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros2_node = mam_python.ros2_node:main',
        ],
    },
)
