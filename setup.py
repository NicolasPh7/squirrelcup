from setuptools import setup, find_packages

setup(
    name='mam_python',
    version='1.0.0',
    description='MAM Eurobot 2026 - Python Core Software',
    author='Nicolas',
    packages=find_packages(),
    python_requires='>=3.8',
    install_requires=[
        'numpy',
        'rclpy',
        'geometry-msgs',
        'std-msgs',
        'sensor-msgs',
        'nav-msgs',
    ],
    entry_points={
        'console_scripts': [],
    },
)