from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'odom_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e2box',
    maintainer_email='e2b@e2box.co.kr',
    description='Odometry nodes for IMU, encoder, and encoder+IMU fusion.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'encoder_odometry = odom_pkg.encoder_odometry:main',
            'encoder_imu_odometry = odom_pkg.encoder_imu_odometry:main',
            'ekf_encoder_imu_odometry = odom_pkg.ekf_encoder_imu_odometry:main',
        ],
    },
)
