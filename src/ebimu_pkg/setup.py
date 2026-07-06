from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ebimu_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='e2box',
    maintainer_email='e2b@e2box.co.kr',
    description='EBIMU ROS2 strict IMU-only odometry package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ebimu_publisher = ebimu_pkg.ebimu_publisher:main',
            'ebimu_subscriber = ebimu_pkg.ebimu_subscriber:main',
            'imu_odometry = ebimu_pkg.imu_odometry:main',
            'encoder_publisher = ebimu_pkg.encoder_publisher:main',
            'encoder_imu_odometry = ebimu_pkg.encoder_imu_odometry:main',
            'encoder_odometry = ebimu_pkg.encoder_odometry:main'
        ],
    },
)
