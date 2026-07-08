from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'imu_pkg'

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
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='e2box',
    maintainer_email='e2b@e2box.co.kr',
    description='IMU publisher and debug subscriber nodes.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_pkg.imu_publisher:main',
            'imu_subscriber = imu_pkg.imu_subscriber:main',
        ],
    },
)
