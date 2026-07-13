from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mppi_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e2box',
    maintainer_email='e2b@e2box.co.kr',
    description='Bringup and path clients for Nav2 MPPI control.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mppi_path_client = mppi_bringup.mppi_path_client:main',
        ],
    },
)
