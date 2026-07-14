from setuptools import find_packages, setup

package_name = 'bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='e2box',
    maintainer_email='e2b@e2box.co.kr',
    description='Serial bridge node for encoder feedback and cmd_vel control.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'serial_bridge = bridge_pkg.serial_bridge:main',
        ],
    },
)
