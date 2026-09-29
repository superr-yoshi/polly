from glob import glob

from setuptools import find_packages, setup

package_name = 'poli_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='수아',
    maintainer_email='teampolly33@gmail.com',
    description='POLI hardware layer: RRC Lite adapter, Arduino Mega bridge, fake nodes',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_rrc_node = poli_hardware.rrc_node:main_fake',
            'rrc_adapter_node = poli_hardware.rrc_node:main_real',
            'fake_mega_node = poli_hardware.fake_mega_node:main',
            'mega_bridge_node = poli_hardware.mega_bridge_node:main',
        ],
    },
)
