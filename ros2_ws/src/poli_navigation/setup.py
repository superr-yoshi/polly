from glob import glob

from setuptools import find_packages, setup

package_name = 'poli_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='genie',
    maintainer_email='genie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'scan_monitor = poli_navigation.scan_monitor:main',
        'fake_scan = poli_navigation.fake_scan:main',
        'fake_odom = poli_navigation.fake_odom:main',
        'fake_robot = poli_navigation.fake_robot:main',
        'mission2 = poli_navigation.mission2_node:main',
        ],
    },
)

