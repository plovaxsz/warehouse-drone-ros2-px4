from setuptools import setup
import os
from glob import glob

package_name = 'px4_clean_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='plovaxs',
    maintainer_email='plovaxs@todo.todo',
    description='PX4 Navigation Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_sender = px4_clean_nav.cmd_sender:main',
            'scan_to_cloud = px4_clean_nav.scan_to_cloud:main',
            'surveillance_drone = px4_clean_nav.surveillance_drone:main',
            'offboard_control = px4_clean_nav.offboard_control:main',
            'tf_broadcaster = px4_clean_nav.tf_broadcaster:main',
        ],
    },
)
