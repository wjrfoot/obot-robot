from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'obot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Custom ROS 2 application nodes for OBot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tank_mixer_node = obot.tank_mixer_node:main',
            'wifi_signal_sampler = obot.wifi_signal_sampler:main',
            'wifi_heatmap_aggregator = obot.wifi_heatmap_aggregator:main',
            'wifi_area_surveyor = obot.wifi_area_surveyor:main',
        ],
    },
)