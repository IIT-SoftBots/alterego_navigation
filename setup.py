from glob import glob
from setuptools import setup

package_name = 'alterego_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AlterEGO Team',
    maintainer_email='luca.garello@iit.it',
    description='Nav2-based autonomous mapping and waypoint navigation for AlterEGO.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer = alterego_navigation.frontier_explorer:main',
            'waypoint_mission = alterego_navigation.waypoint_mission:main',
        ],
    },
)
