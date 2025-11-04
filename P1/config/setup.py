from setuptools import find_packages, setup

package_name = 'drone_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'drone_connector = drone_pkg.drone_connector:main',
        'telemetry_monitor = drone_pkg.telemetry_monitor:main',
        'video_viewer = drone_pkg.video_viewer:main',
        'battery_failsafe = drone_pkg.battery_failsafe:main',
        'mission_planner = drone_pkg.mission_planner:main',
        'object_detector_node = drone_pkg.object_detector_node:main'
    ],
},
)   
