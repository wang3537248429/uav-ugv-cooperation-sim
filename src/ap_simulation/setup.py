import os
from glob import glob

from setuptools import setup

package_name = 'ap_simulation'


def recursive_data_files(directory):
    """Install a directory tree while preserving its package-relative paths."""
    data_files = []
    for root, _, files in os.walk(directory):
        if files:
            data_files.append(
                (
                    os.path.join('share', package_name, root),
                    [os.path.join(root, filename) for filename in files],
                )
            )
    return data_files


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.parm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.config')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ] + recursive_data_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AeroMind Sim contributors',
    maintainer_email='contributors@aeromind-sim.invalid',
    description='Gazebo and ArduPilot SITL competition simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_view = ap_simulation.camera_view:main',
            'sim_camera_node = ap_simulation.sim_camera_node:main',
            'gz_pose_bridge = ap_simulation.gz_pose_bridge:main',
            'gz_entity_sync = ap_simulation.gz_entity_sync:main',
            'sim_ugv_node = ap_simulation.sim_ugv_node:main',
            'motor_actuator_bridge = ap_simulation.motor_actuator_bridge:main',
            'sim_truth_external_nav = ap_simulation.sim_truth_external_nav:main',
            'optical_flow_odometry = ap_simulation.optical_flow_odometry_node:main',
            'sim_optical_flow_sensor = ap_simulation.sim_optical_flow_sensor:main',
            'sitl_json_state_bridge = ap_simulation.sitl_json_state_bridge:main',
            'competition_field_markers = ap_simulation.competition_field_markers:main',
        ],
    },
)
