from setuptools import setup

package_name = 'ap_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AeroMind Sim contributors',
    maintainer_email='contributors@aeromind-sim.invalid',
    description='UAV competition mission state machine',
    license='MIT',
    entry_points={
        'console_scripts': [
            'uav_mission_state_machine = ap_mission.uav_mission_state_machine:main',
        ],
    },
)
