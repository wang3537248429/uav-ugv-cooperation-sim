from setuptools import setup

package_name = 'ap_mavlink_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink>=2.4.49'],
    zip_safe=True,
    maintainer='AeroMind Sim contributors',
    maintainer_email='contributors@aeromind-sim.invalid',
    description='MAVLink bridge for ArduPilot SITL',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mavlink_bridge_node = ap_mavlink_bridge.mavlink_bridge_node:main',
        ],
    },
)
