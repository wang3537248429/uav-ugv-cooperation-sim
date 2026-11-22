from glob import glob
from setuptools import setup


setup(
    name='ap_navigation',
    version='1.0.0',
    packages=['ap_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ap_navigation']),
        ('share/ap_navigation', ['package.xml']),
        ('share/ap_navigation/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AeroMind Sim contributors',
    maintainer_email='contributors@aeromind-sim.invalid',
    description='Indoor point-cloud mapping and localization',
    license='MIT',
    entry_points={'console_scripts': [
        'pointcloud_navigation = ap_navigation.navigation_node:main',
        'map_tool = ap_navigation.map_tool:main',
    ]},
)
