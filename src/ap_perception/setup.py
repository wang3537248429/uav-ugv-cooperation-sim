from setuptools import setup

package_name = 'ap_perception'

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
    description='Synthetic-camera competition perception',
    license='MIT',
    entry_points={
        'console_scripts': [
            'red_ball_detect_node = ap_perception.red_ball_detect_node:main',
            'code_detect_node = ap_perception.code_detect_node:main',
            'frame_detect_node = ap_perception.frame_detect_node:main',
        ],
    },
)
