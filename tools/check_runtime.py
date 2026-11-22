#!/usr/bin/env python3
"""Check installed node imports, image ABI compatibility and bundled assets."""

import importlib
from pathlib import Path
import pkgutil
import subprocess
import sys
import xml.etree.ElementTree as ET


PACKAGES = ('ap_mavlink_bridge', 'ap_mission', 'ap_navigation', 'ap_perception', 'ap_simulation')


def check_python():
    for name in (*PACKAGES, 'PyQt5.QtWidgets'):
        package = importlib.import_module(name)
        if name in PACKAGES:
            for module in pkgutil.walk_packages(package.__path__, name + '.'):
                importlib.import_module(module.name)

    import cv2
    import numpy as np
    from cv_bridge import CvBridge

    # ROS Humble's cv_bridge is built against NumPy 1.x. Test the actual ABI,
    # and the ArUco API used by the camera and perception nodes.
    if int(np.__version__.split('.')[0]) != 1:
        raise RuntimeError('ROS Humble requires NumPy 1.x; install requirements.txt')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    cv2.aruco.generateImageMarker(dictionary, 0, 32)
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    bridge = CvBridge()
    restored = bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(image, 'bgr8'), 'bgr8')
    if not np.array_equal(image, restored):
        raise RuntimeError('cv_bridge image round-trip failed')


def check_assets():
    from ament_index_python.packages import get_package_share_directory

    for name in (*PACKAGES, 'ap_gazebo_bridge', 'ros_gz_bridge', 'ros_gz_interfaces'):
        share = Path(get_package_share_directory(name))
        if not (share / 'package.xml').is_file():
            raise FileNotFoundError(f'Missing installed package manifest: {name}')
    share = Path(get_package_share_directory('ap_simulation'))
    for relative in ('config/sim_params.yaml', 'config/sitl_params.parm', 'config/pointcloud_flight.yaml',
                     'config/gz_gui.config', 'rviz/competition.rviz', 'worlds/competition_world.sdf'):
        if not (share / relative).is_file():
            raise FileNotFoundError(f'Missing installed simulation resource: {relative}')
    check_model_resources(share)


def check_model_resources(share):
    models = share / 'models'
    for sdf in [share / 'worlds/competition_world.sdf', *models.rglob('model.sdf')]:
        for element in ET.parse(sdf).iter():
            value = (element.text or '').strip()
            if element.tag in ('uri', 'albedo_map') and value.startswith('model://'):
                resource = models / value.removeprefix('model://')
                if not resource.exists():
                    raise FileNotFoundError(f'Missing bundled model resource: {value}')
            elif element.tag == 'albedo_map' and value and not (sdf.parent / value).is_file():
                raise FileNotFoundError(f'Missing bundled texture in {sdf.name}: {value}')


def main():
    try:
        check_python()
        check_assets()
        result = subprocess.run(['gz', 'sim', '--versions'], capture_output=True, text=True, check=True)
        if not any(line.strip().startswith('7.') for line in result.stdout.splitlines()):
            raise RuntimeError('Gazebo Sim 7 (Garden) is required')
    except Exception as error:
        print(f'运行依赖检查失败：{error}\n请运行 make setup 安装依赖，或 make build 重新构建。', file=sys.stderr)
        return 1
    print('运行依赖检查通过：节点导入、OpenCV ArUco、cv_bridge、场景资源和 Gazebo Sim 7。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
