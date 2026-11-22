"""Round-stable ArUco selection and matching Gazebo texture/SDF generation."""

from pathlib import Path
import secrets
import xml.etree.ElementTree as ET

import cv2


def next_marker_id(previous, count=50, randbelow=secrets.randbelow):
    if count < 2 or not 0 <= previous < count:
        raise ValueError('a random marker round requires at least two valid IDs')
    candidate = randbelow(count - 1)
    return candidate + (candidate >= previous)


def marker_sdf(marker_id, directory, model_file, dictionary_name='DICT_4X4_50'):
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    texture = directory / f'{dictionary_name}_{marker_id}.png'
    if not texture.exists():
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 256)
        if not cv2.imwrite(str(texture), marker):
            raise OSError(f'Unable to write {texture}')
    root = ET.parse(model_file).getroot()
    root.find('model').set('name', 'round_aruco')
    root.find('.//albedo_map').text = str(texture.absolute())
    return ET.tostring(root, encoding='unicode')
