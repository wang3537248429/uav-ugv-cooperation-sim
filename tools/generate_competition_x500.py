#!/usr/bin/env python3
"""Generate the competition-sized model from the vendored PX4 x500 base.

The upstream x500_base directory is intentionally immutable. This generator
scales its geometry and link offsets, then derives a non-rotating competition
guard collision envelope while retaining the flight mass/inertia calibration
that the ArduPilot competition controller uses.
"""

from __future__ import annotations

import argparse
import copy
import difflib
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = (
    ROOT / "src" / "ap_simulation" / "models" / "competition_x500"
)
UPSTREAM = (
    ROOT / "src" / "ap_simulation" / "models" / "x500_base" / "model.sdf"
)
TEMPLATE = MODEL_DIR / "model.sdf.in"
OUTPUT = MODEL_DIR / "model.sdf"
SCALE = 0.67

BASE_MASS = 1.5356923076923077
BASE_INERTIA = {
    "ixx": 0.01663666666666667,
    "ixy": 0.0,
    "ixz": 0.0,
    "iyy": 0.01663666666666667,
    "iyz": 0.0,
    "izz": 0.03071384615384616,
}
ROTOR_JOINT_DAMPING = 0.004
BASE_LINEAR_VELOCITY_DECAY = 0.40
BASE_ANGULAR_VELOCITY_DECAY = 0.05
BASE_COLLISIONS = {
    "base_link_collision_0": ("0 0 0.007 0 0 0", "0.235 0.235 0.04"),
    "base_link_collision_1": ("0 -0.065 -0.105 -0.35 0 0", "0.012 0.012 0.16"),
    "base_link_collision_2": ("0 0.065 -0.105 0.35 0 0", "0.012 0.012 0.16"),
    "base_link_collision_3": ("0 -0.088 -0.18 0 0 0", "0.165 0.012 0.012"),
    "base_link_collision_4": ("0 0.088 -0.18 0 0 0", "0.165 0.012 0.012"),
}
GUARD_CENTRES = (
    (0.11658, -0.11658),
    (-0.11658, 0.11658),
    (0.11658, 0.11658),
    (-0.11658, -0.11658),
)
GUARD_RADIUS = 0.048
GUARD_COLLISION_LENGTH = 0.010
ROTOR_Z = 0.045


def parse_xml(path: Path) -> ET.ElementTree:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def scaled_text(text: str, factor: float) -> str:
    return " ".join(f"{float(value) * factor:.12g}" for value in text.split())


def scale_pose_translations(model: ET.Element) -> None:
    for pose in model.iter("pose"):
        if not pose.text:
            continue
        values = pose.text.split()
        if len(values) < 3:
            continue
        values[:3] = [
            f"{float(value) * SCALE:.12g}"
            for value in values[:3]
        ]
        pose.text = " ".join(values)


def scale_geometries(model: ET.Element) -> None:
    for box in model.iter("box"):
        size = box.find("size")
        if size is not None and size.text:
            size.text = scaled_text(size.text, SCALE)
    for cylinder in model.iter("cylinder"):
        for tag in ("radius", "length"):
            value = cylinder.find(tag)
            if value is not None and value.text:
                value.text = f"{float(value.text) * SCALE:.12g}"
    for sphere in model.iter("sphere"):
        radius = sphere.find("radius")
        if radius is not None and radius.text:
            radius.text = f"{float(radius.text) * SCALE:.12g}"
    for plane in model.iter("plane"):
        size = plane.find("size")
        if size is not None and size.text:
            size.text = scaled_text(size.text, SCALE)
    for mesh in model.iter("mesh"):
        scale = mesh.find("scale")
        if scale is None:
            scale = ET.SubElement(mesh, "scale")
            scale.text = f"{SCALE:.12g} {SCALE:.12g} {SCALE:.12g}"
        elif scale.text:
            scale.text = scaled_text(scale.text, SCALE)


def apply_competition_dynamics(model: ET.Element) -> None:
    base_link = next(
        link for link in model.iter("link")
        if link.get("name") == "base_link"
    )
    mass = base_link.find("./inertial/mass")
    if mass is None:
        raise ValueError("PX4 base_link has no inertial mass")
    mass.text = f"{BASE_MASS:.16g}"
    inertia = base_link.find("./inertial/inertia")
    if inertia is None:
        raise ValueError("PX4 base_link has no inertia matrix")
    for tag, value in BASE_INERTIA.items():
        element = inertia.find(tag)
        if element is None:
            raise ValueError(f"PX4 base_link inertia has no {tag}")
        element.text = f"{value:.16g}"

    # The scaled competition aircraft carries four propeller guards and a
    # needle mount, while PX4's upstream drag is calibrated for the bare
    # full-size x500. A small body velocity decay represents the extra
    # aerodynamic damping and prevents physically unrealistic metre-scale
    # coasting after the FCU has already commanded a stop.
    velocity_decay = base_link.find("velocity_decay")
    if velocity_decay is None:
        velocity_decay = ET.SubElement(base_link, "velocity_decay")
    linear = velocity_decay.find("linear")
    if linear is None:
        linear = ET.SubElement(velocity_decay, "linear")
    linear.text = f"{BASE_LINEAR_VELOCITY_DECAY:.12g}"
    angular = velocity_decay.find("angular")
    if angular is None:
        angular = ET.SubElement(velocity_decay, "angular")
    angular.text = f"{BASE_ANGULAR_VELOCITY_DECAY:.12g}"

    # The official PX4 model is tuned for the upstream 492 mm frame. Keep
    # explicit rotor-joint damping at the competition wheelbase so the visual
    # rotor speed remains well behaved in Gazebo Sim 7. Physical contacts are
    # represented separately by non-rotating guards on base_link.
    for joint in model.iter("joint"):
        if not joint.get("name", "").startswith("rotor_"):
            continue
        dynamics = joint.find("./axis/dynamics")
        if dynamics is None:
            raise ValueError(f"{joint.get('name')} has no axis dynamics")
        damping = dynamics.find("damping")
        if damping is None:
            damping = ET.SubElement(dynamics, "damping")
        damping.text = f"{ROTOR_JOINT_DAMPING:.12g}"
        for tag in ("spring_reference", "spring_stiffness"):
            element = dynamics.find(tag)
            if element is not None:
                dynamics.remove(element)


def apply_competition_collision_geometry(model: ET.Element) -> None:
    """Use static body/guard contacts, never a high-speed propeller contact.

    The PX4 mesh, rotor joints and motor plugins remain authoritative. The
    upstream rotor collision boxes are inappropriate for a guarded 330 mm
    competition aircraft: when a spinning box grazes a thin obstacle, Gazebo
    can inject an unrealistically large rigid-body impulse. Four base_link
    cylinders follow the visible guards exactly and therefore preserve the
    competition footprint without turning contact into a rotor strike.
    """
    base_link = next(
        link for link in model.iter("link")
        if link.get("name") == "base_link"
    )
    for name, (pose_text, size_text) in BASE_COLLISIONS.items():
        collision = base_link.find(f"./collision[@name='{name}']")
        if collision is None:
            raise ValueError(f"PX4 base_link has no {name}")
        pose = collision.find("pose")
        size = collision.find("./geometry/box/size")
        if pose is None or size is None:
            raise ValueError(f"{name} is not the expected box collision")
        pose.text = pose_text
        size.text = size_text

    surface_template = base_link.find(
        "./collision[@name='base_link_collision_0']/surface"
    )
    if surface_template is None:
        raise ValueError("PX4 base_link body collision has no surface settings")
    for index, (x, y) in enumerate(GUARD_CENTRES):
        collision = ET.SubElement(
            base_link,
            "collision",
            {"name": f"competition_prop_guard_collision_{index}"},
        )
        pose = ET.SubElement(collision, "pose")
        pose.text = f"{x:.12g} {y:.12g} {ROTOR_Z:.12g} 0 0 0"
        geometry = ET.SubElement(collision, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        radius = ET.SubElement(cylinder, "radius")
        radius.text = f"{GUARD_RADIUS:.12g}"
        length = ET.SubElement(cylinder, "length")
        length.text = f"{GUARD_COLLISION_LENGTH:.12g}"
        collision.append(copy.deepcopy(surface_template))

    for link in model.iter("link"):
        name = link.get("name", "")
        if not name.startswith("rotor_"):
            continue
        pose = link.find("pose")
        collisions = link.findall("collision")
        if pose is None or not pose.text or not collisions:
            raise ValueError(f"{name} has no expected pose/collision")
        values = pose.text.split()
        values[2] = f"{ROTOR_Z:.12g}"
        pose.text = " ".join(values)
        for collision in collisions:
            link.remove(collision)


def apply_gazebo_sim_7_compatibility(model: ET.Element) -> None:
    # PX4 currently carries gz_frame_id as a custom sensor child. Gazebo Sim 7
    # publishes the scoped sensor topic without it, while SDFormat 1.9 reports
    # the extension as an unknown element. Remove it only from the generated
    # derivative so the vendored upstream source stays byte-for-byte intact.
    for sensor in model.iter("sensor"):
        frame_id = sensor.find("gz_frame_id")
        if frame_id is not None:
            sensor.remove(frame_id)


def merge_competition_visuals_into_base_link(model: ET.Element) -> None:
    """Keep competition accessories visual-only to avoid a tiny fixed body."""
    base_link = next(
        link for link in model.findall("link")
        if link.get("name") == "base_link"
    )
    payload = next(
        link for link in model.findall("link")
        if link.get("name") == "competition_payload"
    )
    for visual in payload.findall("visual"):
        base_link.append(copy.deepcopy(visual))
    for sensor in payload.findall("sensor"):
        base_link.append(copy.deepcopy(sensor))
    model.remove(payload)

    payload_joint = next(
        joint for joint in model.findall("joint")
        if joint.get("name") == "competition_payload_joint"
    )
    model.remove(payload_joint)


def build_tree() -> ET.ElementTree:
    upstream_tree = parse_xml(UPSTREAM)
    upstream_model = upstream_tree.getroot().find("model")
    if upstream_model is None:
        raise ValueError("PX4 x500_base SDF has no model")
    upstream_model = copy.deepcopy(upstream_model)
    scale_pose_translations(upstream_model)
    scale_geometries(upstream_model)
    apply_competition_dynamics(upstream_model)
    apply_competition_collision_geometry(upstream_model)
    apply_gazebo_sim_7_compatibility(upstream_model)

    template_tree = parse_xml(TEMPLATE)
    output_model = template_tree.getroot().find("model")
    if output_model is None:
        raise ValueError("competition_x500 template has no model")
    include = output_model.find("include")
    if include is None or include.findtext("uri") != "model://x500_base":
        raise ValueError("competition_x500 template has no PX4 x500_base include")
    insertion_index = list(output_model).index(include)
    output_model.remove(include)

    upstream_children = [
        copy.deepcopy(child)
        for child in list(upstream_model)
        if isinstance(child.tag, str)
        and child.tag not in ("pose", "self_collide", "static")
    ]
    output_model.insert(insertion_index, ET.Element("self_collide"))
    output_model[insertion_index].text = "false"
    insertion_index += 1
    output_model.insert(insertion_index, ET.Element("static"))
    output_model[insertion_index].text = "false"
    insertion_index += 1
    for child in upstream_children:
        output_model.insert(insertion_index, child)
        insertion_index += 1
    merge_competition_visuals_into_base_link(output_model)

    ET.indent(template_tree, space="  ")
    return template_tree


def render_bytes() -> bytes:
    tree = build_tree()
    with tempfile.NamedTemporaryFile(suffix=".sdf") as temporary:
        tree.write(
            temporary.name,
            encoding="UTF-8",
            xml_declaration=True,
            short_empty_elements=True,
        )
        temporary.seek(0)
        return temporary.read()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--write",
        action="store_true",
        help="replace model.sdf with the deterministic generated output",
    )
    args = parser.parse_args()
    generated = render_bytes()
    if args.write:
        OUTPUT.write_bytes(generated)
        print(f"generated {OUTPUT.relative_to(ROOT)}")
        return 0

    current = OUTPUT.read_bytes() if OUTPUT.exists() else b""
    if current == generated:
        print("competition_x500/model.sdf is reproducible")
        return 0
    diff = difflib.unified_diff(
        current.decode("utf-8", errors="replace").splitlines(),
        generated.decode("utf-8", errors="replace").splitlines(),
        fromfile=str(OUTPUT),
        tofile="generated",
        lineterm="",
    )
    print("\n".join(diff))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
