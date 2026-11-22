#!/usr/bin/env python3
"""Launch the simulation-only competition stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def validate_navigation(context):
    """Fail before starting physics if a map or route cannot be used."""
    from ap_navigation.map_io import load_map
    from ap_navigation.route_io import load_route
    from ap_simulation.ugv_geometry import load_obstacles, validate_path
    validate_path(load_obstacles(os.path.join(
        get_package_share_directory('ap_simulation'), 'worlds', 'competition_world.sdf')))
    source = LaunchConfiguration('external_nav_source').perform(context)
    if source not in ('truth', 'odometry', 'mapping', 'localization'):
        raise ValueError('external_nav_source must be truth, odometry, mapping or localization')
    actions = []
    if source in ('mapping', 'localization'):
        map_file = os.path.abspath(os.path.expanduser(LaunchConfiguration('map_file').perform(context)))
        if source == 'mapping' and os.path.exists(map_file):
            raise ValueError(f'Map already exists: {map_file}; use --localize or choose another --map path')
        if source == 'localization':
            load_map(map_file)
        if LaunchConfiguration('localization_backend').perform(context) != 'external_nav':
            raise ValueError('Point-cloud navigation requires localization_backend:=external_nav')
        actions.append(SetLaunchConfiguration('map_file', map_file))
    route = LaunchConfiguration('waypoint_file').perform(context)
    if not route and source in ('truth', 'mapping', 'localization'):
        route = os.path.join(get_package_share_directory('ap_navigation'), 'config', 'competition_waypoints.yaml')
    if route:
        route = os.path.abspath(os.path.expanduser(route))
        load_route(route)
    actions.append(SetLaunchConfiguration('waypoint_file', route))
    profile = (LaunchConfiguration('flight_profile').perform(context) if source in ('truth', 'mapping', 'localization')
               else os.path.join(get_package_share_directory('ap_simulation'), 'config', 'sim_params.yaml'))
    actions.append(SetLaunchConfiguration('active_flight_profile', profile))
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("ap_simulation")
    world = os.path.join(package_share, "worlds", "competition_world.sdf")
    models = os.path.join(package_share, "models")
    params = os.path.join(package_share, "config", "sim_params.yaml")
    sitl_params = os.path.join(package_share, "config", "sitl_params.parm")
    gz_gui_config = os.path.join(package_share, "config", "gz_gui.config")
    rviz_config = os.path.join(package_share, "rviz", "competition.rviz")
    navigation_share = get_package_share_directory('ap_navigation')
    local_dir = os.environ.get('COMPETITION_LOCAL_DIR', os.path.expanduser('~/.cache/aeromind-sim'))

    inherited_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        path for path in (models, inherited_resources) if path
    )
    gazebo_environment = {"GZ_SIM_RESOURCE_PATH": resource_path}
    sitl_workdir = os.environ.get(
        "SITL_WORKDIR",
        os.path.join(local_dir, 'sitl'),
    )
    os.makedirs(sitl_workdir, exist_ok=True)

    gz_headless = LaunchConfiguration("gz_headless")
    rviz = LaunchConfiguration("rviz")
    optical_flow_odom = LaunchConfiguration("optical_flow_odom")
    simulate_optical_flow = LaunchConfiguration("simulate_optical_flow")
    external_nav_source = LaunchConfiguration("external_nav_source")
    localization_backend = LaunchConfiguration("localization_backend")
    auto_start = LaunchConfiguration("auto_start")
    start_sitl = LaunchConfiguration("start_sitl")
    arducopter_bin = LaunchConfiguration("arducopter_bin")
    gz_partition = LaunchConfiguration("gz_partition")
    pointcloud_navigation_enabled = PythonExpression(
        ["'", external_nav_source, "' in ('mapping', 'localization')"]
    )
    native_guidance_enabled = PythonExpression(
        ["'", external_nav_source, "' in ('truth', 'mapping', 'localization')"]
    )

    # Keep server and client as separate processes. Gazebo Sim 7's combined
    # `gz sim <world>` launcher may reuse an unrelated GUI session when
    # another project is already open, even though its transport is
    # partitioned. A dedicated server is authoritative in both modes.
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "--headless-rendering", world],
        output="screen",
        name="gazebo_server",
        additional_env=gazebo_environment,
    )
    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g", "--gui-config", gz_gui_config],
        output="screen",
        name="gazebo_gui",
        additional_env=gazebo_environment,
        condition=UnlessCondition(gz_headless),
    )
    # The JSON backend receives Gazebo state on UDP 9002. ``-w`` guarantees
    # that this launch owns the SITL parameter set instead of stale EEPROM.
    sitl = ExecuteProcess(
        cmd=[
            arducopter_bin,
            "-w",
            "--model",
            "JSON",
            "--speedup",
            "1",
            "--rate",
            "200",
            "--defaults",
            sitl_params,
        ],
        output="screen",
        name="arducopter_sitl",
        cwd=sitl_workdir,
        condition=IfCondition(start_sitl),
    )

    gz_pose_bridge = Node(
        package="ap_simulation",
        executable="gz_pose_bridge",
        parameters=[params, {'gz_topic': PythonExpression([
            "'/model/drone/pose' if '", external_nav_source,
            "' in ('truth', 'mapping', 'localization') else '/world/competition_arena/dynamic_pose/info'"
        ])}],
        output="screen",
    )
    gz_entity_sync = Node(
        package="ap_simulation",
        executable="gz_entity_sync",
        parameters=[params],
        output="screen",
    )
    scene_service_bridge = Node(
        package='ap_gazebo_bridge', executable='scene_service_bridge', name='scene_service_bridge',
        output='screen',
    )
    ugv_pose_bridge = Node(
        package='ap_simulation', executable='gz_pose_bridge', name='ugv_pose_bridge',
        parameters=[{'gz_topic': '/model/ugv_rover/pose', 'model_name': 'ugv_rover',
                     'publish_topic': '/sim/ugv_pose_ned', 'publish_rate': 30.,
                     'use_initial_origin': False, 'origin_x': -2.25, 'origin_y': 0., 'origin_z': 0.}],
        output='screen',
    )
    sitl_json_state_bridge = Node(
        package="ap_simulation",
        executable="sitl_json_state_bridge",
        parameters=[params, {'synchronize_with_pose_time': native_guidance_enabled}],
        output="screen",
    )
    sim_truth_external_nav = Node(
        package="ap_simulation",
        executable="sim_truth_external_nav",
        parameters=[params, {"input_mode": external_nav_source}],
        output="screen",
        condition=IfCondition(PythonExpression(["'", external_nav_source, "' in ('truth', 'odometry')"])),
    )
    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge',
        arguments=['/navigation/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        output='screen', condition=IfCondition(pointcloud_navigation_enabled),
    )
    navigation = Node(
        package='ap_navigation', executable='pointcloud_navigation',
        parameters=[LaunchConfiguration('navigation_params'), {
            'mode': external_nav_source, 'map_file': LaunchConfiguration('map_file'),
        }],
        output='screen', condition=IfCondition(pointcloud_navigation_enabled),
    )
    sim_camera = Node(
        package="ap_simulation",
        executable="sim_camera_node",
        parameters=[params],
        output="screen",
    )
    sim_ugv = Node(
        package="ap_simulation",
        executable="sim_ugv_node",
        parameters=[params],
        output="screen",
    )
    flow_pipeline_enabled = PythonExpression(
        [
            "'", optical_flow_odom, "' == 'true' or '",
            external_nav_source, "' == 'odometry' or '",
            simulate_optical_flow, "' == 'true'",
        ]
    )
    sim_optical_flow_sensor = Node(
        package="ap_simulation",
        executable="sim_optical_flow_sensor",
        parameters=[params],
        output="screen",
        condition=IfCondition(flow_pipeline_enabled),
    )
    optical_flow_odometry = Node(
        package="ap_simulation",
        executable="optical_flow_odometry",
        parameters=[params],
        output="screen",
        condition=IfCondition(flow_pipeline_enabled),
    )
    competition_markers = Node(
        package="ap_simulation",
        executable="competition_field_markers",
        output="screen",
    )

    gz_imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            (
                "/world/competition_arena/model/drone/link/base_link/"
                "sensor/imu_sensor/imu"
                "@sensor_msgs/msg/Imu[gz.msgs.IMU"
            ),
        ],
        output="screen",
        name="gz_imu_bridge",
    )
    motor_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            (
                "/drone/command/motor_speed"
                "@actuator_msgs/msg/Actuators]gz.msgs.Actuators"
            ),
        ],
        output="screen",
        name="motor_gz_bridge",
    )
    motor_actuator_bridge = Node(
        package="ap_simulation",
        executable="motor_actuator_bridge",
        parameters=[params],
        output="screen",
    )
    mavlink_bridge = Node(
        package="ap_mavlink_bridge",
        executable="mavlink_bridge_node",
        parameters=[params, LaunchConfiguration('active_flight_profile'), {
            "localization_backend": localization_backend,
            "gcs_output_url": LaunchConfiguration('gcs_output_url'),
            'force_vision_flat_attitude': PythonExpression(
                ["'", external_nav_source, "' not in ('mapping', 'localization')"]
            ),
            'ap_viso_yaw_noise_rad': PythonExpression(
                ["0.05 if '", external_nav_source, "' in ('mapping', 'localization') else 0.2"]
            ),
            'vision_speed_lpf_alpha': PythonExpression(
                ["0.35 if '", external_nav_source, "' in ('mapping', 'localization') else 0.75"]
            ),
            'set_ekf_origin_on_start': native_guidance_enabled,
            'set_home_on_start': native_guidance_enabled,
        }],
        output="screen",
    )

    red_ball_detector = Node(
        package="ap_perception",
        executable="red_ball_detect_node",
        output="screen",
    )
    code_detector = Node(
        package="ap_perception",
        executable="code_detect_node",
        parameters=[params],
        output="screen",
    )
    frame_detector = Node(
        package="ap_perception",
        executable="frame_detect_node",
        output="screen",
    )
    mission = Node(
        package="ap_mission",
        executable="uav_mission_state_machine",
        parameters=[params, LaunchConfiguration('active_flight_profile'), {
            "localization_backend": localization_backend,
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            # The truth reference uses the same native takeoff and guidance
            # profile as lidar navigation, avoiding a low-altitude RC handoff.
            'native_guided_takeoff': native_guidance_enabled,
            # Rendering and RViz can delay ArduPilot's LAND touchdown
            # telemetry enough to keep the native LAND submode armed. The
            # point-cloud profile already has a bounded GUIDED_DESCEND
            # controller with XY hold; use it for deterministic visual runs,
            # including the truth reference. The flow baseline retains LAND.
            'native_autopilot_landing': PythonExpression(
                ["'False' if '", external_nav_source,
                 "' in ('truth', 'mapping', 'localization') else 'True'"]
            ),
            'post_puncture_home_speed_max': PythonExpression(
                ["0.02 if '", external_nav_source, "' in ('truth', 'mapping', 'localization') else 0.06"]
            ),
            'post_puncture_home_settle_time': PythonExpression(
                ["3.0 if '", external_nav_source, "' in ('truth', 'mapping', 'localization') else 1.0"]
            ),
            'prefer_takeoff_rc_fallback': PythonExpression(
                ["'", external_nav_source, "' == 'odometry'"]
            ),
            'allow_takeoff_rc_fallback': PythonExpression(
                ["'", external_nav_source, "' == 'odometry'"]
            ),
            'enable_sim_truth_monitor': PythonExpression(
                ["'", external_nav_source, "' not in ('mapping', 'localization')"]
            ),
        }],
        output="screen",
    )
    mission_start = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/mission/start",
            "std_msgs/msg/Bool",
            "{data: true}",
            "--once",
        ],
        output="screen",
        condition=IfCondition(auto_start),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(rviz),
    )
    camera_view = Node(
        package='ap_simulation', executable='camera_view', output='screen',
        condition=IfCondition(PythonExpression([
            "'", gz_headless, "'.lower() == 'false' and '", LaunchConfiguration('camera_view'),
            "'.lower() == 'true'"])),
    )
    rviz_frame_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="uav_map_to_local_ned",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "3.141592653589793",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "uav_map",
            "--child-frame-id", "local_ned",
        ],
        output="screen",
    )

    default_arducopter = os.environ.get(
        'ARDUCOPTER_BIN',
        os.path.join(local_dir, 'dependencies', 'ardupilot', 'build', 'sitl', 'bin', 'arducopter'),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gz_headless", default_value="false"),
            DeclareLaunchArgument(
                'worker_threads', default_value='1',
                description='Workers per OpenCV/BLAS process; avoids CPU oversubscription'),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                'camera_view', default_value='true', description='Open the live downward camera window'),
            DeclareLaunchArgument("optical_flow_odom", default_value="false"),
            DeclareLaunchArgument(
                "simulate_optical_flow",
                default_value="false",
                description="Publish idealized flow/range samples from Gazebo truth",
            ),
            DeclareLaunchArgument(
                "localization_backend",
                default_value="external_nav",
                description="Navigation estimate: external_nav, fcu_local, or optical_flow_ekf",
            ),
            DeclareLaunchArgument(
                "external_nav_source",
                default_value="mapping",
                description="ExternalNav input: mapping, localization, truth (baseline), or odometry",
            ),
            DeclareLaunchArgument('map_file', default_value=os.path.join(local_dir, 'maps', 'arena.npz')),
            DeclareLaunchArgument('waypoint_file', default_value='', description='Map-frame waypoint YAML'),
            DeclareLaunchArgument('navigation_params', default_value=os.path.join(
                navigation_share, 'config', 'navigation.yaml')),
            DeclareLaunchArgument('flight_profile', default_value=os.path.join(
                package_share, 'config', 'pointcloud_flight.yaml'),
                description='Point-cloud flight speed/tuning profile'),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument(
                "gcs_output_url",
                default_value="",
                description=(
                    "Optional pymavlink output for QGroundControl/Mission Planner, "
                    "for example udpout:127.0.0.1:14550"
                ),
            ),
            DeclareLaunchArgument("start_sitl", default_value="true"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value="competition_ws",
                description=(
                    "Gazebo Transport partition; isolates this arena from "
                    "unrelated Gazebo servers and GUI sessions"
                ),
            ),
            DeclareLaunchArgument(
                "arducopter_bin",
                default_value=default_arducopter,
                description="Path to an ArduCopter SITL executable",
            ),
            OpaqueFunction(function=validate_navigation),
            # Apply before Gazebo, ros_gz bridges and simulation nodes start.
            # Their child `gz topic` / `gz service` processes inherit the same
            # partition and cannot accidentally attach to another project.
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            SetEnvironmentVariable('OPENCV_FOR_THREADS_NUM', LaunchConfiguration('worker_threads')),
            SetEnvironmentVariable('OPENBLAS_NUM_THREADS', LaunchConfiguration('worker_threads')),
            SetEnvironmentVariable('OMP_NUM_THREADS', LaunchConfiguration('worker_threads')),
            SetEnvironmentVariable('MKL_NUM_THREADS', LaunchConfiguration('worker_threads')),
            LogInfo(
                msg=(
                    "\n[competition_ws] Starting Gazebo motor physics, "
                    "ArduPilot SITL, perception and mission state machine"
                )
            ),
            gazebo_server,
            TimerAction(period=2.0, actions=[gazebo_gui]),
            gz_pose_bridge,
            gz_entity_sync,
            scene_service_bridge,
            ugv_pose_bridge,
            gz_imu_bridge,
            sitl_json_state_bridge,
            sim_truth_external_nav,
            lidar_bridge,
            navigation,
            sim_optical_flow_sensor,
            optical_flow_odometry,
            sim_camera,
            rviz_frame_transform,
            rviz_node,
            camera_view,
            TimerAction(period=2.0, actions=[sitl]),
            TimerAction(period=8.0, actions=[mavlink_bridge]),
            TimerAction(
                period=9.0,
                actions=[
                    motor_gz_bridge,
                    motor_actuator_bridge,
                    sim_ugv,
                    competition_markers,
                ],
            ),
            TimerAction(
                period=13.0,
                actions=[
                    red_ball_detector,
                    code_detector,
                    frame_detector,
                ],
            ),
            TimerAction(period=18.0, actions=[mission]),
            TimerAction(period=24.0, actions=[mission_start]),
        ]
    )
