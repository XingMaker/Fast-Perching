import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_plan_hz = DeclareLaunchArgument(
        'plan_hz', default_value='5',
        description='Planning frequency')

    declare_K = DeclareLaunchArgument(
        'K', default_value='16',
        description='Number of key points')

    declare_vmax = DeclareLaunchArgument(
        'vmax', default_value='6.0',
        description='Maximum velocity')

    declare_amax = DeclareLaunchArgument(
        'amax', default_value='6.0',
        description='Maximum acceleration')

    declare_thrust_max = DeclareLaunchArgument(
        'thrust_max', default_value='17.0',
        description='Maximum thrust')

    declare_thrust_min = DeclareLaunchArgument(
        'thrust_min', default_value='5.0',
        description='Minimum thrust')

    declare_omega_max = DeclareLaunchArgument(
        'omega_max', default_value='3.0',
        description='Maximum angular velocity')

    declare_omega_yaw_max = DeclareLaunchArgument(
        'omega_yaw_max', default_value='0.5',
        description='Maximum yaw angular velocity')

    declare_robot_l = DeclareLaunchArgument(
        'robot_l', default_value='0.02',
        description='Robot length')

    declare_robot_r = DeclareLaunchArgument(
        'robot_r', default_value='0.13',
        description='Robot radius')

    declare_platform_r = DeclareLaunchArgument(
        'platform_r', default_value='1.0',
        description='Platform radius')

    declare_v_plus = DeclareLaunchArgument(
        'v_plus', default_value='0.3',
        description='Landing velocity offset')

    declare_rhoT = DeclareLaunchArgument(
        'rhoT', default_value='100000.0',
        description='Time regularization weight')

    declare_rhoP = DeclareLaunchArgument(
        'rhoP', default_value='10000000.0',
        description='Position penalty weight')

    declare_rhoV = DeclareLaunchArgument(
        'rhoV', default_value='1000.0',
        description='Velocity penalty weight')

    declare_rhoA = DeclareLaunchArgument(
        'rhoA', default_value='1000.0',
        description='Acceleration penalty weight')

    declare_rhoVt = DeclareLaunchArgument(
        'rhoVt', default_value='100000.0',
        description='Terminal velocity weight')

    declare_rhoThrust = DeclareLaunchArgument(
        'rhoThrust', default_value='10000.0',
        description='Thrust penalty weight')

    declare_rhoOmega = DeclareLaunchArgument(
        'rhoOmega', default_value='100000.0',
        description='Omega penalty weight')

    declare_rhoPerchingCollision = DeclareLaunchArgument(
        'rhoPerchingCollision', default_value='1000000.0',
        description='Perching collision penalty weight')

    declare_perching_px = DeclareLaunchArgument(
        'perching_px', default_value='0.5',
        description='Perching target position x')

    declare_perching_py = DeclareLaunchArgument(
        'perching_py', default_value='0.0',
        description='Perching target position y')

    declare_perching_pz = DeclareLaunchArgument(
        'perching_pz', default_value='2.0',
        description='Perching target position z')

    declare_perching_vx = DeclareLaunchArgument(
        'perching_vx', default_value='2.0',
        description='Perching target velocity x')

    declare_perching_vy = DeclareLaunchArgument(
        'perching_vy', default_value='0.0',
        description='Perching target velocity y')

    declare_perching_vz = DeclareLaunchArgument(
        'perching_vz', default_value='0.0',
        description='Perching target velocity z')

    declare_perching_axis_x = DeclareLaunchArgument(
        'perching_axis_x', default_value='0.0',
        description='Perching rotation axis x')

    declare_perching_axis_y = DeclareLaunchArgument(
        'perching_axis_y', default_value='1.0',
        description='Perching rotation axis y')

    declare_perching_axis_z = DeclareLaunchArgument(
        'perching_axis_z', default_value='0.0',
        description='Perching rotation axis z')

    declare_perching_theta = DeclareLaunchArgument(
        'perching_theta', default_value='-1.5708',
        description='Perching rotation angle')

    declare_replan = DeclareLaunchArgument(
        'replan', default_value='false',
        description='Enable replanning')

    declare_pause_debug = DeclareLaunchArgument(
        'pause_debug', default_value='false',
        description='Enable pause debug mode')

    # RViz2 node
    rviz_config = os.path.join(
        get_package_share_directory('planning'),
        'config',
        'rviz_sim.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    # Drone visualization node
    drone_vis_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        namespace='drone0',
        parameters=[{
            'robot_scale': 1.0,
            'mesh_resource': 'package://odom_visualization/meshes/f250.dae',
            'color/r': 0.0,
            'color/g': 1.0,
            'color/b': 0.0,
            'color/a': 1.0,
        }],
        remappings=[
            ('odom', 'odom'),
        ],
        output='screen')

    # Plate visualization node
    plate_vis_node = Node(
        package='odom_visualization',
        executable='odom_visualization_plate',
        name='odom_visualization_plate',
        namespace='drone0',
        parameters=[{
            'height': 1.2,
            'width': 1.6,
            'platform_r': 1.0,
        }],
        remappings=[
            ('odom', 'target_odom'),
        ],
        output='screen')

    # Planning node
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning',
        namespace='drone0',
        parameters=[{
            'plan_hz': LaunchConfiguration('plan_hz'),
            'K': LaunchConfiguration('K'),
            'vmax': LaunchConfiguration('vmax'),
            'amax': LaunchConfiguration('amax'),
            'thrust_max': LaunchConfiguration('thrust_max'),
            'thrust_min': LaunchConfiguration('thrust_min'),
            'omega_max': LaunchConfiguration('omega_max'),
            'omega_yaw_max': LaunchConfiguration('omega_yaw_max'),
            'robot_l': LaunchConfiguration('robot_l'),
            'robot_r': LaunchConfiguration('robot_r'),
            'platform_r': LaunchConfiguration('platform_r'),
            'v_plus': LaunchConfiguration('v_plus'),
            'rhoT': LaunchConfiguration('rhoT'),
            'rhoP': LaunchConfiguration('rhoP'),
            'rhoV': LaunchConfiguration('rhoV'),
            'rhoA': LaunchConfiguration('rhoA'),
            'rhoVt': LaunchConfiguration('rhoVt'),
            'rhoThrust': LaunchConfiguration('rhoThrust'),
            'rhoOmega': LaunchConfiguration('rhoOmega'),
            'rhoPerchingCollision': LaunchConfiguration('rhoPerchingCollision'),
            'perching_px': LaunchConfiguration('perching_px'),
            'perching_py': LaunchConfiguration('perching_py'),
            'perching_pz': LaunchConfiguration('perching_pz'),
            'perching_vx': LaunchConfiguration('perching_vx'),
            'perching_vy': LaunchConfiguration('perching_vy'),
            'perching_vz': LaunchConfiguration('perching_vz'),
            'perching_axis_x': LaunchConfiguration('perching_axis_x'),
            'perching_axis_y': LaunchConfiguration('perching_axis_y'),
            'perching_axis_z': LaunchConfiguration('perching_axis_z'),
            'perching_theta': LaunchConfiguration('perching_theta'),
            'replan': LaunchConfiguration('replan'),
            'pause_debug': LaunchConfiguration('pause_debug'),
        }],
        output='screen')

    return LaunchDescription([
        declare_plan_hz,
        declare_K,
        declare_vmax,
        declare_amax,
        declare_thrust_max,
        declare_thrust_min,
        declare_omega_max,
        declare_omega_yaw_max,
        declare_robot_l,
        declare_robot_r,
        declare_platform_r,
        declare_v_plus,
        declare_rhoT,
        declare_rhoP,
        declare_rhoV,
        declare_rhoA,
        declare_rhoVt,
        declare_rhoThrust,
        declare_rhoOmega,
        declare_rhoPerchingCollision,
        declare_perching_px,
        declare_perching_py,
        declare_perching_pz,
        declare_perching_vx,
        declare_perching_vy,
        declare_perching_vz,
        declare_perching_axis_x,
        declare_perching_axis_y,
        declare_perching_axis_z,
        declare_perching_theta,
        declare_replan,
        declare_pause_debug,
        rviz_node,
        drone_vis_node,
        plate_vis_node,
        planning_node,
    ])
