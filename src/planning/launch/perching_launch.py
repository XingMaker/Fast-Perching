from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    planning_pkg = get_package_share_directory('planning')

    # Declare launch arguments
    rviz_config = os.path.join(planning_pkg, 'config', 'rviz_sim.rviz')

    return LaunchDescription([
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # Odom visualization
        Node(
            package='odom_visualization',
            executable='odom_visualization',
            name='odom_visualization',
            namespace='drone0',
            output='screen',
            parameters=[{'robot_scale': 1.0}],
            remappings=[('odom', 'odom')]
        ),

        # Odom visualization plate
        Node(
            package='odom_visualization',
            executable='odom_visualization_plate',
            name='odom_visualization_plate',
            namespace='drone0',
            output='screen',
            parameters=[
                {'height': 1.2},
                {'width': 1.6},
                {'platform_r': 1.0}
            ],
            remappings=[('odom', 'target_odom')]
        ),

        # Planning node
        Node(
            package='planning',
            executable='planning_node',
            name='planning',
            namespace='drone0',
            output='screen',
            parameters=[
                {'plan_hz': 5},
                {'K': 16},
                {'vmax': 15.0},
                {'amax': 6.0},
                {'thrust_max': 17.0},
                {'thrust_min': 5.0},
                {'omega_max': 3.0},
                {'omega_yaw_max': 0.5},
                {'robot_l': 0.02},
                {'robot_r': 0.13},
                {'platform_r': 1.0},
                {'v_plus': 0.3},
                {'rhoT': 100000.0},
                {'rhoP': 10000000.0},
                {'rhoV': 1000.0},
                {'rhoA': 1000.0},
                {'rhoVt': 100000.0},
                {'rhoThrust': 10000.0},
                {'rhoOmega': 100000.0},
                {'rhoPerchingCollision': 1000000.0},
                # Drone initial state
                {'drone_init_px': 0.0},
                {'drone_init_py': 0.0},
                {'drone_init_pz': 2.0},
                {'drone_init_vx': 0.0},
                {'drone_init_vy': 0.0},
                {'drone_init_vz': 0.0},
                # Target platform initial state
                {'perching_px': 0.5},
                {'perching_py': 0.0},
                {'perching_pz': 2.0},
                {'perching_vx': 10.0},
                {'perching_vy': 0.0},
                {'perching_vz': 0.0},
                {'perching_axis_x': 0.0},
                {'perching_axis_y': 1.0},
                {'perching_axis_z': 0.0},
                {'perching_theta': -1.5708},
                {'replan': False},
                {'pause_debug': False}
            ],
            remappings=[
                ('odom', 'odom'),
                ('heartbeat', 'heartbeat'),
                ('trajectory', 'trajectory'),
                ('replanState', 'replanState'),
                ('triger', '/triger'),
                ('land_triger', '/land_triger'),
                ('target', '/target/odom')
            ]
        )
    ])
