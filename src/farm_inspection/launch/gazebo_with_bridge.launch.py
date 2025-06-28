from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Caminho relativo ao pacote
    pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    
    # Caminho do arquivo SDF
    sdf_path = os.path.join(pkg_share, 'models', 'building_robot.sdf')

    # Caminho do executável lidar_node
    lidar_node_path = os.path.join(pkg_share, 'scripts', 'build', 'lidar_node')
    lidar_node_cwd = os.path.join(pkg_share, 'scripts', 'build')

    # Processo para iniciar o Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', sdf_path, '--verbose'],
        output='screen'
    )

    # Processo para rodar lidar_node
    lidar_node = ExecuteProcess(
        cmd=[lidar_node_path],
        cwd=lidar_node_cwd,
        output='screen'
    )

    # Bridge cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    # Bridge imu
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
        output='screen'
    )

    # Bridge lidar_distance
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar_distance@std_msgs/msg/Float64@ignition.msgs.Double'],
        output='screen'
    )

    bridge_camera_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        lidar_node,
        bridge_cmd_vel,
        bridge_imu,
        bridge_lidar,
        bridge_camera_image,
    ])

