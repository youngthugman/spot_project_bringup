from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=['/home/max/spot_ws/src/spot_project_bringup/config/urg_node_serial.yaml'],
    )

    static_tf_body_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_laser_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'laser'],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/home/max/spot_ws/src/spot_project_bringup/config/slam_spot.yaml'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        aarguments=['-d', '/home/max/.rviz2/spotrviz.rviz'],
    )

    ld.add_action(urg_node)
    ld.add_action(static_tf_body_to_laser)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz)

    return ld


    