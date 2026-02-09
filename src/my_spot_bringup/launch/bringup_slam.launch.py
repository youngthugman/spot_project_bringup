from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = LaunchDescription()
# Runs spot driver launch py to execute driver nodes
    spot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('spot_driver'),
                '/launch/spot_driver.launch.py'
            ]
        ),
        launch_arguments={
            'config_file': '/home/max/spot_ws/src/spot_ros2/spot_driver/config/spot_ros_example.yaml'
        }.items(),
    )

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=['/home/max/spot_ws/src/spot_project_bringup/config/urg_node_serial.yaml'],
    )
    #Puts the urdf into text for the robot_description to run off.
    my_spot_lidar_mount_path = "/home/max/spot_ws/src/spot_project_bringup/src/my_spot_description/urdf/spot_lidar_mount.urdf"
    with open(my_spot_lidar_mount_path,"r" ) as text:
        robot_description = text.read()
    robot_state_publisher = Node(
        package ="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher" ,
        output="screen" ,
        parameters=[{"robot_description": robot_description}],

    )

    #Need to change frame id: laser to use this not laser_link.
    #static_tf_body_to_laser = Node(
        #package='tf2_ros',
        #executable='static_transform_publisher',
        #name='body_to_laser_static_tf',
        #output='screen',
        #arguments=['0', '0', '0', '0', '0', '0', 'body', 'laser'],
    #)

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
        arguments=['-d', '/home/max/.rviz2/spotrviz.rviz'],
    )
    ld.add_action(spot_driver_launch)
    ld.add_action(urg_node)
    ld.add_action(robot_state_publisher)
    #ld.add_action(static_tf_body_to_laser)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz)

    return ld


    