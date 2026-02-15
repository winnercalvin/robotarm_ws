import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dsr_pkg = get_package_share_directory('dsr_bringup2')
    
    dsr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dsr_pkg, 'launch', 'dsr_bringup2_rviz.launch.py')
        ),
        launch_arguments={
            'mode': 'real', 'host': '192.168.1.100', 'port': '12345', 'model': 'm0609'
        }.items()
    )

    # 1. 변수 이름을 'action_server_node'로 수정하여 일치시킴
    action_server_node = Node(
        package='tacobot',
        executable='action_server', # setup.py의 entry_points 이름과 같아야 함
        name='action_server_node',
        output='screen'
    )

    controller_node = Node(
        package='tacobot',
        executable='main_controller',
        name='main_controller_node',
        output='screen'
    )
    
    delayed_controller = TimerAction(period=5.0, actions=[controller_node])

    return LaunchDescription([
        dsr_launch,
        action_server_node, # 이제 위에서 정의한 변수명과 일치하므로 에러가 나지 않습니다!
        delayed_controller
    ])