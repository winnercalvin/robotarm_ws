import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 두산 로봇 드라이버 패키지 위치 찾기
    dsr_pkg = get_package_share_directory('dsr_bringup2')
    
    # 2. 로봇 드라이버 실행 설정 (기존에 긴 명령어 치던 것)
    # ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real ...
    dsr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dsr_pkg, 'launch', 'dsr_bringup2_rviz.launch.py')
        ),
        launch_arguments={
            'mode': 'real',
            'host': '192.168.1.100', 
            'port': '12345',
            'model': 'm0609'
        }.items()
    )

    # 3. 우리가 만든 액션 서버들 (잡기, 붓기)
    grab_server_node = Node(
        package='tacobot',
        executable='scooper_grab',
        name='scooper_grab_node',
        output='screen'
    )

    pour_server_node = Node(
        package='tacobot',
        executable='scooper_pour',
        name='scooper_pour_node',
        output='screen'
    )

    # 4. 메인 컨트롤러 (지휘자)
    # [팁] 드라이버랑 서버가 켜질 시간을 주기 위해 5초 뒤에 실행되게 함
    controller_node = Node(
        package='tacobot',
        executable='task_controller',  # setup.py의 entry_points 이름과 같아야 함
        name='main_controller_node',
        output='screen'
    )
    
    delayed_controller = TimerAction(
        period=5.0,  # 5초 대기
        actions=[controller_node]
    )

    # 5. 모든 설정을 모아서 반환
    return LaunchDescription([
        dsr_launch,       # 로봇 켜기
        grab_server_node, # 잡기 서버 켜기
        pour_server_node, # 붓기 서버 켜기
        delayed_controller # 5초 뒤 컨트롤러 실행
    ])