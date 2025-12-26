from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 👉 여기에 실제 네 패키지 이름 적기
    pkg_name = 'dsr_project'

    return LaunchDescription([
        # 1) Doma pick 액션 서버
        Node(
            package=pkg_name,
            # executable='doma_pick_stop_server',   # setup.py의 console_scripts 이름과 같아야 함
            executable='Doma_a_pick_server',   # setup.py의 console_scripts 이름과 같아야 함
            name='Doma_a_pick_server',
            output='screen'
        ),

        # 2) Erasing 액션 서버
        Node(
            package=pkg_name,
            executable='erasing_stop_server',
            name='erasing_action_server',
            output='screen'
        ),

        # 3) Brushing 액션 서버
        Node(
            package=pkg_name,
            executable='brushing_stop_server',
            name='brushing_action_server',
            output='screen'
        ),

        # 4) Oiling 액션 서버
        Node(
            package=pkg_name,
            executable='oiling_stop_server',
            name='oiling_action_server',
            output='screen'
        ),

        # 5) Doma place 액션 서버
        Node(
            package=pkg_name,
            # executable='doma_place_stop_server',
            executable='Doma_a_place_server',
            name='Doma_a_place_server',
            output='screen'
        ),

        # 6) 메인 코디네이터 노드
        #    (doma_pick -> erasing -> brushing -> oiling -> doma_place 순서를 안에서 실행)
        Node(
            package=pkg_name,
            executable='main_node_gem',  # 메인 노드 executable 이름
            name='main_node',
            output='screen'
        ),

        # 필요하면 여기서 GUI/모니터링 노드도 추가 가능
        # Node(
        #     package=pkg_name,
        #     executable='your_gui_node',
        #     name='gui_monitor',
        #     output='screen'
        # ),
    ])
