# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
# from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():
#     # --- Launch Arguments ---
#     use_sim_time = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true'
#     )

#     # --- Package Paths ---
#     pkg_gazebo_simulator_share = get_package_share_directory('gazebo_simulator')
#     pkg_gazebo_ros_share = get_package_share_directory('gazebo_ros')

#     # --- File Paths ---
#     robot_xacro_file = PathJoinSubstitution([
#         pkg_gazebo_simulator_share,
#         'urdf',
#         'diffbot.urdf.xacro'
#     ])

#     rviz_config_file = os.path.join(
#         pkg_gazebo_simulator_share,
#         'rviz',
#         'skidbot.rviz'
#     )

#     world_file = os.path.join(
#         pkg_gazebo_simulator_share,
#         'worlds',
#         'simulator.world'
#     )

#     # --- XACRO to URDF Processing ---
#     # Command()를 이용하여 런타임에 xacro를 URDF로 변환
#     robot_description_content = Command(['xacro ', robot_xacro_file])
#     robot_description = {'robot_description': robot_description_content}

#     # --- Nodes and Launch Actions ---

#     # 1. Gazebo 실행
#     start_gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros_share, 'launch', 'gazebo.launch.py')
#         ),
#         launch_arguments={
#             'world': world_file,
#             'paused': 'false',
#             'use_sim_time': LaunchConfiguration('use_sim_time'),
#             'gui': 'true',
#             'headless': 'false',
#             'debug': 'false'
#         }.items()
#     )

#     # 2. Robot State Publisher
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
#     )

#     # 3. Joint State Publisher
#     joint_state_publisher_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
#     )

#     # 4. Spawn Entity
#     spawn_entity_node = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         output='screen',
#         arguments=['-topic', 'robot_description',
#                    '-entity', 'diffbot',
#                    '-x', '0.0', '-y', '0.0', '-z', '0.5'],
#         parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
#     )

#     # 5. RViz2 실행 (지연 시작)
#     rviz_start_action = TimerAction(
#         period=3.0,
#         actions=[
#             Node(
#                 package='rviz2',
#                 executable='rviz2',
#                 name='rviz2',
#                 arguments=['-d', rviz_config_file],
#                 parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#                 output='screen'
#             )
#         ]
#     )

#     # --- Return LaunchDescription ---
#     return LaunchDescription([
#         use_sim_time,
#         start_gazebo,
#         robot_state_publisher_node,
#         joint_state_publisher_node,
#         spawn_entity_node,
#         rviz_start_action,
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # --- Launch Arguments ---
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Package Paths ---
    pkg_gazebo_simulator_share = get_package_share_directory('gazebo_simulator')
    pkg_gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # --- File Paths ---
    robot_xacro_file = PathJoinSubstitution([
        pkg_gazebo_simulator_share,
        'urdf',
        'diffbot.urdf.xacro'
    ])
    rviz_config_file = os.path.join(
        pkg_gazebo_simulator_share,
        'rviz',
        'skidbot.rviz'
    )
    world_file = os.path.join(
        pkg_gazebo_simulator_share,
        'worlds',
        'simulator.world'
    )

    # --- XACRO to URDF Processing ---
    robot_description_content = Command(['xacro ', robot_xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # --- Nodes and Launch Actions ---

    # 1. Gazebo 실행 (Headless 모드)
    # gui를 'false'로, headless를 'true'로 설정하여 서버만 실행합니다.
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': 'false',      # Gazebo GUI 끄기
            'headless': 'true',  # Headless 모드 켜기
            'paused': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'debug': 'false'
        }.items()
    )

    # 2. Robot State Publisher (필수)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 3. Joint State Publisher (필수)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 4. Spawn Entity (필수)
    # Headless로 실행된 Gazebo 서버에 로봇 모델을 스폰합니다.
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'diffbot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 5. RViz2 실행 (필수)
    # 모든 센서 데이터와 로봇 상태를 여기서 시각화합니다.
    rviz_start_action = TimerAction(
        period=3.0, # Gazebo 서버가 안정적으로 실행될 시간을 줍니다.
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            )
        ]
    )

    # --- Return LaunchDescription ---
    return LaunchDescription([
        use_sim_time,
        start_gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        rviz_start_action,
    ])