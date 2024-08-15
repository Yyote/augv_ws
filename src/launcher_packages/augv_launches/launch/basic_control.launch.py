from launch import LaunchDescription
import launch_ros.actions as actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    DeclareLaunchArgument(
        name='full_robot_name',
		default_value='r',
    )

    DeclareLaunchArgument(
        name='robot_name',
		default_value='r',
    )

    DeclareLaunchArgument(
        name='robot_id',
		default_value='0',
    )

    DeclareLaunchArgument(
        name='robot_type',
		default_value='tracked',
    )

    # 2. Регуляторы для машинок
    if robot_types[i] == 'omni':
        ld.add_action(actions.Node(
            package="augv_regulators",
            executable="omni_regulator_node",
            namespace=f"{LaunchConfiguration("full_robot_name")}",
            parameters=[
                {'id': i + 1},
            ],
            # emulate_tty=True, output='screen'
        ))
    elif robot_types[i] == 'ackerman':
        ld.add_action(actions.Node(
            package="augv_regulators",
            executable="ackerman_regulator_node",
            namespace=f"robot{i + 1}",
            parameters=[
                {'id': i + 1},
            ],
            # emulate_tty=True, output='screen'
        ))
    elif robot_types[i] == 'tracked':
        ld.add_action(actions.Node(
            package="augv_regulators",
            executable="tracked_regulator_node",
            namespace=f"robot{i + 1}",
            parameters=[
                {'id': i + 1},
            ],
            # emulate_tty=True, output='screen'
        ))
    else:
        raise Exception(f'No such type: {robot_types[i]}')

    # 5. Автопилот для каждой машинки
    ld.add_action(
            actions.Node(
            package="autopilot_lite", 
            executable="autopilot_lite_node",
            namespace=f"robot{i + 1}",
            parameters=[
                {'id': i + 1},
            ],
            # emulate_tty=True,
            # output="screen"
        )
    )
    
    # 6. Robot Info для каждой машинки
    ld.add_action(
            actions.Node(
            package="robot_info", 
            executable="robot_info_node",
            namespace=f"robot{i + 1}",
            parameters=[{
                'id': i + 1,
                'platform_type': robot_types[i],
            }],
            # emulate_tty=True,
            # output="screen"
        )
    )

    return ld
