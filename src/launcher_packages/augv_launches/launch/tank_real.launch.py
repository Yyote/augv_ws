from launch import LaunchDescription
import launch_ros.actions as actions

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_types = [
        'tracked',
    ]

    robot_id = 0
    
    for i in range(len(robot_types)):
        # 2. Регуляторы для машинок
        if robot_types[i] == 'tracked':
            ld.add_action(actions.Node(
                package="augv_regulators",
                executable="tracked_regulator_node",
                namespace=f"robot{robot_id}",
                parameters=[
                    {'id': robot_id},
                ],
                remappings=[
                    (f'/robot{i}/odom', f'/robot{i}/zed_node/odom'),
                    (f'/robot{i}/pose', f'/robot{i}/zed_node/pose')
                ]
                # emulate_tty=True, output='screen'
            ))
        else:
            raise Exception(f'No such type: {robot_types[i]}')

        # 3. Потенциальные поля для каждой машинки
        ld.add_action(
                actions.Node(
                package="potential_planner", 
                executable="potential_fields",
                namespace=f"robot{robot_id}",
                parameters=[
                    {'id': robot_id},
                    {'k1': 0.04},
                    {'k2': 0.08},
                    {'k3': 0.2},
                    {'rmax1': 3},
                    {'rmax2': 1},
                    {'rmax3': 0.5},
                ],
                # emulate_tty=True,
                # output="screen"\
                remappings=[
                    (f'/robot{i}/odom', f'/robot{i}/zed_node/odom'),
                    (f'/robot{i}/pose', f'/robot{i}/zed_node/pose')
                ]
            )
        )
        
        # 4. A* для каждой машинки
        ld.add_action(
                actions.Node(
                package="astar_planner", 
                executable="astar",
                namespace=f"robot{robot_id}",
                parameters=[
                    {'id': robot_id},
                ],
                # emulate_tty=True,
                # output="screen"
                remappings=[
                    (f'/robot{i}/odom', f'/robot{i}/zed_node/odom'),
                    (f'/robot{i}/pose', f'/robot{i}/zed_node/pose')
                ]
            )
        )

        # 5. Автопилот для каждой машинки
        ld.add_action(
                actions.Node(
                package="autopilot_lite", 
                executable="autopilot_lite_node",
                namespace=f"robot{robot_id}",
                parameters=[
                    {'id': robot_id},
                ],
                # emulate_tty=True,
                # output="screen"
                remappings=[
                    (f'/robot{i}/odom', f'/robot{i}/zed_node/odom'),
                    (f'/robot{i}/pose', f'/robot{i}/zed_node/pose')
                ]
            )
        )

    return ld
