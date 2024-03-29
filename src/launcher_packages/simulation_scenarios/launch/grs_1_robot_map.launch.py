from launch import LaunchDescription
import launch_ros.actions as actions

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_types = [
        'omni',
        'omni',
        'omni',
        'omni',
        'omni',
    ]
    
    robot_coords = [
        1, 2, 
        2, 2, 
        3, 2, 
        4, 2, 
        5, 2, 
    ]
    
    
    map_name = 'map_test.tmx'
    
    # 1. Симулятор
    
    ld.add_action(actions.Node(
        package="gr_kinematic_sim",
        executable="sim",
        name="test_scenario_sim",
        parameters=[
            {'robot_types' : robot_types},
            {'robot_coords' : robot_coords},
            {'map_name' : map_name},
        ],
        # emulate_tty=True, output='screen'
    ))
    
    for i in range(len(robot_types)):
        # 2. Регуляторы для машинок
        if robot_types[i] == 'omni':
            ld.add_action(actions.Node(
                package="augv_regulators",
                executable="omni_regulator_node",
                namespace=f"robot{i + 1}",
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

        # 3. Потенциальные поля для каждой машинки
        ld.add_action(
                actions.Node(
                package="potential_planner", 
                executable="potential_fields",
                namespace=f"robot{i + 1}",
                parameters=[
                    {'id': i + 1},
                ],
                # emulate_tty=True,
                # output="screen"
            )
        )
        
        # 4. A* для каждой машинки
        ld.add_action(
                actions.Node(
                package="astar_planner", 
                executable="astar",
                namespace=f"robot{i + 1}",
                parameters=[
                    {'id': i + 1},
                ],
                # emulate_tty=True,
                # output="screen"
            )
        )

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

    return ld
