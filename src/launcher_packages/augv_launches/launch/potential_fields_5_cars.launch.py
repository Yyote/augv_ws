from launch import LaunchDescription
import launch_ros
# from potential_fields_params import PotentialParams

def generate_launch_description():
    ld = LaunchDescription()
    for i in range(1, 8):
        # if i == 1:
        #     continue
        ld.add_action(
            launch_ros.actions.Node(
                package="potential_planner", 
                executable="potential_fields",
                namespace=f"robot{i}",
                parameters=[
                    {'id': i},
                ],
                emulate_tty=True,
                output="screen"
            )
        )
    return ld