byte POSE=0
byte VEL=1

byte ctr_type
drone_msgs/DronePose pose

uint8 completion_policy
uint8 POLICY_NO_POLICY = 0
uint8 POLICY_DISTANCE_REACHED = 1
uint8 POLICY_POSITION_REACHED = 2
uint8 POLICY_TIMER = 3

float32 policy_timer_time
float32 policy_distance_threshold
float32 policy_orientation_threshold