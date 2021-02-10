std_msgs/Header header
int32 mode
geometry_msgs/Vector3 base_pose
geometry_msgs/Twist base_twist
sensor_msgs/JointState arm_state
float64 object_position
float64 object_velocity
bool in_contact

int32 FIXED_BASE = 0
int32 MOVING_BASE = 1

int32 FIXED_BASE_SIZE = 21
int32 MOVING_BASE_SIZE = 27