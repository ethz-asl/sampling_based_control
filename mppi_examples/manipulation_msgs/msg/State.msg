std_msgs/Header header
#geometry_msgs/Vector3 base_pose
#geometry_msgs/Twist base_twist
#geometry_msgs/Vector3 base_effort
sensor_msgs/JointState arm_state        # size 9*2 = 18
sensor_msgs/JointState object_state     # size 7*2 = 14
bool in_contact
#float64 tank_state

int64 SIZE = 33   # 18 + 14 + 1