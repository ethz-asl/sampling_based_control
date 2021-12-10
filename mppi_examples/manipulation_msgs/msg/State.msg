std_msgs/Header header
#geometry_msgs/Vector3 base_pose
#geometry_msgs/Twist base_twist
#geometry_msgs/Vector3 base_effort
sensor_msgs/JointState arm_state   # size 9*2 = 18
float64[] object_position          # size 3 + 4 = 7
float64[] object_velocity          # size 7 
bool in_contact
#float64 tank_state

int64 SIZE = 33   # 18 + 3 + 4 + 7 + 1