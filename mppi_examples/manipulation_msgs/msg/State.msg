std_msgs/Header header
sensor_msgs/JointState arm_state        # size 9*2 = 18
sensor_msgs/JointState object_state     # size 7*2 = 14
bool in_contact

int64 SIZE = 33   # 18 + 14 + 1