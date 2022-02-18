from aiohttp import ClientResponse
from manipulation_msgs.srv import CylinderFit
from cylinder_fitting import fit
from geometry_msgs.msg import *
import rospy

def fit_cylinder(req):
    rospy.loginfo("Receiving direction: ")
    rospy.loginfo(req.init_direction)
    rospy.loginfo("Returning: ")
    
    direction = Vector3()
    direction.x = 0
    direction.y = 0
    direction.z = 0

    axis_point = Point32()
    axis_point.x =0
    axis_point.y =0
    axis_point.z =0

    c_point = Point32()
    c_point.x =0
    c_point.y =0
    c_point.z =0
    
    return [direction,axis_point,0.01,c_point,0.05]


def cylinder_fitting_server():
    rospy.init_node('cylinder_fitting_server')
    service = rospy.Service('fit_cylinder_with_points', CylinderFit, fit_cylinder)
    rospy.loginfo("Ready fit cylinders.")
    rospy.spin()


if __name__ == "__main__":
    cylinder_fitting_server()