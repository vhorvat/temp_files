#!/usr/bin/env python3


import rospy
from ur5_cpp_controller.srv import GoToPose
from geometry_msgs.msg import Pose

def call_go_to_pose_goal(pose_goal):
    rospy.wait_for_service('/go_to_pose_goal')
    try:
        go_to_pose = rospy.ServiceProxy('/go_to_pose_goal', GoToPose)
        response = go_to_pose(pose_goal)  
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('ur5_controller_client')
    
    pose_goal = Pose()
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.7
    pose_goal.position.z = 0.5
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 1.0


    success = call_go_to_pose_goal(pose_goal)
    if success:
        print("Pose goal reached.")
    else:
        print("Failed to reach pose goal.")
