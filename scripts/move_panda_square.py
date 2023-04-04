#!/usr/bin/env python3

import copy
from math import pi
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
import time
from ros_cartesian_space_movements.msg import square_size

def callback(data):
    #publish initialised
    try:
        #define and print starting config
        print('-----------------------------------------------------------------')
        print('Move Panda - Received Square Size, s = %s' % data.size)
        print('-----------------------------------------------------------------')
        print('Move Panda - Going To Start Configuration')
        print('-----------------------------------------------------------------')
        #calculate the starting config
        start_configuration = [0, -pi / 4, 0, -pi / 2, 0, pi / 3, 0]
        
        #move robot to starting config
        group.go(start_configuration, wait=True)
        #stopping any residual movement
        group.stop()
        
        #print planning info
        print('-----------------------------------------------------------------')
        print('Move Panda - Planning Motion Trajectory')
        print('-----------------------------------------------------------------')
        #init array of points to move robot
        waypoints = []
        
        #get current position of the robot
        wpose = group.get_current_pose().pose
        
        #calculate waypoints of trajectory        
        wpose.position.y += data.size
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += data.size
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= data.size
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= data.size
        waypoints.append(copy.deepcopy(wpose))
        
        #calculate trajectory plan using waypoints
        (plan, fraction) = group.compute_cartesian_path(
            waypoints,
            0.01,
            0.00)
        
        #init message for trajectory planning
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        
        #publish message
        print('-----------------------------------------------------------------')
        print('Move Panda - Showing Planned Trajectory')
        print('-----------------------------------------------------------------')
        display_trajectory_publisher.publish(display_trajectory)
        
        #sleep for 5 seconds
        time.sleep(5)
        
        #executing trajectory
        print('-----------------------------------------------------------------')
        print('Move Panda - Executing Planned Trajectory')
        print('-----------------------------------------------------------------')
        group.execute(plan, wait=True)
        
    except rospy.ServiceException as e:
        print('-----------------------------------------------------------------')
        print("Service call failed: %s" % e)
        print('-----------------------------------------------------------------')
        
def move_panda_square():
    #init commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    #init new node
    rospy.init_node('move_panda_square', anonymous=True)
    
    #wait for service & subscribe to square_size and send data to callback
    print('-----------------------------------------------------------------')
    print('Move Panda - Waiting For Square Size')
    print('-----------------------------------------------------------------')
    rospy.Subscriber('size', square_size, callback)
    #spin to prevent from dying
    rospy.spin()
    
if __name__ == "__main__":
    #init robot commander
    robot = moveit_commander.RobotCommander()

    #init scene planning interface
    scene = moveit_commander.PlanningSceneInterface()

    #init move group commander
    group = moveit_commander.MoveGroupCommander('panda_arm')

    #init display trajectory publisher
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=0)
    move_panda_square()
