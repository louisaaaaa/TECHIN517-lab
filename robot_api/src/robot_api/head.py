#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import math
import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


LOOK_AT_ACTION_NAME = 'head_controller/point_head' 
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  
PAN_JOINT = 'head_pan_joint' 
TILT_JOINT = 'head_tilt_joint' 
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -1.571  # TODO: Minimum pan angle, in radians.
    MAX_PAN = 1.571  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -0.785  # TODO: Minimum tilt angle, in radians.
    MAX_TILT =  1.571 # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # rospy.init_node('head_control')
        self.look_at_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)
        self.pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for head control action servers...")
        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()
        rospy.loginfo("Head control action servers found.")


    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        # TODO: Send the goal
        # TODO: Wait for result
        goal = PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(1.0)
        self.look_at_client.send_goal(goal)
        self.look_at_client.wait_for_result()


    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        # TODO: Create a trajectory point
        # TODO: Set positions of the two joints in the trajectory point
        # TODO: Set time of the trajectory point

        # TODO: Create goal
        # TODO: Add joint names to the list
        # TODO: Add trajectory point created above to trajectory

        # TODO: Send the goal
        # TODO: Wait for result
        trajectory = JointTrajectory()
        trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.trajectory.points.append(point)
        self.pan_tilt_client.send_goal(goal)
        self.pan_tilt_client.wait_for_result()
