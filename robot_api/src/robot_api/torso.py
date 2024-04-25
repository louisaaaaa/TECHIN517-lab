#!/usr/bin/env python

import rospy
import actionlib
import geometry_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


ACTION_NAME = '/torso_controller/follow_joint_trajectory'
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):

        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """

        if height < self.MIN_HEIGHT or height > self.MAX_HEIGHT:
            return

        # Set position of trajectory point
        # Set time of trajectory point
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        # Create goal
        goal = FollowJointTrajectoryGoal()
        # Add joint name to list
        goal.trajectory.joint_names = [JOINT_NAME]
        # Add the trajectory point created above to trajectory
        goal.trajectory.points = [point]


        # Send goal
        self.client.send_goal(goal)
        # Wait for result
        self.client.wait_for_result()

