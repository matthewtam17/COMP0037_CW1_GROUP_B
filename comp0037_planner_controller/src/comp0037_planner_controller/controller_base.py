#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time
import math

# This is the base class of the controller which moves the robot to its goal.
#This is the low-level controller.

class ControllerBase(object):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)
        # Star time variable to help record the time needed for a robot to drive a path
        self.start_time = 0
        self.distance = 0
        self.total_angle = 0
        self.futureAngle = 0

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.
        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()
        self.lastpose = self.pose
        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates.
        self.occupancyGrid = occupancyGrid
        
        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rate = rospy.Rate(10)

    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose
        self.distance =  self.distance + sqrt((pose.x-self.lastpose.x)**2 + (pose.y-self.lastpose.y)**2)
        self.total_angle = self.total_angle + abs(self.shortestAngularDistance(pose.theta,self.lastpose.theta))
        self.lastpose = pose

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose
    
    def shortestAngularDistance(self,fromAngle,toAngle):
        return NotImplementedError()

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer):
        self.plannerDrawer = plannerDrawer
        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)')
        self.start_time = rospy.get_time()
        self.distance = 0
        self.total_angle = 0
        self.lastpose = self.pose
        self.skipwaypoints = []
        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]

            #if waypointNumber in self.skipwaypoints:
            #    print skipping
            #    continue
            #futureAngle = 0
            #i = waypointNumber
            #dX = cell.coords[0] - self.pose.x
            #dY = cell.coords[1] - self.pose.y
            #angleError = atan2(dY, dX)
            #while (futureAngle == 0):
            #    dY = path.waypoints[i+1].coords[1] - path.waypoints[i].coords[1]
            #    dX = path.waypoints[i+1].coords[0] - path.waypoints[i].coords[0]
            #    futureAngle = self.shortestAngularDistance(angleError, atan2(dY, dX))
            #    i = i + 1
            #for i in range(waypointNumber+1,i-1):
            #    self.skipwaypoints.append(i)
            #    

            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            self.driveToWaypoint(waypoint)
            # Handle ^C
            if rospy.is_shutdown() is True:
                break

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')
        
        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)
        
        print('Total Elapsed Time to Drive to Goal: ' + str(rospy.get_time() - self.start_time))
        print('Total Distance Taken By Robot to Drive to Goal: ' + str(self.distance))
        print('Total Angle Turned By Robot to Drive to Goal: ' + str(self.total_angle))
 
