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
        # Controller variable on whether to use our proposed enhancements to increase the 
        #speeds of the robot or not.
        self.enhancements = 0
        # To use enhancements, set this to 1

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
        # Code to record the distance and path that the robot actually needed to drive over path
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
        self.distance = 0
        self.total_angle = 0
        self.lastpose = self.pose
        self.skipwaypoints = []
        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            if self.enhancements:
                ###    MY CODE TO REMOVE WAYPOINTS WTIHIN A STRAIGHT LINE   ###
                #While driving to each waypoint, if we have the "enhancements" switched on,
                #my code then iterates through subsequent waypoints, checking whether they have
                #the same orientation/angle as that from the current target waypoint to the next one
                #It goes as far as when it detects the angle is no longer the same i.e. path no longer straight
                #Then breaks, so it would know the number of waypoints after the current one that can be removed
                #as they are all part of the straight line. This makes the robot "jump" waypoints and hence drive
                #faster towards goal.
                #If the current waypoint is already listed as needed to be skipped, just ignore this waypoint and continue
                if waypointNumber in self.skipwaypoints:
                    print"skipping"
                    continue
                try:
                    #Temporary angle to store the orientaton of subsequent waypoints to check against
                    futureAngle = 0
                    i = waypointNumber
                    #Here, we form the initial orientation- the orientation of the current waypoint to the next one
                    #That we will be checking all subsequent waypoinnts againt to see if they are part of the same straight line
                    dX = path.waypoints[waypointNumber+1].coords[0] - cell.coords[0]
                    dY = path.waypoints[waypointNumber+1].coords[1] - cell.coords[1]
                    angleError = atan2(dY, dX)
                    while (futureAngle == 0):
                            try:
                                #For each subsequent waypoint, obtain the n to n + 1 orientation
                                dY = path.waypoints[i+1].coords[1] - path.waypoints[i].coords[1]
                                dX = path.waypoints[i+1].coords[0] - path.waypoints[i].coords[0]
                                #Compare the n to n + 1 orientation with the original orientation
                                #If it is different it means no longer straight line then loop will break
                                futureAngle = self.shortestAngularDistance(angleError, atan2(dY, dX))
                                #Iterator to store the number of subsequent waypoints that are within the straight line
                                i = i + 1
                            except:
                                break
                    for i in range(waypointNumber+1,i-1):
                        #After loop breaks, mark all the waypoints we've identified as within the straight line to skip them
                        self.skipwaypoints.append(i)
                except:
                    continue

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
 
