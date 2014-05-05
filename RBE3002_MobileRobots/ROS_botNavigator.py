#!/usr/bin/env python

from turtle import Turtle
import roslib
import rospy
import sys
import select
import termios
import tty
import utils
import time as pyTime
from time import sleep
from math import pi
from tf.transformations import euler_from_quaternion
from math import degrees
from math import sin
from math import cos
import math
import botDriver

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D 
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import GridCells
from nav_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg._Point import Point
from nav_msgs.msg._GridCells import GridCells

'''
Need:
mapserver running
subscribe to map topic
    - 1D list of values - obstacles are 100, clear is 0, unknown are -1
    - map info - resolution, cell size, timestamp
publish a gridcells message to rviz - list of points
rviz subscribes to gridcells messages in the map_msgs topic


Note: This code didn't get finished because I broke my hip and nearly failed
the class. But it's almost there!
'''
class Direction:
    up = 1
    left = 2
    right = 3
    down = 4
    invalid = 5

        
class Navigation(object):
    '''
    /move_base_simple_goal/goal
    /initial_pose
    ''' 
    def __init__(self, waypoints):
        #this will compute a path using A* and generate a series of waypoints
        for node in waypoints:
            self.driveToWaypoint(self, node)
        
    def driveToWaypoint(self, waypoint):
        '''
        driveToWaypoint handles navigation to waypoints
        
        Args:
            waypoint: A node containing x, y, and value variables
            
        Returns: Once the robot has reached every waypoint
        '''
        #waypoint is a dict containing x and y vals
        xo = odom.curVals['pX']
        yo = odom.curVals['pY']
        destX = waypoint.x
        destY = waypoint.y
        
        deltaX = destX - xo
        deltaY = destY - yo
        
        heading = math.tan(deltaY/deltaX)
        distance = math.sqrt(deltaX*deltaX+deltaY*deltaY)
        
        if (math.abs(odom.curVals['theta']-heading) > .05):
            botDriver.rotate(heading)
        if (distance > .05):
            driveStraight(distance)
        return
            
class Node(object):
    x = 0
    y = 0
    value = 0
    gscore = 0
    parent = None
    heuristic = 0
    fscore = gscore + heuristic
    point = Point(self.x, self.y, 0)
    
    def __init__(self, location, gscore, parent=None):
        self.heuristic = calcHeur(self, destinationNode)
        self.x = location.x
        self.y = location.y
        self.gscore = gscore
        if parent:
            self.parent = parent
        
        
    def isSame(self, node):
        if (self.x == node.x) and (self.y == node.y):
            return True
        else:
            return False
        
def calcHeur(start, end):
    '''
    Returns distance required to reach the goal from "start" without diagonal motion
    '''
    return (abs(goal.x-start.x)+abs(goal.y - start.y))
    
    
class PathFinder(object):
    '''
    PathFinder will search the map for a valid path to the goal.
    
    Args:
        Map = ROS map file to be traversed
        start = Initial location
        goal = Desired location
        
    Returns:
        Path = An array of coordinates representing a path as a series of waypoints.
        Resolution is, as of yet, unspecified.
    '''
    map = None
    closed = None
    open = None
    start = None
    goal = None
    
    
    def __init__(start, goal, resolution, map):
        self.map = map
        self.start = start
        self.goal = goal
        self.closed = GridCells()
        self.closed.cell_height = resolution
        self.closed.cell_width = resolution
        self.closed.header.frame_id = "map"
        
        self.open = GridCells()
        self.open.cell_height = resolution
        self.open.cell_width = resolution
        self.open.header.frame_id = "map"
        
        
        
    def astar(self):
        closedset = [] #nodes already evaluated
        openset = [self.start] #nodes to be evaluated - contains start
        came_from = self.map #navigated nodes
        
        start.gscore = 0
        start.heuristic = calcHeur(self.start, self.goal) + start.gscore
        
        lastNode = start
        
        while len(self.openset) > 0:
            distance = 100000
            
            for node in self.open:
                if node.fscore < distance:
                    current = node
                    distance = node.fscore
                if node.isSame(self.goal):
                    distance = 0
                    current = node
            if current.isSame(self.goal):
                return make_path(current)
            
            self.open.remove(current)
            self.closed.append(current)
            
            currentDirection = direction(current, last)
            
            upPoint = Point(current.x, current.y+resolution, 0)
            leftPoint = Point(current.x - resolution, current.y,0)
            rightPoint = Point(current.x+resolution, current.y, 0)
            downPoint = Point(current.x, current.y-resolution, 0)
            
            upNode = Node(upPoint, current.score + resolution, current)
            leftNode = Node(leftPoint, current.score + resolution, current)
            rightNode = Node(rightPoint, current.score + resolution, current)
            downNode = Node(downPoint, current.score + resolution, current)
            
            if currentDirection is Direction.up:
                upNode.heuristic -= 2*resolution
            elif currentDirection is Direction.right:
                rightNode.heuristic -= 2*resolution
            elif currentDirection is Direction.left:
                leftNode.heuristic -= 2*resolution
            elif currentDirection is Direction.down:
                downNode.heuristic -= 2*resolution
                
            possibleneighbors = [upNode, rightNode, leftNode, downNode]
            validneighbors = []
            
            #remove neighbors that aren't in c-space
            for neighbor in possibleneighbors:
                for tiles in botWorld:
                    if neighbor.isSame(item):
                        validneighbors.append(neighbor)
                        
             #remove neighbors that are already in the closedset           
            for neighbor in validneighbors:
                for closedTiles in closedset:
                    if neighbor.isSame(item):
                        validneighbors.remove(neighbor)
                        
            for neighbor in validneighbors:
                gscore = current.score + resolution
                fscore = gscore + neighbor.heuristic
                for object in openset:
                    if neighbor.isSame(object):
                        if neighbor.gscore < object.gscore:
                            openset.remove(object)
                        else:
                            validneighbors.remove(neighbor)
            openset = validneighbors + openset
            
            closed = []
            for node in closedset:
                closed.append(node.point)
                
            ClosedGC.cells = closed
            open = []
            for node in openset:
                open.append(node.point)
            OpenGC.cells = open
            
            openPub.publish(OpenGC)
            closedPub.publish(ClosedGC)
            
            last = current
            
    def direction(current, last):
        '''
        returns the last dirstion of motion for travel. Used to determine the value of the heuristic
        '''
        diffx = current.x-last.x
        diffy = current.y-last.y
        if diffy > 0:
            return Direction.up
        if diffy < 0:
            return Direction.down
        if diffx > 0:
            return Direction.right
        if diffx < 0:
            return Direction.left
        else:
            return Direction.invalid
                

        
    
        
        
class RobotOdometry(object):
    '''
    RobotOdometry handles odom data
    
    Whenever the robot publishes odom data, odomHandler is called. This will update data in
    the curVals dictionary. When another function wants to access odometry data, they can
    look at odom.curVals['pX'] (for example). "odom" is defined as a global variable to
    avoid passing it to every navigation function.
    '''
    curVals = {
               "pX":0,
               "pY":0,
               "theta":0,
               "vX":0,
               "vY":0,
               "vTheta":0
               }
    botPos = Point(curVals['pX'], curVals['pY'], 0)
    botNode = Node(0,botPos, 0,0)
    
    def __init__(self):
        odom_Subscriber = rospy.Subscriber("odom", Odometry, self.odomHandler)
    
    def odomHandler(self, data):
        '''
        This function updates data in the curVals dictionary to the values generated
        by the robot
        
        Args:
            data: Odom info sent by the robot
            
        Returns:
            N/a
        '''
        quat = data.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.curVals['pX'] = data.pose.pose.position.x
        self.curVals['pY'] = data.pose.pose.position.y
        self.curVals['theta'] = yaw
        self.curVals['vx'] = data.twist.twist.linear.x
        self.curVals['vy'] = data.twist.twist.linear.y
        self.curVals['vTheta'] = data.twist.twist.angular.z
        string = repr(self.curVals['pX']) + "," + repr(self.curVals['pY'])
        
        self.botPos = Point(curVals['pX'], curVals['pY'], 0)
        self.botNode = Node(0,botPos, 0,0)
        print string
        
class Mapper(object):
    '''
    Mapper takes the unprocessed map, converts it into c-space, 
    '''
    width = 0
    height = 0
    resolution = 0
    originx = 0
    originy = 0
    world = None
    
    def __init__(self):
        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.MapHandler)
        
    def MapHandler(self,map):
        global botWorld, resolution
        
        self.width = map.info.width
        self.height = map.info.height
        self.resolution = map.info.resolution
        resolution = self.resolution
        self.originX = map.info.origin.position.x
        self.originY = map.info.origin.position.y
        self.cSpaceConverter(map.data)
        
        cell = GridCells()
        cell.header.frame_id = "map"
        cell.cell_width = self.resolution
        celll.cell_height = self.resolution
        
        inaccessible = GridCells()
        inaccessible.header.frame_id = "map"
        inaccessible.cell_width = self.resolution
        inaccessible.cell_height = self.resolution
        
        freespace = []
        occupied = []
        
        for position, score in enumerate(self.world):
            point = Point()
            point.x = (index % self.width)*self.resolution+self.originX
            point.y = (index/self.width)*self.resolution+self.originY
            point.z = 0
            
            
            
            if score is 0 or -1:
                freespace.append(point)
            if score is 2:
                occupied.append(point)
                
        cell.cells = freespace
        
        inaccessible.cells = [occupied]
        
        inaccessiblePublisher.publish(inaccessible)
        
        botWorld = freespace
        
        
    def cSpaceConverter(self, world):
        '''
        This removes all inaccessible nodes from the world.
        '''
        worldlist = list(given)
        for position, score in enumerate(world):
            if score is 100:
                #I got this code from Lilian and Alex
                worldlist[current + 4*self.width] = 2
                worldlist[current + 3*1 + 3*self.width] = 2
                worldlist[current + 2*1 + 3*self.width] = 2
                worldlist[current + 2*1] = 2
                worldlist[current + 1*1] = 2
                worldlist[current + 0*1] = 2
                worldlist[current + (-1)*1] = 2
                worldlist[current + (-2)*1] = 2
                worldlist[current + (-3)*1] = 2
                worldlist[current + 3*1 + (-1)*self.width] = 2
                worldlist[current + 2*1 + (-1)*self.width] = 2
                worldlist[current + 1*1 + (-1)*self.width] = 2
                worldlist[current + 0*1 + (-1)*self.width] = 2
                worldlist[current + (-1)*1 + (-1)*self.width] = 2
                worldlist[current + (-2)*1 + (-1)*self.width] = 2
                worldlist[current + (-3)*1 + (-1)*self.width] = 2
                worldlist[current + 3*1 + (-2)*self.width] = 2
                worldlist[current + 2*1 + (-2)*self.width] = 2
                worldlist[current + 1*1 + (-2)*self.width] = 2
                worldlist[current + 0*1 + (-2)*self.width] = 2
                worldlist[current + (-1)*1 + (-2)*self.width] = 2
                worldlist[current + (-2)*1 + (-2)*self.width] = 2
                worldlist[current + (-3)*1 + (-2)*self.width] = 2
                worldlist[current + 3*1 + (-3)*self.width] = 2
                worldlist[current + 2*1 + (-3)*self.width] = 2
                worldlist[current + 1*1 + (-3)*self.width] = 2
                worldlist[current + 0*1 + (-3)*self.width] = 2
                worldlist[current + (-1)*1 + (-3)*self.width] = 2
                worldlist[current + (-2)*1 + (-3)*self.width] = 2
                worldlist[current + (-3)*1 + (-3)*self.width] = 2
                worldlist[current + (-4)*self.width] = 2
        self.world = tuple(worldlist)
        
    
def pubPath(path):
    trail = GridCells()
    trail.header.frame_id = "map"
    trail.cell_width = mapper.resolution
    trail.cell_height = mapper.resolution
    trail.cells = path
    
    trailPublisher.publish(trail)

def setupStart(pos):
    global startTheta
    start = Point()
    start.x = pos.pose.pose.position.x
    start.y = pos.pose.pose.position.y
    start.x = round((int)((start.x/resolution))*resolution,2) #rounds to even multiple of resolution
    start.y = round((int)((start.y/resolution))*resolution,2) #rounds to even multiple of resolution
    quat = pos.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    startTheta = yaw
    
def setupEnd(pos):
    global destinationNode, endTheta
    x = pos.pose.position.x
    y = pos.pose.position.y
    px = round(((int)(px/resolution))*resolution,2) #rounds to even multiple of resolution
    py = round(((int)(py/resolution))*resolution,2) #rounds to even multiple of resolution
    quat = pos.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    endTheta = yaw
    goal = Point(x,y,0)
    destinationNode = Node(goal, 0)

    startNode = 
        
    
def isAccessible(node):
    '''
    This checks whether a node is in the bot's c-space. If it isn't returns false
    '''
    for aNode in botWorld:
        if node.isSame(aNode):
            return True
    return False

if __name__=="__main__":
    global odom_Subscriber, vel_Publisher, odom, mapper, trailPublisher, inaccessiblePublisher
    rospy.init_node('rmowris_lab3')
    vel_Publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)  
    current = rospy.Publisher('/current', PointStamped)
    end = rospy.Publisher('/end', PointStamped)
    rospy.Subscriber('/goal_node'),PoseStamped, getEndPosition)
    
    
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, setupStart)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, setupEnd)
    
    
    inaccessiblePublisher = rospy.Publisher('/inaccessible', GridCells)
    trailPublisher = rospy.Publisher('/trail', GridCells)
    openPub = rospy.Publisher('/openCells', GridCells)
    closedPub = rospy.Publisher('closedCells', GridCells)
    
    odom = RobotOdometry()
    mapper = Mapper()
    bumper_Subscriber = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, bumpHandler)
    sleep(1) #wait for the first odometry data to publish
    while (1):
        if rospy.is_shutdown():
            break
        counter += 1