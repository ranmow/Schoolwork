#!/usr/bin/env python
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


from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D 
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent

'''
ToDo:
    - Create publishTist(u,w), and z 
    - spinWheels(u1, u2, time)
'''

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
        #print string

def gradualAcceleration(distance, speed):
    '''
    This uses trapezoidal control to gradually accelerate. Yeah, it's
    kinda dumb and hard-coded, but it works and doesn't over-shoot!
    
    Params:
        Distance: Distance, in meters, to traverse
        Speed: Meters/second
        
    Returns:
        Exits once the distance has been traversed
    '''
    time = distance/speed
    speedIncrement = float(speed/20)
    actualSpeed = 0
    print speedIncrement
    sleepTime = float(time/140)
    for x in range(0,140):
        if (x<20):
            actualSpeed += speedIncrement
        if (x>=20) and (x<120):
            actualSpeed = speed
        if (x>119):
            actualSpeed -= speedIncrement
        print actualSpeed
        publishTwist(actualSpeed, 0)
        sleep(sleepTime)
    


def publishTwist(u, w):
    '''
    publishTwist sends linear and angular velocities to the bot
    
    This will take in two vectors representing velocities, u (linear) and
    w (angular). Default arguments are 0
    
    Args:
        u: A 2x1 vector containing x, y velocities
        w: radians/second representing rotation about Z. Positive = ccw
    '''
    twist = Twist()
    twist.linear.x = u
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = w
    #print twist #for debugging
    vel_Publisher.publish(twist)
    return
    
def spinWheels(u1, u2, time):
    '''
    spinWheels spins the wheels at specified velocities for (time) seconds
    
    This will take these velocities, compute translational and rotational
    velocities according to them, and send them to the robot using 
    publishTwist
    
    Args:
        u1: Velocity of the left wheel
        u2: Velocity of the right wheel
        time: number of seconds to rotate wheel(s) for
        
    Returns:
        None
    '''
    r = .035
    xvel = (r/2)*(u1+u2)
    yvel = 0
    u = xvel
    w = (.035/.23)*(u1-u2)
    startTime = pyTime.time()
    while ((pyTime.time() - startTime) < time):
        publishTwist(u,w)
        sleep(.1)
    return
    
def stopRobot():
    '''
    This sends a command to stop both motors on the kobuki
    '''
    publishTwist(0, 0)
    
def driveStraight(speed, distance):
    '''
    Drives the robot straight for a specified distance at a specified speed
    
    Args:
        Speed at which the robot will drive (find out units)
        Distance (in meters) that the robot will drive
#     '''
    initialX = odom.curVals['pX']
    initialY = odom.curVals['pY']
    initDist = math.sqrt(initialX*initialX + initialY*initialY)
    desiredPos = initDist + distance
#     print initialPos
#     print desiredPos
    while (1):
#         publishTwist(speed, 0)
        spinWheels(speed, speed, .5)
        curX = odom.curVals['pX']
        curY = odom.curVals['pY']
        deltaX = curX - initialX
        deltaY = curY - initialY
        distTravelled = math.sqrt(deltaX*deltaX+deltaY*deltaY)
#         print currentPos
        if (math.fabs(distance - distTravelled) < .1):
            stopRobot()
            return
    
def rotate(desiredAngle):
    '''
    This will drive the robot to an angle, relative to its current orientation
    0 degrees will keep the robot fixed.
    
    This code was inspired by kobuki_testsuite at:
    https://github.com/yujinrobot/kobuki/tree/hydro-devel/kobuki_testsuite/src/kobuki_testsuite
    
    Positive angles = counter-clockwise motion
    '''
    u = 0
    tolerance = .1
    zvel = .8
    initialAngle = odom.curVals['theta']
   # print initialAngle
   # print desiredAngle
    desiredAngle = utils.wrap_to_pi(desiredAngle + initialAngle)
    while not angleCloseEnough(desiredAngle):
        currentAngle = odom.curVals['theta']
        speed = zvel * utils.sign(utils.wrap_to_pi(desiredAngle - currentAngle))
        #print currentAngle
        publishTwist(0, speed)
    stopRobot()
    return
   
def angleCloseEnough(desiredTheta):
    '''
    Returns true if the robot's theta is close to the given angle (with some
    smart handling of pi wraparound)
    '''
    theta = odom.curVals['theta']
    if math.fabs(utils.wrap_to_pi(desiredTheta - theta)) < math.radians(3.0):
        return True
    else:
        return False
     
    
def driveArc(radius, speed, angle):
    '''
    This drives the robot in an arc
    
    Args:
        radius: radius of the arc, as measured to the center of the robot
        speed: speed of the motion, in m/s
        angle: angle of the arc
        
    Returns:
        When the robot reaches the end position, this exits
    '''
    r = .035
    base = .115
    #radii for both wheels. These are r1 and r2
    initialxPos = odom.curVals['pX']
    initialyPos = odom.curVals['pY']
    initialTheta = odom.curVals['theta']
    rOne = radius - (base/2)
    rTwo = radius + (base/2)
    d1 = rOne*angle
    d2 = rTwo*angle
    d = radius*angle
    omega = speed/radius
    desiredTheta = angle + initialTheta
    desiredyPos = initialyPos + d*sin(initialTheta)
    while not angleCloseEnough(desiredTheta):
#         publishTwist(speed, 0)
        publishTwist(speed, omega)
    #print "Done!"
    stopRobot()
    return
    
    
def executeTrajectory():
    '''
    This executes the trajectory required for lab 2.
    '''
    #print "In trajectory"
    stopRobot()
    driveStraight(5, .6)
    rotate(-1.57)
    driveArc(.15,.2,3.14)
    rotate(3*math.pi/4)
    driveStraight(5, .42)
#     sleep(3)
    
def bumpHandler(data):
    '''
    When the center button is pressed, this will trigger some code to be executed
    '''
    #print "BUMP!"
    if data.state == BumperEvent.PRESSED:
        executeTrajectory()
        
    
    
if __name__=="__main__":
    global odom_Subscriber, vel_Publisher, odom
    rospy.init_node('botDriver')
    vel_Publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)  
    odom = RobotOdometry()
    bumper_Subscriber = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, bumpHandler)
    sleep(1) #wait for the first odometry data to publish
    counter = 0
    #gradualAcceleration(1, .25)
    while (1):
        if rospy.is_shutdown():
            break
        counter += 1

    
