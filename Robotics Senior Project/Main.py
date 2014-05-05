#!/usr/bin/env python
'''
Module to run the full program

Author: Nicholas Morin, Ransom Mowris
'''
import time, sys, os, argparse
from threading import Thread, Lock
from multiprocessing import Process, Pipe
from PyQt4 import QtGui, QtCore

sys.path.append(os.path.join(os.path.dirname(__file__), '../DetectCylinder'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../RTPVideoStream'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../DisplayKinect'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from openravepy import *
from openravepy.interfaces import BaseManipulation, TaskManipulation
from Graspability import Graspability
from HighlightGraspable import HighlightGraspable
from PackBotCamera import HeadCamera
from kinect import Kinect
from ClickToGrip import GraspInterface
from DetectCylinder import DetectCylinder
from RobotWrapper import RobotWrapper
from PackBotController import *
from RawKinect import RawKinect
from VideoStream import showVideo
from QTViewer import Viewer, viewMain


class Main:
  def __init__(self, simulate, conn, envFile='environment.xml'):
    self.conn = conn #Conn is the link to our QT GUI which tells it when a traj is ready
    self.simulate = simulate #boolean which dictates whether PackBot will execute a traj

    self.env = Environment()
    self.env.SetViewer('qtcoin')
 
    self.env.Load(envFile) #contains the robot and the floor
    self.robot = self.env.GetRobots()[0]
    self.robot.SetActiveDOFs(range(5))
  
    self.taskmanip = TaskManipulation(self.robot) #used for controlling the gripper
    self.basemanip = BaseManipulation(self.robot) #used for controlling EOD arm

    self.simController = RaveCreateController(self.env,'IdealController')
 
    if self.simulate:
      self.robotWrapper = RobotWrapper(self.robot, self.simController, self.simController, True)
      self.robotWrapper.swapToSimController()
      self.grasp = Graspability(self.env, self.robotWrapper, lambda:self.robot.GetDOFValues()[:5])

      self.graspSelector = GraspInterface(self.env, self.simController, self.robotWrapper, self.grasp.getTraj, self.simulate)
    else:
      # real robot so we have a video stream to start
      self.videoThread = Thread(target=showVideo)
      self.videoThread.daemon = False
      self.videoThread.start()

      #Establishing a network connection to PackBot...
      self.controller = PackBotController(self.env)
      self.controller.InitializeCommunications()
      
      print 'giving time to make establish full connection to packbot'
      
      time.sleep(0.5)

      #initializes the arm callback
      self.robotWrapper = RobotWrapper(self.robot, self.simController, self.controller)
      self.robotWrapper.swapToRealController()

      time.sleep(.5)

      #graspability optimizes grasps to approach cans straight-on
      self.grasp = Graspability(self.env, self.robotWrapper, self.robot.GetDOFValues)
      #click-to-grasp callback thread
      self.graspSelector = GraspInterface(self.env, self.controller, self.robotWrapper, self.grasp.getTraj, self.simulate)


    #Custom UI elements which color graspable objects green and enable camera+kinect field of view shading
    self.highlight = HighlightGraspable(self.env, self.grasp.getTraj)
    self.cam = HeadCamera(self.env, self.robot)
    self.kinect = Kinect(self.env, self.robot)

    #Using the kinect+PCL to detect cylindrical objects and add them to the env
    self.detect = DetectCylinder(self.env, self.robot) 
    self.cylinders = self.detect.getCylinders()
    self.rawKinect = RawKinect(self.env, self.robot)

    #Setup the run variables
    self.runLock = Lock()
    self.run = True

    # create a lock and a 20Hz thread to handle various FoV elements
    self.DrawPackBotFoV = False
    self.ViewPackBotFoV = False
    self.DrawKinectFoV = False
    self.UpdateLock = Lock()
    self.foVUpdateThread  = Thread(target=self._FoVUpdate)
    self.foVUpdateThread.daemon = False
    self.foVUpdateThread.start()

    # We want to show the voxel grid by default
    self.ShowVoxelGrid = True

    # We don't want to see the point cloud by default
    self.ShowPointCloud = False

    # Initialize the number of voxels we want to draw
    self.numVoxelY = 100
    self.numVoxelX = 200
    self.numVoxelZ = 300

    if self.graspSelector.handle is None:
      print 'Callback was not registered -- grasp selection will not work'

  def Execute(self, traj=None):
    '''
    Execute the last valid trajectory saved in self.graspSelector

    Args:
         Traj: The trajectory to be executed. By default, this is none and
         will be parsed from the graspselector function (click2grip.py)

    Returns:
         Nothing. Runs a trajectory on the robot or in simulation
        
    '''     
    # save the currect active manip
    previousManip = self.robot.GetActiveManipulator()
 
    # set the active maninipulator to the gripper while we run
    manip = self.robot.SetActiveManipulator("gripper")   
    if traj:
      result = traj
    else:
      result = self.graspSelector.getTraj()
    if result:
      if self.simulate:
        self.robotWrapper.swapToSimController()
        self.taskmanip.ReleaseFingers()
        self.robot.WaitForController(0)
        self.robot.GetController().SetPath(result)
        self.robot.WaitForController(0)
        self.taskmanip.CloseFingers()
        self.robot.WaitForController(0)
        self.robotWrapper.swapToRealController()
      else:        
        print 'Running grasp in:'
        for t in ['3','2','1']:
          print t
          time.sleep(1)
	print 'GO'
        self.controller.Play()
        self.controller.GripOpen()
        print 'Wait for grip to open'
        time.sleep(3)
        self.controller.TestTrajectorySending(result)
        self.controller.GripClose()
    else:
      print 'No valid trajectory found'
    # reset the active manipulator 
    self.robot.SetActiveManipulator(previousManip)
     
  def parseCommand(self, command):
    '''
    parseCommands receives commands sent from the QTViewer
    
    Commmands are transmitted as a list.
    '''
    # TODO: make a dictionary of the valid commands with appropriate functions 
    print 'Received command for:', command[0]
    if command[0] == 'UpdateObstacleData':
      self.UpdateObstacleData()
    elif command[0] == 'UpdateCylinderData':
      self.UpdateCylinderData()
    elif command[0] == 'Plan':
      self.PlanTrajectories()
    elif command[0] == 'Execute':
      self.Execute()
    elif command[0] == 'Quit':
      self.quit()
    elif command[0] == 'Home':
      self.home()
    elif command[0] == 'DrawPackBotFoV':
      with self.UpdateLock:
        self.DrawPackBotFoV = command[1]
    elif command[0] == 'ViewPackBotFoV':
      with self.UpdateLock:
        self.ViewPackBotFoV = command[1]
    elif command[0] == 'DrawKinectFoV':
      with self.UpdateLock:
        self.DrawKinectFoV = command[1]
    elif command[0] == 'ShowVoxel':
      self.ShowVoxelGrid = command[1]
    elif command[0] == 'ShowPointCloud':
      self.ShowPointCloud = command[1]
    elif command[0] == 'UpdateNumVoxel':
      if command[2] == 'X':
        self.numVoxelX = command[1]
      elif command[2] == 'Y':
        self.numVoxelY = command[1]
      elif command[2] == 'Z':
        self.numVoxelZ = command[1]

  def home(self):
    '''
    Home the PackBot (move arm to a "Packed" pose) suitable for storage
    '''
    solution = [-0.0018, -2.9191, 2.381, -2.592, -1.5702]
    self.robot.SetActiveDOFValues(solution)
    traj = self.basemanip.MoveActiveJoints(goal=solution, outputtrajobj=True, 
                            maxiter=4000, steplength=0.01, maxtries=2, execute=False)
    self.controller.Play()
    self.controller.TestTrajectorySending(traj)
 
  def UpdateObstacleData(self):
    '''
    Update the obstacle data from the kinect
    '''
    self.rawKinect.clearData()
    self.rawKinect.setCylinders(self.cylinders)
    self.rawKinect.displayData(self.ShowVoxelGrid, self.ShowPointCloud, 
                            self.numVoxelY, self.numVoxelX, self.numVoxelZ)
    print 'Done obstacle update'

  def UpdateCylinderData(self):
    '''
    UpdateCylinderData clears the environment of cylinders and searches
    kinect data for cylinders again
    '''
    self.detect.deleteCylinders(self.cylinders)
    self.grasp.reset()
    self.graspSelector.saveTraj(None)
    self.detect.findCylinders()
    self.cylinders = self.detect.getCylinders()
    print 'Done cylinder update'

  def PlanTrajectories(self):
    '''
    Plan trajectories to the detected cylinders and color them appropriately.
    Graspable cylinders are green, all others are red.
    '''
    self.highlight.colorKinBodies(self.cylinders)
    print 'Done planning'
  
  def _FoVUpdate(self):
    '''
    Function to update various FoV elements at ~20Hz
    '''
    run = False
    with self.runLock:
      run = self.run
    if run:
      print 'Ready'
    count = 0
    while run:
      with self.UpdateLock:
        if self.ViewPackBotFoV:
          self.cam.enterFirstPerson()
        else:
          self.cam.returnToMainCam()
        if self.DrawPackBotFoV:
          self.cam.drawFoV()
        else:
          self.cam.clearFoV()
        if self.DrawKinectFoV:
          self.kinect.drawFoV()
        else:
          self.kinect.clearFoV()
        if count > 19:
          count = 0
          if self.graspSelector.getTraj():
            self.conn.send(['SetExecuteColor', 'green'])
          else:
            self.conn.send(['SetExecuteColor', 'red'])
        count += 1
      time.sleep(0.05)
      with self.runLock:
        run = self.run

  def start(self):
    '''
    Starts the main loop to check for updates from the gui and 
    initializes the cylinders and obstacle data
    '''
    run = False
    with self.runLock:
      run = self.run
    while run:
      command = self.conn.recv()
      self.parseCommand(command)
      with self.runLock:
        run = self.run

  def test(self):
    '''
    Function to test trajectory planning and target coloration
    '''
    self.cylinders = [self.env.GetKinBody('target'+str(i+1)) for i in range(1)]
    self.highlight.colorKinBodies(self.cylinders)
   
  def quit(self):
    '''
    Method to turn run to off and to cleanup the various threads
    '''
    with self.runLock:
      self.run = False
    self.foVUpdateThread.join()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulate', action='store_true',
                    help="Whether or not to move.")
    parser.add_argument('--test', action='store_true',
                    help="Whether or not to use the test configuration file")
    args = parser.parse_args(sys.argv[1:])

    parent_conn, child_conn = Pipe()
    qtgui = Process(target=viewMain, args=(child_conn,))
    qtgui.start()
    if args.test:
      m = Main(args.simulate, parent_conn, 'cylinders.xml')
      m.test()  
    else:
      m = Main(args.simulate, parent_conn)
    m.start()
    print 'Cleaning up'
    qtgui.join()
    RaveDestroy()
    print 'Done'



