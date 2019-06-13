#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np
from respawn_helper import Respawn

class mobile_robot_actual:
   'Training environment for real life '

   def __init__(self ,mode='discrete' ,min_dist = 0.2):
      rospy.init_node('mobile_robot_actual', anonymous=True)
      self.robot_pos = np.array([None,None]);
      self.goal_pos= np.array([None,None]);

   def read_data(self):

       data = rospy.wait_for_message('/fiducial_transforms', FiducialTransformArray, timeout=15)
       # print('no:',len(data.transforms))
       # print(self.robot_pos,self.goal_pos)
       if len(data.transforms) >= 2:
           self.goal_pos=np.array([data.transforms[0].transform.translation.x,data.transforms[0].transform.translation.y])
           self.robot_pos=np.array([data.transforms[1].transform.translation.x,data.transforms[0].transform.translation.y])

       else:
           print('Either the robot position or the target position is unavailable')

       if self.goal_pos.all() != None and self.robot_pos.all() != None:
           dist = np.linalg.norm(self.robot_pos-self.goal_pos)
           print('dist:',dist)


if __name__ == '__main__':
    robot1 = mobile_robot_actual()
    while True:
        robot1.read_data()
    rospy.spin()
