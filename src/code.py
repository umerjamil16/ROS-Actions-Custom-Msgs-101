#! /usr/bin/env python
import rospy
import actionlib
from actions_quiz.msg import CustomActionMsgFeedback, CustomActionMsgResult, CustomActionMsgAction

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class CustomServer(object):

  # create messages that are used to publish feedback/result
  _feedback = CustomActionMsgFeedback()
#  _result   = CustomActionMsgResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
    self._as.start()

  def drone_takeoff(self):
    #Create a msg to takeoff the drone
    takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    #Drone taking off
    i = 0
    while not i == 3:
        takeoff_pub.publish(Empty())
        rospy.loginfo("Taking off the drone...")
        rospy.sleep(1)
        i += 1
  move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

  def drone_land(self):
    #Drone Landing
    #Create a msg to land the drone
    land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)
    i = 0
    while not i == 3:
        #first stop the drone twist
        move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        move_msg = Twist()
        move_msg.linear.x = 1
        move_msg.angular.z = 1
        move_pub.publish(move_msg)
        land_pub.publish(Empty())
        rospy.loginfo("Landing the drone...")
        rospy.sleep(1)
        i += 1

  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    # this is the function that computes the Fibonacci sequence
    # and returns the sequence to the node that called the action server

    # helper variables
    r = rospy.Rate(1)
    success = True

    #get the flight side time
    #side_time = goal.goal
    if goal.goal == 'TAKEOFF':
        #takeoff
        self._feedback.feedback = "TAKEOFF"
        self._as.publish_feedback(self._feedback)
        self.drone_takeoff()
        if self._as.is_preempt_requested():
            rospy.loginfo('The goal has been cancelled/preempted')
            self._as.set_preempted()
            success = False
    elif goal.goal == 'LAND':
        #land drone
        self._feedback.feedback = "LAND"
        self._as.publish_feedback(self._feedback)
        self.drone_land()
        if self._as.is_preempt_requested():
            rospy.loginfo('The goal has been cancelled/preempted')
            self._as.set_preempted()
            success = False
    else:
        self._feedback.feedback = "BAD INPUT, TRY AGAIN"

      # builds the next feedback msg to be sent
      # the sequence is computed at 1 Hz frequency
    r.sleep()

    if success:
        rospy.loginfo('Succeeded in drone man_ ' )

if __name__ == '__main__':
  rospy.init_node('drone_move_str')
  CustomServer()
  rospy.spin()