#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import queue
import tf

class FollowWaypoint:
    def __init__(self):
        rospy.init_node("waypoint_follower")

        self.waypoint_topic = "initialpose"
        self.publish_waypoint_topic = "move_base_simple/goal"
        self.odom_pose_topic = "mbrb/odom"
        self.map_frame = "map"

        self.pub_waypoint = rospy.Publisher(self.publish_waypoint_topic,PoseStamped,queue_size=1)
        self.rate = rospy.Rate(10)

        self.waypoint_queue = queue.Queue()

        self.current_goal = None
        self.listener = tf.TransformListener()
        
    def subscriber(self):
        rospy.Subscriber(self.waypoint_topic, PoseWithCovarianceStamped, self.waypointCallback)
        rospy.Subscriber(self.odom_pose_topic, Odometry, self.odomCallback)
        
        rospy.spin()
    
    def waypointCallback(self,message):
        dummy_message = PoseStamped()
        
        dummy_message.header = message.header
        dummy_message.pose = message.pose.pose

        dummy_message = self.listener.transformPose(self.map_frame,dummy_message)

        self.waypoint_queue.put(dummy_message)
        print("new waypoint was added, total waypoint : {}".format(self.waypoint_queue.qsize()))
        print(dummy_message.pose)
        print("********************")
    
    def calPose(self,odom_message):
        x = odom_message.pose.pose.position.x
        y = odom_message.pose.pose.position.y
        
        x_goal = x
        y_goal = y
        # if goal exist set x_goal and y_goal to that goal position
        if self.current_goal != None:
            x_goal = self.current_goal.pose.position.x
            y_goal = self.current_goal.pose.position.y
            
        return (x-x_goal)**2 + (y-y_goal)**2

    def odomCallback(self,message):
        del_pose = self.calPose(message)
        
        # reach target and queue is not empty
        if del_pose < 5e-2 and not self.waypoint_queue.empty():
            self.current_goal = self.waypoint_queue.get()
            
            next_waypoint = self.current_goal
            self.pub_waypoint.publish(next_waypoint)
            self.rate.sleep()
        
            print("next way point was publish, {} waypoint left".format(self.waypoint_queue.qsize()))
            print(next_waypoint.pose)
            print("********************")

if __name__ == "__main__":
    follow_waypoint = FollowWaypoint()
    follow_waypoint.subscriber()
