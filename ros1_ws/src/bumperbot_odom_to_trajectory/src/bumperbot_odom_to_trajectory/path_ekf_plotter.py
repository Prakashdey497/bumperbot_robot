#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class PathEkfPlotter(object):
    
    def __init__(self,max_append):
        rospy.loginfo("The parameter max_list_append is %d" % max_append)
        
        self.xAnt=0.0
        self.yAnt=0.0
        self.cont=0
        self.max_append_ = max_append
        
        
        self.odom_sub_ = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.odom_combined_callback)
        self.ekf_path_pub_ = rospy.Publisher('/ekfpath', Path, queue_size=1)
        
        self.path_ = Path() 
        self.odom_msg_ = Odometry()
        
        
    def odom_combined_callback(self,data):
        pose = PoseStamped()    

        #Set a atributes of the msg
        pose.header.frame_id = "odom"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)
        
        
        if (self.xAnt != pose.pose.position.x and self.yAnt != pose.pose.position.y):
                #Set a atributes of the msg
                pose.header.seq = self.path_.header.seq + 1
                self.path_.header.frame_id="odom"
                self.path_.header.stamp=rospy.Time.now()
                pose.header.stamp = self.path_.header.stamp
                self.path_.poses.append(pose)
                #Published the msg

        self.cont=self.cont+1
        if self.cont>self.max_append_:
            self.path_.poses.pop(0)

        self.ekf_path_pub_.publish(self.path_)

        #Save the last position
        xAnt=pose.pose.position.x
        yAnt=pose.pose.position.y