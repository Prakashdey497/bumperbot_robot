#!/usr/bin/env python3

import rospy
from bumperbot_odom_to_trajectory.path_odom_plotter import PathOdomPlotter


if __name__== '__main__':
    rospy.init_node('path_odom_plotter')
    max_list_append = rospy.get_param('~max_list_append')
    controller = PathOdomPlotter(max_list_append)

    rospy.spin()