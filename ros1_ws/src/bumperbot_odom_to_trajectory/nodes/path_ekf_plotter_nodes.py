#!/usr/bin/env python3

import rospy
from bumperbot_odom_to_trajectory.path_ekf_plotter import PathEkfPlotter


if __name__== '__main__':
    rospy.init_node('path_ekf_plotter')
    max_list_append = rospy.get_param('~max_list_append')
    controller = PathEkfPlotter(max_list_append)

    rospy.spin()