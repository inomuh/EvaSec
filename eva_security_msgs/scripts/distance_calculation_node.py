#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
from nav_msgs.msg import Odometry
from eva_security_msgs.msg import DistanceInfo

class EvaSecurity_Distance_Calculation(object):
    def __init__(self):
        self.old_pos_x = 0
        self.old_pos_y = 0
        self.now_pos_x = 0
        self.now_pos_y = 0
        self.odom_check = False

        self.total_distance = 0

        self.main()

    def main(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        dist_calc_pub = rospy.Publisher('/distance_calc', DistanceInfo, queue_size=10)

        rate = rospy.Rate(10)

        dist_calc_msg = DistanceInfo()

        while not rospy.is_shutdown():
            if self.odom_check:
                self.total_distance += self.dist_calc_func()
                dist_calc_msg.distance_info = self.total_distance
                rospy.loginfo(dist_calc_msg)
                dist_calc_pub.publish(dist_calc_msg)
                rate.sleep()

    def odom_callback(self, odom_msg):
        self.old_pos_x = self.now_pos_x
        self.old_pos_y = self.now_pos_y
        self.now_pos_x = odom_msg.pose.pose.position.x
        self.now_pos_y = odom_msg.pose.pose.position.y

        if not self.odom_check:
            self.old_pos_x = odom_msg.pose.pose.position.x
            self.old_pos_y = odom_msg.pose.pose.position.y
            self.odom_check = True
    
    def dist_calc_func(self):
        calculated_distance = math.sqrt(math.pow(self.now_pos_x - self.old_pos_x,2) + math.pow(self.now_pos_y - self.old_pos_y,2))
        return calculated_distance

if __name__ == '__main__':
    try:
        rospy.init_node('eva_security_distance_calculation_node',anonymous=True)
        node = EvaSecurity_Distance_Calculation()
    except rospy.ROSInterruptException:
        pass
