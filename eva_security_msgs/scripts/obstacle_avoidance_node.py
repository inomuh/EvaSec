#!/usr/bin/env python3
# coding=utf-8

import rospy

from sensor_msgs.msg import Range
from eva_security_msgs.srv import *


class EvaSecurityObstacleAvoidance(object):

    def __init__(self):
        self.robot_move_order = True
        self.old_move_order_check = True        
        self.move_order_change_check = False

        self.obstacle_distance_limit = rospy.get_param("~Parameters/obstacle_distance_limit")

        self.main()

    def main(self):
        rospy.Subscriber('/sonar1', Range, self.sonar_callback)
        
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.move_order_change_check:
                detection = self.obstacle_detector(self.robot_move_order)
                print("Call Obstacle Detection..")
                rospy.loginfo("Call Obstacle Detection..")
                self.move_order_change_check = False

            rate.sleep()

    def sonar_callback(self, distance_msg):
        detected_sonar_distance = float(distance_msg.range)

        if detected_sonar_distance < self.obstacle_distance_limit:
            self.robot_move_order = False

        else:
            self.robot_move_order = True

        if self.robot_move_order != self.old_move_order_check:
            self.move_order_change_check = True
            self.old_move_order_check = self.robot_move_order

    def obstacle_detector(self, request):
        rospy.wait_for_service('obstacle_info')

        try:
            obstacle_info = rospy.ServiceProxy('obstacle_info', ObstacleInfo)
            feedback = obstacle_info.call(ObstacleInfoRequest(request))

            return feedback.feedback

        except rospy.ServiceException as e:
            print("Service call failed: %s", e)


if __name__ == '__main__':
    try:
        rospy.init_node('eva_security_obstacle_avoidance_node', anonymous=True)

        node = EvaSecurityObstacleAvoidance()

    except rospy.ROSInterruptException:
        pass
