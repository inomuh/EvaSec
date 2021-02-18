#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from eva_security_msgs.msg import DistanceInfo
from geometry_msgs.msg import Twist
from eva_security_msgs.srv import *


class EvaSecurityPatrolMission(object):

    def __init__(self):
        self.distance_info_calculated_distance = 0
        self.robot_move_order = True

        self.total_distance_covered = 0
        self.total_distance_of_completed_missions = 0
        self.mission_list = list()
        self.mission_order_func()
        self.ongoing_mission = 0

        self.robot_turning_order = False

        self.completed_mission_percentage = 0

        self.robot_head_angle = 0
        self.robot_head_angle_tolerance = rospy.get_param("~Parameters/head_angle_referance")
        self.angular_ref_speed = rospy.get_param("~Parameters/angular_speed_referance")

        self.main()


    def main(self):
        rospy.Subscriber ('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/distance_calc', DistanceInfo, self.distance_calculation_callback)
        self.robot_speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.obstacle_info_srv = rospy.Service('obstacle_info', ObstacleInfo, self.obstacle_info_service)
        self.info_service = rospy.Service('info_service', InfoService, self.mission_info_service)
        
        rate = rospy.Rate(2)

        robot_speed_msg = Twist()

        while not rospy.is_shutdown():
            if not self.robot_move_order:
                self.robot_stay_func(robot_speed_msg)

            else:
                self.robot_follow_mission_func(robot_speed_msg)

            rate.sleep()

    def read_mission_func(self):
        read_missions_file = dict(rospy.get_param("~Missions"))
        temporary_mission_list = list()
        for i in range(len(read_missions_file.keys())):
            key = "Mission_" + str(i + 1)
            temp_list = [read_missions_file[str(key)]["going_road"], read_missions_file[str(key)]["head_angle"]]
            temporary_mission_list.append(temp_list)

        return temporary_mission_list

    def mission_order_func(self):
        self.mission_list = self.read_mission_func()
        for i in range(len(self.mission_list)):
            self.total_distance_covered += self.mission_list[i][0]


    def robot_stay_func(self, robot_speed_msg):
        print("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\t\t\tOBSTACLE WARNING !!!\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
        robot_speed_msg.linear.x = 0.0
        robot_speed_msg.angular.z = 0.0
        self.robot_speed_pub.publish(robot_speed_msg)

    def robot_follow_mission_func(self, robot_speed_msg):
        if self.ongoing_mission < len(self.mission_list):
            must_going_distance_in_mission = self.mission_list[self.ongoing_mission][0]
            turning_angle_in_mission = self.mission_list[self.ongoing_mission][1]

            ongoing_distance_in_mission = self.distance_info_calculated_distance - self.total_distance_of_completed_missions
            if self.robot_turning_order:
                angular_speed = self.robot_turning_func(turning_angle_in_mission)
                robot_speed_msg.linear.x = 0.0
                robot_speed_msg.angular.z = angular_speed

                self.robot_speed_pub.publish(robot_speed_msg)

            else:

                if (ongoing_distance_in_mission < must_going_distance_in_mission):
                    robot_speed_msg.linear.x = 0.2
                    robot_speed_msg.angular.z = 0.0

                    self.robot_speed_pub.publish(robot_speed_msg)

                else:
                    self.total_distance_of_completed_missions = self.distance_info_calculated_distance
                    self.robot_turning_order = True

        else:
            print("@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
            print("\t\tMISSION COMPLETED")
            print("@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")

            robot_speed_msg.linear.x = 0.0
            robot_speed_msg.angular.z = 0.0

            self.robot_speed_pub.publish(robot_speed_msg)

    def robot_turning_func(self, target_head_angle_degree):
        target_head_angle_radian = target_head_angle_degree*math.pi/180
        angular_speed = self.angular_ref_speed * (target_head_angle_radian - self.robot_head_angle)

        if abs(target_head_angle_radian - self.robot_head_angle) < self.robot_head_angle_tolerance:
            angular_speed = 0.0
            self.robot_turning_order = False
            print("Mission Completed. Go Another.")
            print("\n Mission:",self.ongoing_mission+1,"Started")
            print("\n\n\n")
            self.ongoing_mission += 1

        return angular_speed


    def odom_callback(self, odom_msg):
        roll = pitch = yaw = 0.0
        orientation = odom_msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.robot_head_angle = yaw

 
    def obstacle_info_service(self, request):
        self.robot_move_order = request.request
        feedback = self.robot_move_order

        return ObstacleInfoResponse(feedback)


    def mission_info_service(self, request):
        read_info_service_request = request.request

        if read_info_service_request == "Mission Percentage":
            feedback = float(self.completed_mission_percentage)

        else:
            feedback = 0.0

        return InfoServiceResponse(feedback)

    def distance_calculation_callback(self, odom_msg):
        self.distance_info_calculated_distance = float(odom_msg.distance_info)
        self.completed_mission_percentage = float((self.distance_info_calculated_distance / self.total_distance_covered) * 100)


if __name__ == '__main__':
    try:
        rospy.init_node('eva_security_patrol_mission_node', anonymous=True)
        node = EvaSecurityPatrolMission()

    except rospy.ROSInterruptException:
        pass
