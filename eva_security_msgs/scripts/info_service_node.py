#!/usr/bin/env python3
# coding=utf-8

import rospy
from eva_security_msgs.srv import *

class EvaSecurityInfoService(object):

    def __init__(self):
        self.main()

    def main(self):

        service_feedback = self.service_info_request("Mission Percentage")
        print("\n\n\n")
        print("Completed Mission Percentage")
        print("\t % " + str(service_feedback))
        print("\n\n\n")

    def service_info_request(self, request):
        rospy.wait_for_service('info_service')

        try:
            info_service = rospy.ServiceProxy('info_service', InfoService)
            feedback = info_service.call(InfoServiceRequest(request))

            return feedback.feedback

        except rospy.ServiceException as e:
            print("Service call failed: %s",e)


if __name__ == '__main__':
    try:
        rospy.init_node('eva_security_info_service_node', anonymous=True)
        dugum = EvaSecurityInfoService()

    except rospy.ROSInterruptException:
        pass
