#!/usr/bin/env python
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import time

import rospy

from osrf_gear.msg import Order
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def start_competition():
    rospy.loginfo("Waiting for competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")

    try:
        start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        response = start()
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to start the competition: %s" % exc)
        return
    if not response.success:
        rospy.logerr("Failed to start the competition: %s" % response)
    else:
        rospy.loginfo("Competition started!")


class MyCompetitionClass:
    def __init__(self):
        self.joint_trajectory_publisher = \
            rospy.Publisher("/ariac/arm/command", JointTrajectory, queue_size=10)
        self.received_orders = []
        self.current_joint_state = None
        self.last_joint_state_print = time.time()
        self.has_been_zeroed = False

    def order_callback(self, msg):
        rospy.loginfo("Received order:\n" + str(msg))

    def joint_state_callback(self, msg):
        if time.time() - self.last_joint_state_print >= 10:
            rospy.loginfo("Current Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_joint_state_print = time.time()
        self.current_joint_state = msg
        if not self.has_been_zeroed:
            self.has_been_zeroed = True
            rospy.loginfo("Sending arm to zero joint positions...")
            self.send_arm_to_zero_state()

    def send_arm_to_zero_state(self):
        msg = JointTrajectory()
        msg.joint_names = self.current_joint_state.name
        point = JointTrajectoryPoint()
        point.positions = [0] * len(msg.joint_names)
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]
        rospy.loginfo("Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)


def main():
    rospy.init_node("ariac_example_node")

    comp_class = MyCompetitionClass()
    order_sub = rospy.Subscriber("/ariac/orders", Order, comp_class.order_callback)
    joint_state_sub = rospy.Subscriber(
        "/ariac/joint_states", JointState, comp_class.joint_state_callback)

    rospy.loginfo("Setup complete.")
    start_competition()
    rospy.spin()

if __name__ == '__main__':
    main()
