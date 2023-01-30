# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import DestinationRequest
from rmf_fleet_msgs.msg import FleetState
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import ModeRequest
import uuid
from math import sqrt
import rospy

'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

class RobotAPI(Node):
    def __init__(self, fleet: str, robot: str):
        super().__init__('fleet_adapter_'+fleet+"_"+robot)
        print("+++++++++++++++++++++++",fleet, robot,"+++++++++++++++++++++++++++")
        self.connected = False
        self.robot=robot
        self.fleet=fleet
        self.mode = None
        self.battery_percent = 1.0
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.x_goal = 0
        self.y_goal = 0
        self.level_name = "Floor"
        self.path = []
        self.task_id = str(uuid.uuid4())

        self.nav_publisher = self.create_publisher(DestinationRequest, '/robot_destination_requests', 10)
        self.mode_publisher = self.create_publisher(ModeRequest, "/robot_mode_requests", 10)
        self.mode_msg = ModeRequest()
        self.nav_msg = DestinationRequest()

        self.subscription = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.listener_callback,
            2)
        self.subscription 
        rclpy.spin_once(self)


    def listener_callback(self, fleet):
        if fleet.name == self.fleet:
            for robot in fleet.robots:
                if robot.name == self.robot:
                    self.mode = robot.mode.mode
                    self.battery_percent = robot.battery_percent
                    self.x = robot.location.x
                    self.y = robot.location.y
                    self.yaw = robot.location.yaw
                    self.level_name = robot.location.level_name
                    self.path = robot.path
                    break

    def position(self):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        rclpy.spin_once(self)
        if self.x != None and self.y != None and self.yaw != None:
            print("+++++++++++++++++++++++++++++++Getting position: ", [self.x, self.y, self.yaw],"+++++++++++++++++++++++++++++++++++++++++++")
            return [self.x, self.y, self.yaw]
        return None

    def navigate(self, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        print("Navigating to pose: ", pose, map_name)
        self.nav_msg.fleet_name = self.fleet
        self.nav_msg.robot_name = self.robot
        self.task_id = str(uuid.uuid4())
        self.nav_msg.task_id = self.task_id
        self.nav_msg.destination.x = pose[0]
        self.nav_msg.destination.y = pose[1]
        self.nav_msg.destination.yaw = pose[2]
        self.nav_msg.destination.level_name = map_name
        self.x_goal = pose[0]
        self.y_goal = pose[1]
        try:
            self.nav_publisher.publish(self.nav_msg)
        except Exception as e:
            print(e)
            return False

        return True

    def navigation_remaining_duration(self):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        return sqrt((self.x_goal-self.x)**2 + (self.y_goal-self.y)**2)/4

    def navigation_completed(self):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''

        if sqrt((self.x_goal-self.x)**2 + (self.y_goal-self.y)**2) < 0.5:
            print("Navigation completed!")
            return True
        print("Navigation in progress...")
        return False

    def stop(self):
        print("Stop")
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        # self.mode_msg.fleet_name = self.fleet
        # self.mode_msg.robot_name = self.robot
        # self.mode_msg.task_id = self.task_id
        # self.mode_msg.mode.mode = RobotMode.MODE_PAUSED
        # try:
        #     self.mode_publisher.publish(self.mode_msg)
        # except:
        #     return False

        return True

    def start_process(self, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        print("starting_process")
        return True

    def process_completed(self):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        print("process_completed")
        return True

    def battery_soc(self):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''

        if self.battery_percent > 1.0 and self.battery_percent < 0.0:
            return None
        return self.battery_percent
