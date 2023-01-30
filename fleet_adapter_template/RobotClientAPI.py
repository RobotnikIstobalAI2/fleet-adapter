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
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from sensor_msgs.msg import BatteryState
import uuid
from math import sqrt
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import actionlib

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
        self.battery_percent = 0.80
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.x_goal = 0
        self.y_goal = 0
        self.level_name = "Floor"
        self.task_id = str(uuid.uuid4())
        self.current_goal = False

        self.goal = MoveBaseGoal()
        rospy.init_node('ros1_adapter', anonymous=True)
        rospy.Subscriber("/" + self.robot + "/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
        rospy.Subscriber("/" + self.robot + "/battery_state", BatteryState, self.callback_battery)
        self.client_move = actionlib.SimpleActionClient("/" + self.robot + "/move_base", MoveBaseAction)

    def callback_battery(self, data):
        print(data.data)


    def callback_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]


    def position(self):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        if self.x != None and self.y != None and self.yaw != None:
            return [self.x, self.y, self.yaw]
        return None

    def navigate(self, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        print("Navigating to pose: ", pose, map_name)
        # if self.current_goal == False:
        #     self.current_goal = True
        # elif self.current_goal == True:
        #     self.client_move.wait_for_result()
        # self.client_move.wait_for_server()
        self.x_goal = pose[0]
        self.y_goal = pose[1]
        self.goal.target_pose.header.frame_id = "robot_map"
        self.goal.target_pose.pose.position.x = self.x_goal
        self.goal.target_pose.pose.position.y = self.y_goal
        orientation = quaternion_from_euler(0,0,pose[2])
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2] 
        self.goal.target_pose.pose.orientation.w = orientation[3]
        self.client_move.send_goal(self.goal)
        self.current_goal = True
        
        # return self.client_move.get_result()
        return True

    def navigation_remaining_duration(self):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        return 0

    def navigation_completed(self):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        if sqrt((self.x_goal-self.x)**2 + (self.y_goal-self.y)**2) < 0.5 and self.current_goal:
            print("Navigation completed!")
            self.current_goal = False
            return True
        return False

    def stop(self):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        print("Stop")
        self.current_goal = False
        self.client_move.cancel_all_goals()

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
