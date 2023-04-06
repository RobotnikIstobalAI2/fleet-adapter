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

import paho.mqtt.client as mqtt
import json
import math
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        #Position information
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.x_goal = 0
        self.y_goal = 0
        #Dictionary robot-pose
        self.robotpose = {}
        #Dictionary robot-feedback
        self.robotfeedback = {}
        #Dictionary robot-status
        self.robotresult= {}
        #Current goal variable
        self.current_goal = False
        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")
        #MQTT Connection
        self.client = self.connect_mqtt()
        self.client.loop_start()
        self.client.subscribe("pose/#")
        self.client.subscribe("feedback/#")
        self.client.subscribe("result/#")

    def connect_mqtt(self):
        def on_connect(client, userdate, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker")
            else:
                print("Failed to connect")
        client = mqtt.Client('fleet-adapter')
        client.on_connect = on_connect
        client.message_callback_add('pose/#', self.on_message_pose)
        client.message_callback_add('feedback/#', self.on_message_feedback)
        client.message_callback_add('result/#', self.on_message_result)
        client.connect('127.0.0.1', 1883)
        return client

    def on_message_pose(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        pose=json.loads(decoded_message)['pose']['pose']
        robot = msg.topic.split("/")[1]
        self.robotpose[robot] = pose

    def on_message_feedback(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        feedback=json.loads(decoded_message)['status']['status']
        robot = msg.topic.split("/")[1]
        self.robotfeedback[robot] = feedback

    def on_message_result(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        result=json.loads(decoded_message)['status']['status']
        robot = msg.topic.split("/")[1]
        self.robotresult[robot] = result

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        self.x = self.robotpose[robot_name]['position']['x']
        self.y = self.robotpose[robot_name]['position']['y']
        self.theta = euler_from_quaternion(
            self.robotpose[robot_name]['orientation']['x'], self.robotpose[robot_name]['orientation']['y'],
            self.robotpose[robot_name]['orientation']['z'], self.robotpose[robot_name]['orientation']['w'])[2]
        if self.x != None and self.y != None and self.theta != None:
            return [self.x,self.y,self.theta]
        else:
            return None

    def navigate(self, robot_name: str, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        self.x_goal = pose[0]
        self.y_goal = pose[1]
        orientation = quaternion_from_euler(0,0,pose[2])
        data = {"goal":{"target_pose":{"header":{
                                            "frame_id":map_name},
                                       "pose":{
                                            "position":{
                                                "x":self.x_goal,
                                                "y":self.y_goal,
                                                "z":0
                                            },
                                            "orientation":{
                                                "x":orientation[0],
                                                "y":orientation[1],
                                                "z":orientation[2],
                                                "w":orientation[3]
                                            }}}}}
        self.client.publish("goal/"+robot_name ,json.dumps(data))
        if self.robotfeedback[robot_name] == 1:
            self.current_goal = True
            return True
        else:
            self.current_goal = False
            return False

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        self.current_goal = False
        cancel_all_goals = {"id":""}
        self.client.publish("cancel/"+robot_name ,json.dumps(cancel_all_goals))
        return True

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return 0.0

    def navigation_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        if (self.robotresult[robot_name] == 3) and self.current_goal: 
            self.current_goal = False
            return True
        else:
            return False

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        return 0.8
