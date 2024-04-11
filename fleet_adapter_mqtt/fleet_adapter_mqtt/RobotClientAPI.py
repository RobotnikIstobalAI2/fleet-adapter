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

# RobotClientAPI.py
#
# @author Emima Jiva. (c) ai2-UPV Todos los derechos reservados.
#
# Proyecto "WASHCARROB"
# Version: 1.0
# Rev: 2023. cambios introducidos  

import paho.mqtt.client as mqtt
import json
import math
import numpy as np
import time

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
    def __init__(self, broker: str, port: int, keep_alive: int, anonymous_access: bool, user: str, password: str, \
                pose_topic: str, feedback_topic: str, result_topic: str, battery_topic: str, \
                goal_dist: int, dispenser_topic: str, ingestor_topic: str, start_ae_topic: str):
        #Delivery topic
        self.dispenser_topic = dispenser_topic
        self.ingestor_topic = ingestor_topic
        self.start_ae_topic = start_ae_topic
        #Distance parameter
        self.goal_dist = goal_dist
        #Position information
        self.x = {}
        self.y = {}
        self.theta = {}
        self.x_goal = {}
        self.y_goal = {}
        self.theta_goal = {}
        #Dictionary robot-pose
        self.robotpose = {}
        #Dictionary battery level
        self.battery = {}
        #Dictionary result
        self.resultgoal = {}
        #Dictionary result
        self.feedback = {}
        #A dict with existing robots
        self.robots = {}
        #MQTT Connection
        self.client = self.connect_mqtt(broker, port, keep_alive, anonymous_access, user, password, \
                                        pose_topic, feedback_topic, result_topic, battery_topic)
        self.client.loop_start()

    def connect_mqtt(self, broker, port, keep_alive, anonymous_access, user, password, \
                     pose_topic, feedback_topic, result_topic, battery_topic):
        def on_connect(client, userdate, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker")
            else:
                print("Failed to connect")
        client = mqtt.Client('fleet-adapter')
        client.on_connect = on_connect
        client.message_callback_add(pose_topic, self.on_message_pose)
        client.message_callback_add(result_topic, self.on_message_result)
        client.message_callback_add(feedback_topic, self.on_message_feedback)
        client.message_callback_add(battery_topic, self.on_message_battery)
        if not anonymous_access:
            client.username_pw_set(user, password)
        client.connect(broker, port, keep_alive)
        client.subscribe(pose_topic, 2)
        client.subscribe(result_topic, 2)
        client.subscribe(feedback_topic, 2)
        client.subscribe(battery_topic, 2)
        return client
    
    def on_message_pose(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        pose=json.loads(decoded_message)['pose']['pose']
        robot = msg.topic.split("/")[1]
        self.robotpose[robot] = pose
        self.robots[robot] = robot
    
    def on_message_result(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        result=json.loads(decoded_message)['status']['status']
        robot = msg.topic.split("/")[1]
        self.resultgoal[robot] = result

    def on_message_feedback(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        feed=json.loads(decoded_message)['status']['status']
        robot = msg.topic.split("/")[1]
        self.feedback[robot] = feed

    def on_message_battery(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        bat=json.loads(decoded_message)['level']
        robot = msg.topic.split("/")[1]
        self.battery[robot] = bat

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        if self.robots.get(robot_name) is not None:
            self.x[robot_name] = round(self.robotpose[robot_name]['position']['x'],2)
            self.y[robot_name] = round(self.robotpose[robot_name]['position']['y'],2)
            self.theta[robot_name] = round(euler_from_quaternion(
            self.robotpose[robot_name]['orientation']['x'], self.robotpose[robot_name]['orientation']['y'],
            self.robotpose[robot_name]['orientation']['z'], self.robotpose[robot_name]['orientation']['w'])[2],2)
            return [self.x[robot_name],self.y[robot_name],self.theta[robot_name]]
        else:
            print("No position for " + robot_name)
            return None

 
    def navigate(self,
                 robot_name: str,
                 cmd_id: int,
                 pose,
                 map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        self.x_goal[robot_name] = round(pose[0],2)
        self.y_goal[robot_name] = round(pose[1],2)
        self.theta_goal[robot_name] = round(pose[2],2)
        orientation = quaternion_from_euler(0,0,round(pose[2],2))
        data = {"goal":{"target_pose":{"header":{
                                            "frame_id":map_name},
                                    "pose":{
                                            "position":{
                                                "x":self.x_goal[robot_name],
                                                "y":self.y_goal[robot_name],
                                                "z":0
                                            },
                                            "orientation":{
                                                "x":orientation[0],
                                                "y":orientation[1],
                                                "z":orientation[2],
                                                "w":orientation[3]
                                            }}}}}
        self.client.publish("goal/"+robot_name ,json.dumps(data), 2)
        if self.feedback.get(robot_name) is not None and self.feedback[robot_name] == 1:
            self.feedback[robot_name] = 0
            return True
        else:
            return False

    def start_process(self,
                      robot_name: str,
                      cmd_id: int,
                      process: str,
                      map_name: str):
        return True
    
    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        return 0.0
    
    def stop(self, robot_name: str, cmd_id: int):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        cancel_all_goals = {"id":""}
        self.client.publish("cancel/"+robot_name ,json.dumps(cancel_all_goals), 2)
        return True

    def navigation_completed(self, robot_name: str, cmd_id: int):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        distance = (math.sqrt((self.x_goal[robot_name]-self.x[robot_name])**2 + (self.y_goal[robot_name]-self.y[robot_name])**2))
        #if (self.resultgoal.get(robot_name) is not None and self.resultgoal[robot_name] == 3):
        if (distance < self.goal_dist):
            self.resultgoal[robot_name] = 0
            #print("Navigation completed! " +  robot_name, flush=True)
            return True
        else:
            #print("Navigation not completed " + robot_name + " " + str(distance), flush=True)
            return False 
        
    def process_completed(self, robot_name: str, cmd_id: int):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        return self.navigation_completed(robot_name, cmd_id)
    
    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        if self.battery.get(robot_name) is not None:
           #print("Battery " + str(self.battery.get(robot_name)), flush=True)
            return self.battery[robot_name]/100
        else:
            #print("Battery none ", flush=True)
            return None
    
    def requires_replan(self, robot_name: str):
        '''Return whether the robot needs RMF to replan'''
        return False
        
    def pub_dispenser_requests(self, robot_name: str, task_id: str):
        data = { "data": task_id }
        self.client.publish(robot_name + self.dispenser_topic ,json.dumps(data), 2)

    def pub_ingestor_requests(self, robot_name: str, task_id: str):
        data = { "data": task_id }
        self.client.publish(self.ingestor_topic + robot_name ,json.dumps(data), 2)
    
    def publish_action_execution(self, robot_name: str):
        data = { "data": True }
        self.client.publish(self.start_ae_topic + robot_name ,json.dumps(data), 2)

