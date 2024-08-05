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
#
# @author Emima Jiva. <emji@ai2.upv.es>  ai2-UPV
# @maintanier Guillem Gari  <ggari@robotnik.es> Robotnik Automation S.L.
# @maintanier Sandra Moreno <smoreno@robotnik.es> Robotnik Automation S.L.

import rclpy
import rclpy.node
from rclpy.duration import Duration

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary, ModeRequest, RobotMode
from rmf_door_msgs.msg import DoorRequest, DoorState

import numpy as np

import threading
import math
import copy
import enum
import time
import json

from datetime import timedelta


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2


class PlanWaypoint:
    def __init__(
        self,
        index,
        wp: plan.Waypoint
    ):
        # the index of the Plan::Waypoint in the waypoints in follow_new_path
        self.index = index
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 orientation_charger,
                 update_frequency,
                 adapter,
                 api,
                 lane_merge_distance,
                 finish_ae_topic,
                 door_state_topic,
                 finish_dock_topic,
                 finish_undock_topic,
                 recharge_soc):
        adpt.RobotCommandHandle.__init__(self)
        self.debug = False
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        self.lane_merge_distance = lane_merge_distance
        self.finish_ae_topic = finish_ae_topic
        self.door_state_topic = door_state_topic
        self.finish_dock_topic = finish_dock_topic
        self.finish_undock_topic = finish_undock_topic
        self.perform_filtering = True
        self.orientation_charger = orientation_charger
        self.recharge_soc = recharge_soc

        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} \
          does not exist in the navigation graph"
        self.charger_waypoint_index = waypoint.index
        # self.charger_is_set = False
        self.update_frequency = update_frequency
        self.update_handle = None  # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        # (x,y,theta) in RMF coordinates (meters, radians)
        self.position = position
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        self.adapter = adapter
        self.action_execution = None
        self.pub_action_execution = False

        self.requested_waypoints = []  # RMF Plan waypoints
        self.remaining_waypoints = []
        self.docks = {}

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        self.last_replan_time = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint = None  # this is a Plan::Waypoint
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None
        # The graph index of the waypoint the robot starts or ends an action
        self.action_waypoint_index = None
        self.current_cmd_id = 0
        self.started_action = False

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()
        self._stopping_thread = None
        self._quit_stopping_event = threading.Event()

        self.node.get_logger().info(
            f"The robot is starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        # Dock variables
        self.dock_finish = False
        self.undock_finish = False

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.node.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        self.node.create_subscription(
            ModeRequest,
            '/action_execution_notice',
            self.mode_request_cb,
            qos_profile=qos_profile_system_default)

        self.node.create_subscription(
            DoorRequest,
            '/adapter_door_requests',
            self.door_request_cb,
            qos_profile=qos_profile_system_default)

        self.action_execution_pub = self.node.create_publisher(
            ModeRequest,
            '/action_execution_notice',
            1
        )

        self.door_state_pub = self.node.create_publisher(
            DoorState,
            '/door_states',
            1
        )

        self.api.client.message_callback_add(
            finish_ae_topic + self.name,
            self._finish_ae_cb
        )
        self.api.client.subscribe(finish_ae_topic + self.name, 2)

        self.api.client.message_callback_add(
            door_state_topic,
            self.door_state_cb
        )
        self.api.client.subscribe(door_state_topic, 2)

        self.api.client.message_callback_add(
            finish_dock_topic,
            self.finish_dock_cb
        )
        self.api.client.subscribe(finish_dock_topic, 2)

        self.api.client.message_callback_add(
            finish_undock_topic,
            self.finish_undock_cb
        )
        self.api.client.subscribe(finish_undock_topic, 2)

        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.start()

        self.initialized = True

    def next_cmd_id(self):
        self.current_cmd_id = self.current_cmd_id + 1
        self.node.get_logger().debug(
            f'Issuing cmd_id for {self.name}: {self.current_cmd_id}'
        )
        return self.current_cmd_id

    def sleep_for(self, seconds):
        goal_time = self.node.get_clock().now() + Duration(
            nanoseconds=1e9 * seconds
        )
        while (self.node.get_clock().now() <= goal_time):
            time.sleep(0.001)

    def wait_on(self, event: threading.Event, seconds):
        goal_time = (
            self.node.get_clock().now() + Duration(nanoseconds=1e9 * seconds)
        )
        while self.node.get_clock().now() <= goal_time:
            if event.wait(0.001):
                return True
        return False

    def clear(self):
        self.requested_waypoints = []
        self.remaining_waypoints = []
        self.state = RobotState.IDLE

    def interrupt(self):
        self.node.get_logger().debug(
            f'Interrupting {self.name} '
            f'(latest cmd_id is {self.current_cmd_id})'
        )
        self._quit_dock_event.set()
        self._quit_path_event.set()
        self._quit_stopping_event.set()

        if self._follow_path_thread is not None:
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()

        if self._dock_thread is not None:
            if self._dock_thread.is_alive():
                self._dock_thread.join()

        if self._stopping_thread is not None:
            if self._stopping_thread.is_alive():
                self._stopping_thread.join()

    def stop(self):
        self.node.get_logger().debug(
            f'stop for {self.name} with PlanId'
            f'{self.update_handle.unstable_current_plan_id()}'
        )

        self.interrupt()
        # Stop the robot. Tracking variables should remain unchanged.
        with self._lock:
            self._quit_stopping_event.clear()

            def _stop():
                while not self._quit_stopping_event.is_set():
                    # self.node.get_logger().info(
                    #     f"Requesting {self.name} to stop..."
                    # )
                    if self.api.stop(self.name, self.next_cmd_id()):
                        break
                    self._quit_stopping_event.wait(0.1)

            self._stopping_thread = threading.Thread(target=_stop)
            self._stopping_thread.start()

    def replan(self):
        if self.update_handle is not None:
            now = self.adapter.now()
            if self.last_replan_time is not None:
                # TODO(MXG): Make the 15s replan cooldown configurable
                if now - self.last_replan_time < timedelta(seconds=15.0):
                    return
            self.last_replan_time = now
            self.update_handle.replan()
            self.node.get_logger().info(
                f'Requesting replan for {self.name} because of an obstacle'
            )

    def follow_new_path(
        self,
        waypoints,
        next_arrival_estimator,
        path_finished_callback
    ):

        # if self.debug:
        #     plan_id = self.update_handle.unstable_current_plan_id()
        self.interrupt()
        with self._lock:
            self._follow_path_thread = None
            self._quit_path_event.clear()
            self.clear()

            self.node.get_logger().info(f"Received new path for {self.name}")

            wait, entries = self.filter_waypoints(waypoints)

            self.remaining_waypoints = copy.copy(entries)

            assert next_arrival_estimator is not None
            assert path_finished_callback is not None

            def _follow_path():
                target_pose = None
                path_index = 0
                while self.remaining_waypoints \
                        or self.state == RobotState.MOVING:
                    # Save the current_cmd_id before checking if we need to
                    # abort. We should always be told to abort before the
                    # current_cmd_id gets modified, so whatever the value of
                    # current_cmd_id is before being told to abort will be the
                    # value that we want. If we are saving the wrong value
                    # here, then the next thing we will be told to do is abort.
                    cmd_id = self.current_cmd_id
                    # Check if we need to abort
                    if self._quit_path_event.is_set():
                        self.node.get_logger().info(
                            f"[{self.name}] aborting path request"
                        )
                        return
                    # State machine
                    if self.state == RobotState.IDLE or target_pose is None:
                        # Assign the next waypoint
                        self.target_waypoint = self.remaining_waypoints[0]
                        path_index = self.remaining_waypoints[0].index
                        # Move robot to next waypoint
                        target_pose = self.target_waypoint.position
                        [x, y] = self.transforms["rmf_to_robot"].transform(
                            target_pose[:2]
                        )
                        theta = (
                            target_pose[2]
                            + self.transforms['orientation_offset']
                        )
                        self.node.get_logger().info(
                            str(x) + " " + str(y) + " " + str(theta),
                        )
                        response = self.api.navigate(
                            self.name,
                            self.next_cmd_id(),
                            [x, y, theta],
                            self.map_name
                        )

                        if response:
                            self.remaining_waypoints = \
                                self.remaining_waypoints[1:]
                            self.state = RobotState.MOVING
                        else:
                            self.node.get_logger().info(
                                f"Robot {self.name} failed to request "
                                f"navigation to "
                                f"[{x:.0f}, {y:.0f}, {theta:.0f}]."
                                f"Retrying...")
                            self._quit_path_event.wait(0.1)

                    elif self.state == RobotState.MOVING:
                        time.sleep(0.5)
                        if self.api.requires_replan(self.name):
                            self.replan()

                        if self._quit_path_event.wait(0.1):
                            return
                        # Check if we have reached the target
                        with self._lock:
                            if self.api.navigation_completed(
                                    self.name, cmd_id):
                                self.node.get_logger().info(
                                    f"Robot [{self.name}] has reached the "
                                    f"destination for cmd_id {cmd_id}"
                                )
                                self.state = RobotState.IDLE
                                graph_index = self.target_waypoint.graph_index
                                if graph_index is not None:
                                    self.on_waypoint = graph_index
                                    self.last_known_waypoint_index = \
                                        graph_index
                                else:
                                    self.on_waypoint = None  # still on a lane
                            else:
                                # Update the lane the robot is on
                                lane = self.get_current_lane()
                                if lane is not None:
                                    self.on_waypoint = None
                                    self.on_lane = lane
                                else:
                                    # The robot may either be on the previous
                                    # waypoint or the target one
                                    if self.target_waypoint.graph_index is \
                                        not None \
                                        and self.dist(
                                            self.position, target_pose) < 0.5:
                                        self.on_waypoint =\
                                            self.target_waypoint.graph_index
                                    elif self.last_known_waypoint_index is \
                                            not None and self.dist(
                                            self.position,
                                            self.graph.get_waypoint(
                                                self.last_known_waypoint_index
                                            ).location) < 0.5:
                                        self.on_waypoint =\
                                            self.last_known_waypoint_index
                                    else:
                                        # update_off_grid()
                                        self.on_lane = None
                                        self.on_waypoint = None
                            duration = self.api.navigation_remaining_duration(
                                self.name, cmd_id
                            )

                            if path_index is not None and duration is not None:
                                next_arrival_estimator(
                                    path_index,
                                    timedelta(seconds=duration)
                                )

                if (not self.remaining_waypoints) \
                        and self.state == RobotState.IDLE:
                    path_finished_callback()
                    self.node.get_logger().info(
                        f"Robot {self.name} has successfully navigated along "
                        f"requested path."
                    )
            self._follow_path_thread = threading.Thread(
                target=_follow_path)
            self._follow_path_thread.start()

    def dock(
            self,
            dock_name,
            docking_finished_callback):
        ''' Docking is very specific to each application. Hence, the user will
            need to customize this function accordingly. In this example, we
            assume the dock_name is the same as the name of the waypoints that
            the robot is trying to dock into. We then call api.start_process()
            to initiate the robot specific process. This could be to start a
            cleaning process or load/unload a cart for delivery.
        '''
        self.node.get_logger().info("Docking Started")
        self.battery = self.get_battery_soc()
        self.interrupt()
        with self._lock:
            self._quit_dock_event.clear()
            self.dock_name = dock_name
            assert docking_finished_callback is not None
        dock_waypoint = self.graph.find_waypoint(self.dock_name)
        assert(dock_waypoint)
        self.dock_waypoint_index = dock_waypoint.index

        def _dock():
            cmd_id = self.next_cmd_id()
            target_pose = [
                dock_waypoint.location[0],
                dock_waypoint.location[1],
                self.orientation_charger
            ]
            [x, y] = self.transforms["rmf_to_robot"].transform(target_pose[:2])
            theta = target_pose[2]
            self.transforms['orientation_offset']
            response = self.api.navigate(
                self.name,
                self.next_cmd_id(),
                [x, y, theta],
                self.map_name)
            self.node.get_logger().info(
                (
                    "Dock "
                    + str(x)
                    + " "
                    + str(y)
                    + " "
                    + str(self.orientation_charger)
                ),
            )
            if response:
                self.state = RobotState.MOVING
            else:
                self.node.get_logger().info(
                    f"Robot {self.name} failed to request "
                    f"navigation to "
                    f"[{x:.0f}, {y:.0f}, {theta:.0f}]."
                    f"Retrying...")
            while self.state == RobotState.MOVING:
                time.sleep(0.5)
                self.node.get_logger().info("robot_moving")
                # Check if we have reached the target
                with self._lock:
                    print
                    if (
                        self.api.navigation_completed(self.name, cmd_id)
                        and self.api.position
                    ):
                        self.node.get_logger().info(
                            f"Robot [{self.name}] has reached the "
                            f"destination for cmd_id {cmd_id}"
                        )
                        self.state = RobotState.IDLE
                        graph_index = self.dock_waypoint_index
                        if graph_index is not None:
                            self.on_waypoint = graph_index
                            self.last_known_waypoint_index = \
                                graph_index
                        else:
                            self.on_waypoint = None  # still on a lane
                            self.node.get_logger().info("still on a lane")
                    else:
                        # Update the lane the robot is on
                        lane = self.get_current_lane()
                        if lane is not None:
                            self.on_waypoint = None
                            self.on_lane = lane
                        else:
                            # The robot may either be on the previous
                            # waypoint or the target one
                            if self.dock_waypoint_index is \
                                not None \
                                and self.dist(
                                    self.position, target_pose) < 0.5:
                                self.on_waypoint =\
                                    self.dock_waypoint_index
                            elif self.last_known_waypoint_index is \
                                    not None and self.dist(
                                    self.position,
                                    self.graph.get_waypoint(
                                        self.last_known_waypoint_index
                                    ).location) < 0.5:
                                self.on_waypoint =\
                                    self.last_known_waypoint_index
                            else:
                                self.on_lane = None
                                self.on_waypoint = None
            # 1. Publicar dock_request
            self.api.publish_dock_request(self.name, True)
            # 2. Esperar dock_finish
            while not self.dock_finish:
                self.node.get_logger().info(
                    f"Esperando recibir dock finalizado"
                )
            # 3. Mirar hasta que la batería supere soc
            while self.battery < self.recharge_soc:
                self.node.get_logger().info(
                    f"Esperando a que la bateria supere el recharge soc"
                )
                self.battery = self.get_battery_soc()
            # 4. Publicar undock_request
            self.api.publish_undock_request(self.name, True)
            # 5. Esperar undock_finish
            while not self.undock_finish:
                self.node.get_logger().info(
                    f"Esperando recibir undock finalizado"
                )

            with self._lock:
                self.dock_finish = False
                self.undock_finish = False
                self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
                docking_finished_callback()
                self.node.get_logger().info(
                    f"Robot {self.name} has completed docking"
                )
        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def get_position(self):
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position(self.name)
        if position is not None:
            x, y = self.transforms['robot_to_rmf'].transform(
                [
                    position[0],
                    position[1]
                ]
            )
            theta = math.radians(position[2]) - \
                self.transforms['orientation_offset']
            # Wrap theta between [-pi, pi]. Else arrival estimate will
            # assume robot has to do full rotations and delay the schedule
            if theta > np.pi:
                theta = theta - (2 * np.pi)
            if theta < -np.pi:
                theta = (2 * np.pi) + theta
            return [x, y, theta]
        else:
            self.node.get_logger().error(
                "Unable to retrieve position from robot.")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error(
                "Unable to retrieve battery data from robot.")
            return self.battery_soc

    def update(self):
        while rclpy.ok():
            self.position = self.get_position()
            self.battery_soc = self.get_battery_soc()
            if self.update_handle is not None:
                self.update_state()
            sleep_duration = float(1.0 / self.update_frequency)
            self.sleep_for(sleep_duration)

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        # Update position
        with self._lock:
            if (self.on_waypoint is not None):  # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif (self.on_lane is not None):  # if robot is on a lane
                # We only keep track of the forward lane of the robot.
                # However, when calling this update it is recommended to also
                # pass in the reverse lane so that the planner does not assume
                # the robot can only head forwards. This would be helpful when
                # the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:  # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            # if robot is performing an action
            elif (self.action_execution is not None):
                if self.pub_action_execution is False:
                    self.api.publish_action_execution(self.name)
                    self.pub_action_execution = True
                self.node.get_logger().info("Robot is performing an action")
                if not self.started_action:
                    self.started_action = True
                self.update_handle.update_off_grid_position(
                    self.position, self.action_waypoint_index)
            # if robot is merging into a waypoint
            elif (
                self.target_waypoint is not None
                and self.target_waypoint.graph_index is not None
            ):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:  # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name,
                    self.position,
                    max_merge_lane_distance=self.lane_merge_distance
                )

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py, _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        # Spin on the spot
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:  # The robot is on this lane
                return lane_index
        return None

    def dist(self, A, B):
        ''' Euclidian distance between A(x,y) and B(x,y)'''
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def filter_waypoints(
        self,
        wps: list,
        threshold=0.5
    ):
        ''' Return filtered PlanWaypoints'''

        assert(len(wps) > 0)
        first = None
        second = []
        threshold = 0.5
        last_pose = copy.copy(self.position)
        waypoints = []
        for i in range(len(wps)):
            # self.node.get_logger().info(("Waypoints " + str(wps[i].position))
            waypoints.append(PlanWaypoint(i, wps[i]))

        # We assume the robot will backtack if the first waypoint in the plan
        # is behind the current position of the robot
        first_position = waypoints[0].position
        if (
            len(waypoints) > 2
            and self.dist(first_position, last_pose) > threshold
        ):
            changed = False
            index = 0
            while (not changed):
                if self.dist(waypoints[index].position, first_position) > 0.1:
                    changed = True
                    break
                waypoints[index].position = last_pose
                index = index + 1

        if (self.perform_filtering is False):
            for i in range(len(waypoints)):
                self.node.get_logger().info(
                    f" [{self.name}] No filter waypoints "
                    f"{str(waypoints[i].position)}"
                )
            return (first, waypoints)

        changed = False
        # Find the first waypoint
        index = 0
        while (not changed and index < len(waypoints)):
            if (self.dist(last_pose, waypoints[index].position) < threshold):
                first = waypoints[index]
                # self.node.get_logger().info(
                #     "Distance last pose and waypoints  "
                #     + str(first.position)
                # )
                last_pose = waypoints[index].position
            else:
                break
            index = index + 1

        while (index < len(waypoints)):
            parent_index = copy.copy(index)
            wp = waypoints[index]
            if (self.dist(wp.position, last_pose) >= threshold):
                changed = False
                while (not changed):
                    next_index = index + 1
                    if (next_index < len(waypoints)):
                        if (
                            self.dist(
                                waypoints[next_index].position,
                                waypoints[index].position
                            ) < threshold
                        ):
                            if (next_index == len(waypoints) - 1):
                                # append last waypoint
                                changed = True
                                wp = waypoints[next_index]
                                wp.approach_lanes = (
                                    waypoints[parent_index].approach_lanes
                                )
                                # self.node.get_logger().info(
                                #     "Distance waypoint next  "
                                #     + str(wp.position)
                                # )
                                second.append(wp)
                        else:
                            # append if next waypoint changes
                            changed = True
                            wp = waypoints[index]
                            wp.approach_lanes = (
                                waypoints[parent_index].approach_lanes
                            )
                            self.node.get_logger().info(
                                "Append if next waypoint changes  "
                                + str(wp.position)
                            )
                            second.append(wp)
                    else:
                        # we add the current index to second
                        changed = True
                        wp = waypoints[index]
                        wp.approach_lanes = (
                            waypoints[parent_index].approach_lanes
                        )
                        self.node.get_logger().info(
                            "Add the current index to second  "
                            + str(wp.position)
                        )
                        second.append(wp)
                    last_pose = waypoints[index].position
                    index = next_index
            else:
                index = index + 1

        # for i in range(len(second)):
        #     self.node.get_logger().info(
        #         f" [{self.name}] Filter waypoints {str(second[i].position)}"
        #     )
        return (first, second)

    def complete_robot_action(self):
        with self._lock:
            if self.action_execution is None:
                return
            self.action_execution.finished()
            self.action_execution = None
            self.started_action = False
            self.pub_action_execution = False
            self.node.get_logger().info(f"Robot {self.name} has completed the"
                                        f" action it was performing")

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def mode_request_cb(self, msg):
        if msg.fleet_name is None or msg.fleet_name != self.fleet_name or\
                msg.robot_name is None or msg.robot_name != self.name:
            self.node.get_logger().info("Return request")
            return
        if msg.mode.mode == RobotState.IDLE:
            self.complete_robot_action()

    # When the rmf-core publishes to ROS2 in the /adapter_door_requests topic,
    # the information is published in MQTT for the door_node to receive it.
    def door_request_cb(self, msg):
        door_name = msg.door_name
        requested_mode = str(msg.requested_mode.value)
        self.node.get_logger().info(f"Door name {door_name}")
        self.node.get_logger().info(f"Door mode {requested_mode}")
        self.api.publish_door_request(door_name, requested_mode)

    def _finish_ae_cb(
        self,
        client,
        userdata,
        msg
    ):
        robot_name = msg.topic.split("/")[1]
        decoded_message = str(msg.payload.decode("utf-8"))
        data = json.loads(decoded_message)['data']
        self.node.get_logger().info(data)
        self.node.get_logger().info(self.name)
        self.node.get_logger().info(robot_name)
        if (data is True) and (robot_name == self.name):
            mode = RobotMode()
            execution_notice = ModeRequest()
            execution_notice.fleet_name = self.fleet_name
            execution_notice.robot_name = self.name
            execution_notice.mode.mode = RobotState.IDLE
            self.action_execution_pub.publish(execution_notice)

    # When the door_node publishes to /door_state, from MQTT
    # this information is received and published in ROS2.
    # In the /door_states topic as well, but in the ROS2 topic of rmf-core
    def door_state_cb(
        self,
        client,
        userdata,
        msg
    ):
        decoded_message = str(msg.payload.decode("utf-8"))
        door_state_recv = json.loads(decoded_message)['text']
        door_name = door_state_recv[0]
        door_mode = door_state_recv[1]
        self.node.get_logger().info(door_name + " " + door_mode)
        door_state = DoorState()
        door_state.door_name = door_name
        door_state.current_mode.value = int(door_mode)
        self.door_state_pub.publish(door_state)

    def finish_dock_cb(
        self,
        client,
        userdata,
        msg
    ):
        robot_name = msg.topic.split("/")[1]
        if robot_name == self.name:
            self.dock_finish = True

    def finish_undock_cb(
        self,
        client,
        userdata,
        msg
    ):
        robot_name = msg.topic.split("/")[1]
        if robot_name == self.name:
            self.undock_finish = True
