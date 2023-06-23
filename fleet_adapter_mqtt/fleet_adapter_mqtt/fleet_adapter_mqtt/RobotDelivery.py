# RobotDelivery.py
#
# @author Emima Jiva. (c) ai2-UPV Todos los derechos reservados.
#
# Proyecto "WASHCARROB"
# Version: 1.0
# Rev: 2023. cambios introducidos 

import rmf_dispenser_msgs.msg as dispenser_msgs

import rclpy
import time
from rclpy.node import Node
from functools import partial

DISPENSER_RESULT_ACKNOWLEDGED = 0
DISPENSER_RESULT_SUCCESS = 1
DISPENSER_RESULT_FAILED = 2

DISPENSER_STATE_IDLE = 0
DISPENSER_STATE_BUSY = 1
DISPENSER_STATE_OFFLINE = 2

class Dispenser(Node):
    def __init__(self, name):
        super().__init__(name)

        # Variables
        self.reset(name)
        self.request_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            '/dispenser_requests',
            self._process_request_cb,
            1
        )
        self.result_pub = self.create_publisher(
            dispenser_msgs.DispenserResult,
            '/dispenser_results',
            1
        )        
    def reset(self, name=None):
        if name is not None:
            self.name = name

        self.status = {}
        self.success_flag = False

        try:
            self.timer.reset()
            self.timer.cancel()
        except Exception:
            pass

        self.timer = None        
    
    def _process_request_cb(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        if self.status.get(msg.request_guid) is None:
            self.status[msg.request_guid] = DISPENSER_RESULT_ACKNOWLEDGED
            self.wait_robot()
            self.status[msg.request_guid] = DISPENSER_RESULT_SUCCESS
            result = dispenser_msgs.DispenserResult()
            result.time = self.get_clock().now().to_msg()
            result.status = self.status[msg.request_guid]
            result.source_guid = self.name
            result.request_guid = msg.request_guid
            self.result_pub.publish(result)
        elif self.status[msg.request_guid] == DISPENSER_RESULT_SUCCESS:
            self.status[msg.request_guid] = DISPENSER_RESULT_ACKNOWLEDGED
            self.wait_robot()
            self.status[msg.request_guid] = DISPENSER_RESULT_SUCCESS
            result = dispenser_msgs.DispenserResult()
            result.time = self.get_clock().now().to_msg()
            result.status = self.status[msg.request_guid]
            result.source_guid = self.name
            result.request_guid = msg.request_guid
            self.result_pub.publish(result)
        else: 
            self.status[msg.request_guid] =  DISPENSER_RESULT_ACKNOWLEDGED
            self.get_logger().info('Aknowledged')

        # if msg.target_guid != self.name:
        #     return self.get_logger().info('Different name')
        # self.get_logger().info('REQUEST RECEIVED')
        # self.tasks[msg.request_guid] = True
        # self.status [msg.request_guid]
        # if self.prevstatus.get[msg.request_guid] is not None and \
        #     self.prevstatus[msg.request_guid] != self.status[msg.request_guid]:
        #     self.get_logger().info('Hacer lo que queramos')
        #     self.prevstatus[msg.request_guid] = self.status[msg.request_guid]
        # else:
        #     self.prevstatus[msg.request_guid] = self.status[msg.request_guid]

        # if self.tasks.get(msg.request_guid):
        #     status = DISPENSER_RESULT_SUCCESS
        #     self.get_logger().info('Status "%d"' % status)
        # else:
        #     self.tasks[msg.request_guid] = False
        #     status = DISPENSER_RESULT_ACKNOWLEDGED
        #     self.get_logger().info('Status "%d"' % status)
        #     self.timer = self.create_timer(
        #         1,
        #         partial(self._timer_cb, msg=msg)
        #     )
        # result = dispenser_msgs.DispenserResult()
        # result.time = self.get_clock().now().to_msg()
        # result.status = status
        # result.source_guid = self.name
        # result.request_guid = msg.request_guid

        # self.get_logger().info('Publish "%d"' % result.status)
        # self.result_pub.publish(result)

    def _timer_cb(self, msg):
        if not self.timer:
            return

        result = dispenser_msgs.DispenserResult()
        result.time = self.get_clock().now().to_msg()
        result.status = DISPENSER_RESULT_SUCCESS
        result.source_guid = self.name
        result.request_guid = msg.request_guid

        self.get_logger().info('Publish timer "%d"' % result.status)
        self.result_pub.publish(result)
        self.success_flag = True
        self.tasks[msg.request_guid] = True
        self.timer.cancel()
    
    def wait_robot(self):
        self.get_logger().info('Hacemos algo')
        time.sleep(5)