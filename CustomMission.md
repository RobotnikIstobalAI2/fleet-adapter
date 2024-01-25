# Custom Mission

Launch custom mission. The currently scheduled task performs the following:
1.	The robot is positioned to the start point with the corresponding orientation. Both the start point and the orientation are user-configurable. 
2.	It enters the perform_action state where it turns on itself for a time and then exits from that state.
3.	When the perfom_action is finished, it goes to point_2 where the task is terminated and the robot can continue doing any other task.

## Launch custom mission

```bash
docker exec -it open-rmf-fleet-adapter-1 bash
ros2 run fleet_adapter_mqtt dispatch_action -s <start_point> -a teleop -F <fleet_name> -R <robot_name> -o <orientation>
```
The fleet_name and robot_name are optional. If they are not specified, rmf-core will choose which robot to use for the mission.
The orientation is the orientation of the start_point.

## Building a Custom Task

Users can build and send their own tasks by publishing <a href="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_task_msgs/msg/ApiRequest.msg">ApiRequest</a> messages. You will need to fill in the **request_id** and **json_msg** fields according to the types of phases that make up the task, as well as whether the task is intended for a specific robot or the best available fleet. You may follow these steps to construct your own task:

  1.  Create an **ApiRequest** publisher that sends task requests via the **/task_api_requests** topic.
  2.  Fill in the request_id field with a unique string ID that can be used to identify the task.
  3.  For the json_msg field,
        Use the <a href="https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/robot_task_request.json">robot_task_request</a> schema and fill in the JSON payload type with "robot_task_request" to send a task request to a specific robot
        Use the <a href="https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/dispatch_task_request.json">dispatch_task_request</a>  schema and fill in the JSON payload type with "dispatch_task_request" to send a task request to the best available fleet
        The request fields for these objects follow the <a href="https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/task_request.json">task_request</a> schema
    4. Populate the object fields with the required information.
        The **category** and **description** fields under the **task_request** schema take in the string name of the task and the task description respectively. The JSON schema for these descriptions can be found <a href="https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas">here</a>.
    5. Publish the **ApiRequest!**
