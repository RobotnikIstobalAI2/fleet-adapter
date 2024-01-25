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

Users can build and send their own tasks by publishing <a href="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_task_msgs/msg/ApiRequest.msg">ApiRequest/</a> messages. You will need to fill in the request_id and json_msg fields according to the types of phases that make up the task, as well as whether the task is intended for a specific robot or the best available fleet. You may follow these steps to construct your own task:

    Create an ApiRequest publisher that sends task requests via the /task_api_requests topic.
    Fill in the request_id field with a unique string ID that can be used to identify the task.
    For the json_msg field,
        Use the robot_task_request schema and fill in the JSON payload type with "robot_task_request" to send a task request to a specific robot
        Use the dispatch_task_request schema and fill in the JSON payload type with "dispatch_task_request" to send a task request to the best available fleet
        The request fields for these objects follow the task_request schema
    Populate the object fields with the required information.
        The category and description fields under the task_request schema take in the string name of the task and the task description respectively. The JSON schema for these descriptions can be found here. There are currently four task descriptions available:
            Clean: create your own clean task, requires the Clean phase description
            Compose: create your own custom task that may comprise of a sequence of phases, requires descriptions for the relevant phases
            Delivery: create your own delivery task, requires the PickUp and DropOff phase descriptions
            Patrol: create your own patrol task, requires the Place description to indicate where you would like your robot to go to
    Publish the ApiRequest!
