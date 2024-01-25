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