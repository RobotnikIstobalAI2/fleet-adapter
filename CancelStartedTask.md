# Cancel a Started Task

To cancel tasks that have been started from the console, either of the following two commands can be executed:

1. ```bash
   docker exec -it open-rmf-fleet-adapter-1 bash
   ros2 run robotnik_rmf_fleet_adapter_mqtt cancel_task -id <task_id>
   ```

2. ```bash
   docker exec -it open-rmf-fleet-adapter-1 bash
   ros2 run robotnik_rmf_fleet_adapter_mqtt cancel_task --task_id <task_id>
   ```

**NOTE:** The `<task_id>` can be obtained by calling the ROS2 service `/get_dispatches` with the following command:

```bash
ros2 service call /get_dispatches rmf_task_msgs/srv/GetDispatchStates "{}"
```

## Example

Example to show how to cancel the task with id `patrol.dispatch-0`:

```bash
docker exec -it open-rmf-fleet-adapter-1 bash
ros2 run robotnik_rmf_fleet_adapter_mqtt cancel_task -id patrol.dispatch-0
```
