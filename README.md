# Store Battery

Small ROS2 script that lets your robot rotate on the spot until the battery is below a certain level.

You can then use the storage program of your charger to balance and store your batteries.

## WARNING
This uses the velocity command of your robot/the motors to discharge the batteries faster!

Once the wheels stop turning the battery should be at storage level, shut down the robot then.

# Installation
copy to your colcon-workspace `src/`-folder and run:

```bash
# ```~/ros_ws
colcon build
```
and re-source your environment

```bash
source install/setup.bash
```

# Run
```bash
ROS_DOMAIN_ID=31 ros2 run store_battery lower_voltage
```