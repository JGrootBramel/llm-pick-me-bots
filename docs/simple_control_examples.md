# Simple control examples

This file gives some simple examples how to control  the limo cobot sim from the [quickstart guide](quickstart.md)

## 1 drive the base (Gazebo mobile base) via /cmd_vel
First confirm the message type:
```bash
rostopic type /cmd_vel
```
It should be `geometry_msgs/Twist`. Then drive forward:
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
'
```
Stop movement:
```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
'
```

## Move the arm using the FollowJointTrajectory action (recommended)
This is the “standard” way when you see `/follow_joint_trajectory/goal`.
```bash
rostopic echo -n 1 /arm_controller/state
```

```bash
rostopic pub -1 /arm_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "
goal:
  trajectory:
    joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6']
    points:
    - positions: [0.0, -0.6, 0.8, 0.0, 0.6, 0.0]
      velocities: [0,0,0,0,0,0]
      time_from_start: {secs: 2, nsecs: 0}
"
```
