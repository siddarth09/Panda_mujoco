# Panda_mujoco
Mujoco ROS2 implementation of FRANK PANDA arm using mujoco_ros2_control


### TO USE SO101

```bash
ros2 run panda_mujoco so101.py \
  --target ur5 \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1
```

SO101 leader arm teleoperation 

https://github.com/user-attachments/assets/7dd918b9-a046-485d-8769-bb15b1da8d89


Cube detection and Classical Approach of pick and place (Yeah, the gripper isn't working in this, and the cubes are not having enough friction )
Issue being addressed #4

https://github.com/user-attachments/assets/cb84c81d-7aef-4896-9d8d-96ddce69c01b


Teleoperation using a Gamepad controller!

https://github.com/user-attachments/assets/28bc4c80-a9f7-4159-9606-bce1e6056b57

