### Command lines to auto-generate the examples of this folder

```
docker run -it haros_melodic:latest /haros_runner.sh hokuyo_node hokuyo_node node . /root/ws https://github.com/ros-drivers/hokuyo_node

docker run -it haros_melodic:latest /haros_runner.sh ridgeback_base ridgeback_node  node . /root/ws https://github.com/ridgeback/ridgeback_robot https://github.com/clearpathrobotics/puma_motor_driver

docker run -it haros_melodic:latest /haros_runner.sh cob_sick_s300 cob_sick_s300 node . /root/ws https://github.com/ipa320/cob_driver

docker run -it haros_melodic:latest /haros_runner.sh cob_light cob_light node . /root/ws https://github.com/ipa320/cob_driver

docker run -it haros_melodic:latest /haros_runner.sh cob_light cob_light node . /root/ws https://github.com/ipa320/cob_driver

docker run -it haros_melodic:latest /haros_runner.sh ur_robot_driver ur_robot_driver_node node . /root/ws  "https://github.com/UniversalRobots/Universal_Robots_ROS_Driver" "https://github.com/ros-industrial/universal_robot.git -b melodic-devel-staging"
```
