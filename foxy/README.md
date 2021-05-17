# Use the docker container to run the ros-model plugin for HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image, for your desired ROS distro version:
```
cd path-to-ros-model-extrcators-repo
[sudo] docker build --tag=haros_foxy -f foxy/Dockerfile .
```

Call the ros-model extractor plugin, remember you have to also clone the repository to be analysed:

```
[sudo] docker run -it haros_foxy:latest /haros_runner_ros2.sh *package_name* *node_name* *type* *path_to_resulted_model* *workspace_path* *github_repository* *branch*
```

For example:

```
[sudo] docker run -it haros_foxy:latest /haros_runner.sh sick_safetyscanners2 sick_safetyscanners2_node node . /root/ws  https://github.com/SICKAG/sick_safetyscanners2

[sudo] docker run -it haros_foxy:latest /haros_runner.sh turtlesim turtlesim_node node . /root/ws https://github.com/ros/ros_tutorials foxy-devel
```

