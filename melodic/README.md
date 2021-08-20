# Use the docker container to run the ros-model plugin for HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image, for your desired ROS distro version:
```
cd path-to-ros-model-extractors-repo
[sudo] docker build --tag=haros_melodic -f melodic/Dockerfile .
```

Call the ros-model extractor plugin, remember you have to also clone the repository to be analysed:

```
[sudo] docker run -it haros_melodic:latest /haros_runner.sh *package_name* *node_name* *type* *path_to_resulted_model* *workspace_path* "*github_repositoryA -b branch*" "*github_repositoryB*"...
```

For example:

```
[sudo] docker run -it haros_melodic:latest /haros_runner.sh hokuyo_node hokuyo_node node . /home/extractor/ws https://github.com/ros-drivers/hokuyo_node

[sudo] docker run -it haros_melodic:latest /haros_runner.sh ridgeback_base ridgeback_node node . /home/extractor/ws https://github.com/ridgeback/ridgeback_robot https://github.com/clearpathrobotics/puma_motor_driver
```

