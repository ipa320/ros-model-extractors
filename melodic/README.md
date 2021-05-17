# Use the docker container to run the ros-model plugin for HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image, for your desired ROS distro version:
```
cd path-to-ros-model-extrcators-repo
[sudo] docker build --tag=haros_melodic -f melodic/Dockerfile .
```

Call the ros-model extractor plugin, remember you have to also clone the repository to be analysed:

```
[sudo] docker run -it haros_melodic:latest /haros_runner.sh *package_name* *node_name* *type* *path_to_resulted_model* *workspace_path* *github_repository* *branch*
```

For example:

```
[sudo] docker run -it haros_melodic:latest /haros_runner.sh hokuyo_node hokuyo_node node . /root/ws https://github.com/ros-drivers/hokuyo_node
```

