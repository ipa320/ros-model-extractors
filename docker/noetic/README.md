# Use the docker container to run the ros-model plugin for HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image, for your desired ROS distro version:
```
cd path-to-ros-model-extractors-repo
[sudo] docker build --tag=haros_noetic -f noetic/Dockerfile .
```

Call the ros-model extractor plugin, remember you have to also clone the repository to be analysed:

```
[sudo] docker run -it haros_noetic:latest /haros_runner.sh *package_name* *node_name* *type* *path_to_resulted_model* *workspace_path* "*github_repositoryA -b branch*" "*github_repositoryB*"...
```

For example:

```
[sudo] docker run -it haros_noetic:latest /haros_runner.sh cob_sick_s300 cob_sick_s300 node . /home/extractor/ws https://github.com/ipa320/cob_driver

[sudo] docker run -it haros_noetic:latest /haros_runner.sh test_pkg test_node node . /home/extractor/ws "https://github.com/ipa-nhg/test_ros_code_extractor -b ros1Parameters"
```

