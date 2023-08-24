#!/usr/bin/env python
import sys
import os
import subprocess
from datetime import datetime
import shutil
import glob


from ros2runner import ros2Runner

   
def main():

    pkgName = 'turtlesim'
    NodeName = '--all'
    typeOfRequest = 'node'
    pathToOutput =  "/home/divya/ros2_ws/runner_op"
    pathToROSws = "/home/divya/ros2_ws"
    gitRepo = "https://github.com/ros/ros_tutorials/ -b humble-devel"

    # print("pkgName:", pkgName)
    # print("NodeName:", NodeName)
    # print("typeOfRequest:", typeOfRequest)
    # print("pathToOutput:", pathToOutput)
    # print("pathToROSws:", pathToROSws)
    # print("gitRepo:", gitRepo)
   
   
    ros2Extractor = ros2Runner(inputPkgName=pkgName,
                        inputNodeName=NodeName,
                        typeOfRequest=typeOfRequest,
                        pathToROSws=pathToROSws,
                        gitRepo=gitRepo,
                        outputDir=pathToOutput)
    ros2Extractor.extractorRun()
   

main()