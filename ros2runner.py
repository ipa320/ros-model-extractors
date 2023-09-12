#!/usr/bin/env python
import sys
import os
import subprocess
from datetime import datetime
import shutil
from ros_pkg_runner import rosPkgRunner
from file_handling import fileHandling
import glob



class ros2Runner(rosPkgRunner):
   
    ROS2setupBashFile = "install/setup.bash"
    ROS2Version = 2
    ROS2PythonVer = 3
   
   
    def __init__(self,
                inputPkgName:str,
                inputNodeName:str,
                typeOfRequest:str,
                pathToROSws:str,
                gitRepo:str,
                outputDir:str):
       
        super().__init__(inputPkgName=inputPkgName,
                        inputNodeName=inputNodeName,
                        typeOfRequest=typeOfRequest,
                        pathToROSws=pathToROSws,
                        gitRepo=gitRepo,
                        setupBashFile=self.ROS2setupBashFile,
                        outputDir=outputDir)

    def runColconBuild(self,):
        subprocess.run(["colcon", "build", "--cmake-args", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"])
       
    def runColconList(self,):
        os.system('colcon list > /tmp/colcon_list.txt')

           
   
    def calcPathToPkgSRC(self, textList:str):
       
        os.system('cat /tmp/colcon_list.txt |  grep "^$1" | awk \'{ print $2}\' > /tmp/colcon_list_update.txt')
        path_to_src_code=subprocess.run(["cat","/tmp/colcon_list_update.txt"], stdout=subprocess.PIPE, text=True).stdout
        path_to_src_code=path_to_src_code.split("\n")
       
        self.setPathToSRCcode([os.path.join(self.getPathToROSws(), item) for item in path_to_src_code])    
        self.validatePathToSRCcodeList()
   
    def startInstallPkgProcedure(self):
       
        self.runCommonCmd()
        self.runColconBuild()
        self.runSetupBash()
        colconList = self.runColconList()
        self.calcPathToPkgSRC(colconList)
        print("## ROS pkg install and  Build complete")
       
    def callHarosPlugin(self,):
       
        self.harosInit()

       
        for pkgName in self.getPathToSRCcode():
            currOutDir = os.path.join(self.outputDir, pkgName.split('/')[-1])
            fileName = os.path.join(os.path.dirname(__file__), self.getROSModelExtractorPluginFile())
            extractorFile = currOutDir+"/extractor.log"
            extractorContent = None
           
            if(not os.path.isdir(currOutDir)):
                os.makedirs(currOutDir)

            if self.getInputNodeName() == "--all":
                extractorContent = subprocess.run(["python3", fileName,
                                "--package", str(self.getInputPkgName()),
                                "--"+str(self.getTypeOfRequest()),
                                "--model-path",str(currOutDir) ,
                                "--ws", str(self.getPathToROSws()),
                                "--path-to-src", pkgName,
                                "--repo", str(self.getGitModelRepo()),
                                "-a"], stdout=subprocess.PIPE, text=True).stdout
            else:
                extractorContent = subprocess.run(["python3", fileName,
                                "--package", str(self.getInputPkgName()),
                                "--name", str(self.getInputNodeName()),
                                "--"+str(self.getTypeOfRequest()),
                                "--model-path",str(currOutDir),
                                "--ws", str(self.getPathToROSws()),
                                "--path-to-src", pkgName,
                                "--repo", str(self.getGitModelRepo())
                                ], stdout=subprocess.PIPE, text=True).stdout
            fileHandling.wrtieToFile(extractorFile, extractorContent)
       
    def extractorRun(self,clearFlag=False):
       

        #1. clone the git repo at the required
        rosPkgRunner.cloneRepo(self.getPathToROSws(), self.getGitRepo())
       
        #2.  Change working directory to ROS ws
        os.chdir(self.getPathToROSws())
       
        #3. Install dependencies and build
        self.startInstallPkgProcedure()
       
        #4. Init the plugin
        self.callHarosPlugin()
       
        #5. Show the ros models
        self.showAllROSmodel()
       
        #6. Clear the ROS ws SRC folder
        if clearFlag:
            self.cleanUPsrc()        
       
        print( "------------------Done---------------")