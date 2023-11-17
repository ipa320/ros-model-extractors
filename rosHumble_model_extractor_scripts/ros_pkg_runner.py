#!/usr/bin/env python
import sys
import os
import subprocess
from datetime import datetime
import shutil
from file_handling import fileHandling
import glob

class rosPkgRunner:
   
   
   
    def __init__(self,  
                 inputPkgName:str,
                 inputNodeName:str,
                 typeOfRequest:str,
                 outputDir:str,
                 pathToROSws:str,
                 gitRepo:str,
                 setupBashFile:str):
       
        self.setupBashFile = setupBashFile
        self.outputDir = outputDir
       
        # Private attr
        self._inputPkgName = inputPkgName
        self._inputNodeName = inputNodeName
        self._typeOfRequest = typeOfRequest
        self._pathToROSws = pathToROSws
        self._gitRepo = gitRepo
        self._pathToSRCcode = None
       
        self._ROS_model_extractor_plugin_file = "rosHumble_model_extractor.py"
             
        self.mkOuputDir()
       
       
    def getROSModelExtractorPluginFile(self,):
        return self._ROS_model_extractor_plugin_file
   
    def getInputPkgName(self,):
        return self._inputPkgName
   
    def getInputNodeName(self,):
        return self._inputNodeName
   
    def getTypeOfRequest(self,):
        return self._typeOfRequest
   
    def getPathToROSws(self,):
        return self._pathToROSws
   
    def getGitRepo(self,):
        return self._gitRepo
   
    def runSetupBash(self,):
        if(os.path.isfile(self.setupBashFile)):
            filePath = os.path.join(self.getPathToROSws(), self.setupBashFile)
            subprocess.run(['source', filePath], executable="/bin/bash")

    def getPathToSRCcode(self,):
        return self._pathToSRCcode
   
    def setPathToSRCcode(self, newValue):
        self._pathToSRCcode = newValue
       
    def getGitModelRepo(self):
        return self.getGitRepo().split(' ')[0]
   
       
    def runROSdepInstall(self,):
        subprocess.run(["rosdep", "install", "-y", "-i", "-r", "--from-path", "src"])
       
    def runCommonCmd(self,):
        self.runSetupBash()
        self.runROSdepInstall()
       
       
    def startInstallProcedure(self,):
        print("Add additoinal install procedure for respective ROS version")
       
    def harosInit(self,):
        print("## Init HAROS ##")
        subprocess.run(["haros","init"])
       
    def finishedMsg(self,extractorFile):
        print("~~~~~~~~~~~")
        print("Extraction finished. See the following report:")
        subprocess.run(["cat", extractorFile])
        print("~~~~~~~~~~~\n\n")
       
    def showROSmodel(self, modelFile):
        fileHandling.showFileContent(modelFile)

   
    def showAllROSmodel(self,):
       
        fileList = fileHandling.filterFileNamesByExtension(self.outputDir, 'ros')
       
        for fileName in fileList:
            self.showROSmodel(fileName)
       
    def cleanUPsrc(self):
       
        pathToCheck = rosPkgRunner.subDirList(os.path.join(self.getPathToROSws(), 'src'))
        for dirs in pathToCheck:
            print( "Delete: ", dirs)
            shutil.rmtree(dirs)
       
    def callHarosPlugin(self,):
       
        print("Update code to execute harosPlugin command!!")
   
    def mkOuputDir(self,):
       
        current_timestamp = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
        current_timestamp = current_timestamp.split(' ')
        output_dir = os.path.join(self.outputDir, current_timestamp[0], current_timestamp[-1])
       
        if(not os.path.isdir(output_dir)):
            os.makedirs(output_dir)

        self.outputDir = output_dir
        print("## Output Directory Created at : {} ##".format(output_dir))
       
    def extractorRun(self,):
        print(" Write the procedure to run the model extractor file with configuration:")
   
    @staticmethod
    def checkIfValidPkgDir(dirToCheck:str, fileToCheck='package.xml' ):
        fileList = os.listdir(dirToCheck)
        fileList = [f for f in fileList if os.path.isfile(dirToCheck+'/'+f)]
       
        numOfFileToCheck = fileList.count(fileToCheck)
        if numOfFileToCheck == 1:
            return True
        else:
            if numOfFileToCheck > 0:
                print("### More than one {} found in {}, rejected as valid pakage directory ###".format(fileToCheck, dirToCheck))
            return False 
       
    def validatePathToSRCcodeList(self,):
        newList = []
        for dirName in self.getPathToSRCcode():
            if rosPkgRunner.checkIfValidPkgDir(dirName):
                newList.append(dirName)
           
        self.setPathToSRCcode(newList)
       
    @staticmethod
    def cloneRepo(ws_dir:str, repo_url:str):
        clone_cmd = ["git", "clone" ]

        src_dir = os.path.join(ws_dir, "src/")
        print(ws_dir, repo_url,  type(ws_dir), src_dir)
        os.chdir(src_dir)
        repo_url = repo_url.split(' ')
        clone_cmd = clone_cmd + repo_url
        subprocess.run(clone_cmd)
        print("Repository cloned successfully.")
        os.chdir(ws_dir)
       
   
   
    @staticmethod 
    def subDirList(parent_dir):
        subdirs = [os.path.join(parent_dir, d) for d in os.listdir(parent_dir) if os.path.isdir(os.path.join(parent_dir, d))]
        return subdirs