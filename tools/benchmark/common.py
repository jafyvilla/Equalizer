#!/usr/bin/python

import os
import checkXServersAndRestart

numberOfServers = 12
excludedServers = [ 5 ] 

# All test options
protocolsFull = [ 'TenGig', 'IPoIB', 'RDMA', 'SDP' ]  
affFullStateList = [  'GoodAffinity', 'BadAffinity', 'NoAffinity' ]
rtNeuronFullLayoutNames = [ 'RoundRobinDB', 'SpatialDB','Dynamic2D', 'DBDirectSend', 'DBDirectSendSDB', 'DBDirectSendRR' ]
eqPlyFullLayoutNames = [ 'StaticDB', 'Static2D', 'Dynamic2D' ]
roiFullStateList = [  'ROIEnabled', 'ROIDisabled' ]

eqPlyBinaryPath = '/home/bilgili/Build/bin/eqPly'
rtNeuromBinaryPath = '/home/bilgili/Build/bin/rtneuron.equalizer'
eqPlyDefaultArgs = '-m ~/EqualizerData/david1mm.ply -a ~/EqualizerConfigs/eqPly/cameraPath --eq-logfile node01.log'
rtNeuronDefaultArgs = '--profile --path-fps 10 -b /home/bilgili/RTNeuronData/blueconfig --no-sim-data  --no-lod --no-selections  --background 0.5 0.5 0.5 1.0 --path ~/RTNeuronData/camera_path.cam '

testFileName = "FPS.eqPly.txt"
gpuCountFile = "GPUCount.txt"
gpuCountFPSFile = "GPUCountFPS.txt"
gpuCountHFPSFile = "GPUCountHFPS.txt"
gpuCountLFPSFile = "GPUCountLFPS.txt"
commandFile = "CommandString"
rtneuronFPSFile = "statistics.txt"
optionsDumpFilename = "runoptions.obj"

timeSecToWaitForProcess = 20 * 60 # Wait for 20 minutes before killing process ( possible hang )

dirStack = []

class Configuration:
   dirName = ''
   protocol = 'TenGig'
   layoutName = 'Static2D'  
   roiState = 'ROIDisabled'
   affState = 'NoAffinity'
   protocol = 'TCP'
   session = ''
   nbOfFrames = 900
   serverCount = 1
   forceRedo = False
   checkAndRedo = False
    
def saveCurrentDir():
   dirStack.append( os.getcwd() )
   
def gotoPreviousDir():
   os.chdir( dirStack.pop() ) 

def writeCommandStringToFile( cmdStr ):
   f = open( commandFile, 'w' )
   f.write( cmdStr )
   f.close( )

def eqPlysingleTestScheme( application, function, serverCount ):

   # run tests options
   protocols = [ 'TenGig' ]  
   eqPlyLayoutNames = [ 'DBDirectSend' ]
   
   for protocol in protocols:
      for layoutName in eqPlyLayoutNames:

         roiState = "ROIDisabled"
         affState = "NoAffinity"
         
         config = Configuration()
         config.dirName = '%s-%s-%s-%s-%s' % ( application, protocol, layoutName, roiState, affState )
         config.protocol = protocol
         config.layoutName = layoutName
         config.roiState = roiState
         config.affState = affState
         config.session = '%s-%s-%s' % ( application, affState, protocol )
         config.serverCount = serverCount
         config.nbOfFrames = 1210
         function( config )


def eqPlyfullTestScheme( application, function, serverCount ):

   for protocol in protocolsFull:
      for layoutName in eqPlyFullLayoutNames:
         for roiState in roiFullStateList:
            for affState in affFullStateList:
               config = Configuration()
               config.dirName = '%s-%s-%s-%s-%s' % ( application, protocol, layoutName, roiState, affState )
               config.protocol = protocol
               config.layoutName = layoutName
               config.roiState = roiState
               config.affState = affState
               config.session = '%s-%s-%s' % ( application, affState, protocol )
               config.serverCount = serverCount
               config.nbOfFrames = 1210
               function( config )


def eqPlycombinationTestScheme( application, function, serverCount ):

   # run tests options
   protocols = [ 'TenGig' ]  
   eqPlyLayoutNames = [ 'Dynamic2D' ]
   roiStateList = [  'ROIEnabled' ]
   affStateList = [  'GoodAffinity',  'NoAffinity' ]
   
   for protocol in protocols:
      for layoutName in eqPlyLayoutNames:
         for roiState in roiStateList:
            for affState in affStateList:
               config = Configuration()
               config.dirName = '%s-%s-%s-%s-%s' % ( application, protocol, layoutName, roiState, affState )
               config.protocol = protocol
               config.layoutName = layoutName
               config.roiState = roiState
               config.affState = affState
               config.session = '%s-%s-%s' % ( application, affState, protocol )
               config.serverCount = serverCount
               config.nbOfFrames = 1210
               function( config )

def rtneuronsingleTestScheme( application, function, serverCount ):

   protocols = [ 'TenGig' ]  
   rtNeuronLayoutNames = [ 'Dynamic2D' ]
   
   for protocol in protocols:
      for layoutName in rtNeuronLayoutNames:

         roiState = "ROIDisabled"
         affState = "GoodAffinity"
         
         config = Configuration()
         config.dirName = '%s-%s-%s-%s-%s' % ( application, protocol, layoutName, roiState, affState )
         config.protocol = protocol
         config.layoutName = layoutName
         config.roiState = roiState
         config.affState = affState
         config.session = '%s-%s-%s' % ( application, affState, protocol )
         config.serverCount = serverCount
         config.nbOfFrames = 400
         function( config )


def rtneuroncombinationTestScheme( application, function, serverCount ):

   # run tests options
   protocols = [ 'TenGig' ]  
   rtNeuronLayoutNames = [ 'SpatialDB', 'RoundRobinDB','Dynamic2D' ]
   roiStateList = [  'ROIEnabled', 'ROIDisabled' ]
   affStateList = [  'GoodAffinity', 'BadAffinity', 'NoAffinity' ]

   for protocol in protocols:
     for layoutName in rtNeuronLayoutNames:
       for roiState in roiStateList:
          for affState in affStateList:
              config = Configuration()
              config.dirName = '%s-%s-%s-%s-%s' % ( application, protocol, layoutName, roiState, affState )
              config.protocol = protocol
              config.roiState = roiState
              config.layoutName = layoutName
              config.affState = affState
              config.session = '%s-%s-%s' % ( application, affState, protocol )
              config.serverCount = serverCount
              config.nbOfFrames = 400
              function( config )       


def testScheme( schemeName, application, function, serverCount ):

   functioName = '%s%sTestScheme' % ( application, schemeName )
   
   if( not globals().has_key( functioName ) ):
      print "No such schema function: " + functioName
      exit()
   
   schemeFunc = globals()[ functioName ]  
   schemeFunc( application, function, serverCount )
   

                
