#! /usr/bin/python

import os, time, sys, datetime, threading
import numpy as np
import cv2
#from cv_bridge import CvBridge, CvBridgeError
import rosbag
import roslaunch, rospy
from sensor_msgs.msg import CompressedImage

# play MT bagfile, retrack rotated image with kineflyMT.py, to get head (and wing) angles
# write published topics to a bagfile; include /kinefly_flystate

class MainWindow:
    def __init__(self,bagDir='/home/rancher/bagfiles/toTrack'):
        saveBag = True # TO JUST UPDATE YAMLS, COMMENT THIS LINE
        play_bag_speed = .75
        try:
          saveBag
        except NameError:
          print 'not saving video'
          print ' '
          saveBag = False
        t = time.time()
        bagFiles = os.listdir(bagDir)
        bagFiles.sort()
        for file in bagFiles:
            if file.endswith('.bag'):
                #if float(file[:6])<2004
                ## check the latest time stamp of the compressed image stream
                self.inbag = rosbag.Bag(bagDir + '/' +file)
                self.bag_end_time =  self.inbag.get_end_time() - self.inbag.get_start_time()
                ## Find topics and types
                #topics = self.inbag.get_type_and_topic_info()[1].keys()
                #types = []
                #for i in range(0,len(self.inbag.get_type_and_topic_info()[1].values())):
                #    types.append(self.inbag.get_type_and_topic_info()[1].values()[i][0])
                self.nodeName = file[1:20]
                args2 = 'playBagSp:=' + str(play_bag_speed)
                if saveBag:
                    outBag = file[:-22] + 'b' + file[-21:] # path specified in launch file
                    #launchDir = "/home/rancher/catkin/src/Kinefly/launch/mainMTkf_b.launch" # saves all image topics too (delete once you dont' need from _recordMTkf.launch)
                    launchDir = "/home/rancher/catkin/src/Kinefly/launch/mainMTkf_b_sml.launch"
                    args3 = 'outBag:=' + outBag
                    print 'patiently waiting for the tracker..... '
                else:
                    launchDir = "/home/rancher/catkin/src/Kinefly/launch/mainMTkf.launch"
                    #pFix = datetime.datetime.today().strftime('%Y-%m-%d-%H-%M-%S')
                args1 = 'inBag:=' + bagDir + '/' + file
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) 
                roslaunch.configure_logging(uuid) 
                sys.argv.append(args1)
                sys.argv.append(args2)
                if saveBag:
                    sys.argv.append(args3)
                self.parent = roslaunch.parent.ROSLaunchParent(uuid, [launchDir]) 
                rospy.on_shutdown(self.parent.shutdown)
                self.parent.start() # ipython command equivalent to "roslaunch magnotether recordMT.launch"
                print 'give roslaunch 4 s to initiate; ' + args2
                time.sleep(4)
                if saveBag:
                    print 'started to track  ' + file[:-4].replace("_", " ") + ' at: ' + str(datetime.datetime.now())[11:19]
                    tracking_time = self.bag_end_time/play_bag_speed
                    completeTime = str(datetime.datetime.now()+datetime.timedelta(seconds=tracking_time))[11:19]
                    rospy.logwarn ('expect to complete this trial at ' + completeTime )
                    time.sleep(tracking_time)
                else:
                    print 'adjust tracking regions for  ' + file[:-4].replace("_", " ") + ' at: ' + str(datetime.datetime.now())[11:19]
                    raw_input('press Enter to continue...')
                print 'shut down kineflyMT and close bagfiles'
                self.parent.shutdown() ##vid # ipython command equivalent to Ctrl-C for "recordMT.launch"
                print 'wait a few seconds, then move on to the next bag file'
                time.sleep(2.)
        print 'finished.'
        elapsed = time.time() - t
        print 'total processing time [s]:' + str(elapsed)[0:5]

if __name__ == '__main__':
    main = MainWindow()
