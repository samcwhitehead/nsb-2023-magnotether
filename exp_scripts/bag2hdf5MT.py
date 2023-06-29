#! /usr/bin/python
import os, time, sys, datetime
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import rosbag
import numpy as np
import h5py

class MainWindow:
    def __init__(self,bagDir='/home/rancher/bagfiles/toConvert'):
        #recVid = True 
        try:
          recVid
        except NameError:
          print 'not saving video'
          print ' '
          recVid = False
        self.cv_bridge = CvBridge()
        t = time.time()
        bagFiles = os.listdir(bagDir)
        bagFiles.sort()
        for file in bagFiles:
            if file.endswith('.bag'):
                print 'running: ' +  file[:-4].replace("_", " ")
                self.inbag = rosbag.Bag(bagDir + '/' + file)
                ## Find topics (and types)
                self.TandTinfo = self.inbag.get_type_and_topic_info()
                ## Initialize hdf5 file
                if recVid:
                    hdf5_file_name = bagDir + '/' + file[:-5] + 'V.hdf5'
                    self.h5f = h5py.File(hdf5_file_name,'w')
                    ## Read topics and save to hdf5 file
                    self.read_compressed_images('/camera/image_raw/compressed')
                    self.read_rotated_images('/rotated_image')
                    self.h5f.create_dataset ('recVid',data=True)
                else:
                    hdf5_file_name = bagDir + '/' + file[:-4] + '.hdf5'
                    self.h5f = h5py.File(hdf5_file_name,'w')
                    self.h5f.create_dataset ('recVid',data=False)
                ## Read topics and save to hdf5 file
                self.read_angle_data                    ('/angle_data')
                self.read_azimuth_data                  ('/azimuth_data')
                self.read_aout_data                     ('/phidgets_aout_dat')
                # comment following line if directly from MT, w.o. tracking
                self.read_kinefly_angles                ('/kinefly/flystate')
                self.h5f.close()
                print ' '
        print 'Finished.'
        elapsed = time.time() - t
        print 'total processing time [s]:' + str(elapsed)[0:5]

    def read_compressed_images(self,topic_name):
        ros_tstampsIm   = [np.float(msg[2].to_time())        for msg in self.inbag.read_messages(topics = topic_name)]
        img_np_arr      = [np.fromstring(msg[1].data, np.uint8) for msg in self.inbag.read_messages(topics = topic_name)]
        initImage       = cv2.imdecode(img_np_arr[0],cv2.IMREAD_GRAYSCALE)
        images          = np.zeros( ( np.shape(img_np_arr)[0], initImage.shape[0], initImage.shape[1] ), dtype='uint8' )
        print 'decoding magnotether raw images'        
        for i in range(np.shape(img_np_arr)[0]): # in the range of number of frames recorded
            images[i,:,:] = cv2.imdecode(img_np_arr[i],cv2.IMREAD_GRAYSCALE)
        print 'writing images to hdf5'
        self.h5f.create_dataset('raw_ros_tstamps',data=ros_tstampsIm)
        self.h5f.create_dataset('raw_images',     data=images,compression='gzip')

    def read_rotated_images(self,topic_name): 
        ros_tstampsIm = [np.float(msg[2].to_time())                   for msg in self.inbag.read_messages(topics = topic_name)]
        img_ls = [self.cv_bridge.imgmsg_to_cv2(msg[1], 'passthrough') for msg in self.inbag.read_messages(topics = topic_name)]
        images = np.asarray(img_ls)
        print 'writing rotated images to hdf5'
        self.h5f.create_dataset('rot_ros_tstamps',data=ros_tstampsIm)
        self.h5f.create_dataset('rot_images',     data=images,compression='gzip')

    def read_kineflyOutput_images(self,topic_name):
        ros_tstampsIm = [np.float(msg[2].to_time()) for msg in self.inbag.read_messages(topics = topic_name)]
        img_ls = [self.cv_bridge.imgmsg_to_cv2(msg[1], 'passthrough') for msg in self.inbag.read_messages(topics = topic_name)]
        images = np.asarray(img_ls)
        print 'writing kinefly images to hdf5'
        self.h5f.create_dataset('kf_ros_tstamps',data=ros_tstampsIm)
        self.h5f.create_dataset('kf_images',data=images,compression='gzip')
 
    def read_angle_data(self,topic_name):
        ros_tstampsAng = [msg[2].to_time() for msg in self.inbag.read_messages(topics = topic_name)]
        frame = [msg[1].frame for msg in self.inbag.read_messages(topics = topic_name)]
        angle = [msg[1].angle for msg in self.inbag.read_messages(topics = topic_name)]
        print 'writing magnotether angles to hdf5'
        self.h5f.create_dataset('ros_tstampsAng',data=ros_tstampsAng)
        self.h5f.create_dataset('frame',data=frame)
        self.h5f.create_dataset('angle',data=angle)

    def read_azimuth_data(self,topic_name):
        ros_tstampsAzV = [msg[2].to_time() for msg in self.inbag.read_messages(topics = topic_name)]
        azV = [msg[1].azimuth_voltage for msg in self.inbag.read_messages(topics = topic_name)]
        print 'writing azimuth voltage to hdf5'
        self.h5f.create_dataset('ros_tstampsAzV',data=ros_tstampsAzV)
        self.h5f.create_dataset('azV',data=azV)

    def read_aout_data(self,topic_name):
        ros_tstampsAOut = [msg[2].to_time() for msg in self.inbag.read_messages(topics = topic_name)]
        aOutOne = [msg[1].voltage[0] for msg in self.inbag.read_messages(topics = topic_name)]
        aOutTwo = [msg[1].voltage[1] for msg in self.inbag.read_messages(topics = topic_name)]
        aOutThr = [msg[1].voltage[2] for msg in self.inbag.read_messages(topics = topic_name)]
        aOutFou = [msg[1].voltage[3] for msg in self.inbag.read_messages(topics = topic_name)]
        print 'writing aout data to hdf5'
        self.h5f.create_dataset('ros_tstampsAOut',data=ros_tstampsAOut)
        self.h5f.create_dataset('aOutOne' ,data=aOutOne)
        self.h5f.create_dataset('aOutTwo' ,data=aOutTwo)
        self.h5f.create_dataset('aOutThr' ,data=aOutThr)
        self.h5f.create_dataset('aOutFou' ,data=aOutFou)

    def read_kinefly_angles(self,topic_name):
        ros_TAng = [msg[2].to_time() for msg in self.inbag.read_messages(topics = topic_name) if len(msg[1].left.angles)>0 and len(msg[1].right.angles)>0 and len(msg[1].head.angles)>0]
        LA = [msg[1].left.angles[0]  for msg in self.inbag.read_messages(topics = topic_name) if len(msg[1].left.angles)>0 and len(msg[1].right.angles)>0 and len(msg[1].head.angles)>0]
        RA = [msg[1].right.angles[0] for msg in self.inbag.read_messages(topics = topic_name) if len(msg[1].left.angles)>0 and len(msg[1].right.angles)>0 and len(msg[1].head.angles)>0]
        HA = [msg[1].head.angles[0]  for msg in self.inbag.read_messages(topics = topic_name) if len(msg[1].left.angles)>0 and len(msg[1].right.angles)>0 and len(msg[1].head.angles)>0]
        AU = [msg[1].aux.angles[0]   for msg in self.inbag.read_messages(topics = topic_name) if len(msg[1].left.angles)>0 and len(msg[1].right.angles)>0 and len(msg[1].head.angles)>0]
        print 'writing kinefly angles to hdf5'
        self.h5f.create_dataset('ros_tstampsKf',data=ros_TAng)
        self.h5f.create_dataset('LA',data=LA)
        self.h5f.create_dataset('RA',data=RA)
        self.h5f.create_dataset('HA',data=HA)
        self.h5f.create_dataset('AU',data=AU)

if __name__ == '__main__':
    main = MainWindow()
