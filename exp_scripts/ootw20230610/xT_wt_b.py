#!/usr/bin/env python
from panel_com import PanelCom
import os, time, sys, datetime
import numpy as np
from phidgets_aout_proxy import PhidgetsAoutProxy
trialNum = 1
rig = 'MT'
#rig = 'RT'
if rig == 'MT':
    AOexc = 1
    recLaunchDir = "/home/rancher/catkin/src/magnotether/launch/recordMT.launch"
    baseLevelPat = 5
    print 'Magno Tether'
elif rig == 'RT':
    AOexc = 0
    recLaunchDir = "/home/rancher/catkin/src/Kinefly/launch/recordRT.launch"
    baseLevelPat = 0
    print 'Rigid Tether'
else:
    print 'what rig are you using?' 

scriptNmXpPrt = os.path.basename(__file__)[3:5]
reCheck = True
#driver = ' S000'
#driver = ' OK371'
driver = ' XHCS'
#driver = ' S390'
#responder = ' U__X '
responder = ' wldT '

saveBag = True
## Set variable names and parameters
year =  datetime.datetime.today().strftime('%Y')[2:4]
month = datetime.datetime.today().strftime('%m')
day = datetime.datetime.today().strftime('%d')
datum = year + month + day
try:
  trialNum
except NameError:
  print 'assigned trialNum to 1'
  trialNum = 1

bagDir = '/home/rancher/bagfiles/'
while reCheck:
    outFileName= datum + driver + responder + str(trialNum).zfill(3) + ' ' + scriptNmXpPrt + rig + '.bag'
    # check if the destination file exists
    if os.path.isfile(bagDir+outFileName.replace(" ", "_")):
        trialNum += 1
        reCheck = True
    else:
        reCheck = False
outFileName= datum + driver + responder + str(trialNum).zfill(3)+ ' ' + scriptNmXpPrt + rig + '.bag'

## roslaunch prep (from http://wiki.ros.org/roslaunch/API%20Usage) ##vid
if saveBag:
    import roslaunch, rospy ##vid
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) ##vid
    roslaunch.configure_logging(uuid) ##vid
    outFileNameNoSpc = outFileName.replace(" ", "_") ##vid
    pFix = outFileNameNoSpc[:-4] ##vid
    if pFix: ##vid
        print 'yes' ##vid
    else: ##vid
        pFix = datetime.datetime.today().strftime('%Y-%m-%d-%H-%M-%S') ##vid
    includepfix = 'prefix:=' + pFix ##vid
    sys.argv.append(includepfix) ##vid
    parent = roslaunch.parent.ROSLaunchParent(uuid, [recLaunchDir]) ##vid
    rospy.on_shutdown(parent.shutdown) ##vid

SFOscV, NoPattV, CLVoltage = 8., 9.99, 9.
pattern_id, CLpat, allOffPat, SFOscPat, SFOne, SFTwo, SFThr, SFFr = 1, 10, 0, 1, 3, 4, 5, 6 # 11 = stripe
slpT  = .05
gainCL, gainOL_, gainLoom = -.5, 2., 6.
gainOL = gainOL_

AOthr, stim_dur, SFOsc_dur  = 2, 300., 8.
#AOthr, stim_dur, SFOsc_dur  = 2, 8., 4.

################## Start Bagfile ########################
if saveBag:
    parent.start() ##vid ipython command equivalent to "roslaunch magnotether recordMT.launch"
    print 'Saving bag file: ' + pFix + '.bag' ##vid
    aout = PhidgetsAoutProxy()

################## Panels stuff ########################
com_port_num = '/dev/ttyUSB0'
ctlr = PanelCom(com_port_num)
time.sleep(slpT)
print ' '
print 'Started trial ' + str(trialNum) + ' at: ' + str(datetime.datetime.now())[11:19]
t = time.time()
expStartTime = t
modex = 0 # open loop
ctlr.SetMode(modex, 0)
time.sleep(slpT) 
ctlr.SetGainOffset(gainOL, 0, 0, 0)
time.sleep(slpT)
pattern_id = 1 # 1-based
ctlr.SetPatternID(pattern_id)
time.sleep(slpT)

# static starfield
print 'static star field for 5 min.'
aout.set_voltage(AOthr,baseLevelPat/2) # For book keeping of the LED pattern presented
ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
time.sleep(stim_dur)

## Stop Bagfile
if saveBag:
    parent.shutdown() ##vid # ipython command equivalent to Ctrl-C for "recordMT.launch"

elapsed = time.time() - t
print 'Trial time [s]:' + str(elapsed)[0:5]
print outFileName[0:-4]
