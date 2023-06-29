#!/usr/bin/env python
from panel_com import PanelCom
import os, time, sys, datetime
import numpy as np
from phidgets_aout_proxy import PhidgetsAoutProxy

rig = 'MT'
#rig = 'RT'
if rig == 'MT':
    AOexc = 1
    recLaunchDir = "/home/rancher/catkin/src/magnotether/launch/recordMT.launch"
    baseLevelPat = 0
    print 'Magno Tether'
elif rig == 'RT':
    AOexc = 0
    recLaunchDir = "/home/rancher/catkin/src/Kinefly/launch/recordRT.launch"
    baseLevelPat = 0
    print 'Rigid Tether'
else:
    print 'what rig are you using?' 

# to Test
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

scriptNmXpPrt = os.path.basename(__file__)[3:5]
reCheck = True
#driver = ' S000'
#driver = ' OK371'
driver = ' XHCS'
#driver = ' S000'
responder = ' wldT '

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
    pFix = 'test'
    includepfix = 'prefix:=' + pFix ##vid
    sys.argv.append(includepfix) ##vid
    parent = roslaunch.parent.ROSLaunchParent(uuid, [recLaunchDir]) ##vid
    rospy.on_shutdown(parent.shutdown) ##vid

SFOscV, NoPattV, CLVoltage = 8., 9.99, 9.
GtACR_ONV, GtACR_OFFV = .66, 0.
pattern_id, CLpat, allOffPat, SFOscPat, SFOne, SFTwo, SFThr, SFFr = 1, 10, 0, 1, 3, 4, 5, 6 # 11 = stripe
slpT  = .05
gainCL, gainOL_, gainLoom = -.5, 2., 6.
gainOL = gainOL_
pattIDV = NoPattV

################## Analog outputs ########################
#Vs = np.array([0.5, 1.1, 2.42]) # control voltages that lead to light power at the sample of [10, 20, 40] uW/mm^2.
#LP = np.array([10, 20, 40])
#Vs = np.array([1.1, 1.1, 1.1]) # control voltages that lead to light power at the sample of [10, 20, 40] uW/mm^2.
#LP = np.array([20, 20, 20])
## Generate array to determine Pat and Dirctn
# modified: R, P, Yaw 2X (instead of OL, OR), Rg2X, Pg2x
patNames=['LvlHig', 'LvlHig', 'LvlHig']
patss=[[(7.,1.)],[(7.,1.)],[(7.,1.)]] 
patssStack = np.vstack(patss)
numStims = patssStack.shape[0]
origIdxCol = np.zeros((numStims,1))
for i in range(numStims):
    origIdxCol[i,0] = i
pats = np.hstack((patssStack,origIdxCol))
## Randomize
#np.random.shuffle(pats)

AOtwo, AOthr, stim_dur, SFOscDur  = 1, 2., .5, 1.

################## Start Bagfile ########################
if saveBag:
    parent.start() ##vid ipython command equivalent to "roslaunch magnotether recordMT.launch"
    print 'Saving bag file: ' + pFix + '.bag' ##vid
    aout = PhidgetsAoutProxy()

################## Panels stuff ########################
#### stripe fixation for 1 min ####
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

# initial half second nothing
aout.set_voltage(AOthr, NoPattV)
ctlr.SetPositions(0,allOffPat)
time.sleep(0.5)

## Initial open loop SF oscillation (.5 Hz)
ctlr.SetPositions(0,SFOscPat)
time.sleep(slpT)
aout.set_voltage(AOthr, SFOscV)
ctlr.Start()
time.sleep(SFOscDur)
ctlr.Stop()
time.sleep(slpT)

# initial static starfield
print 'back to baseline level'
aout.set_voltage(AOthr,baseLevelPat/2) # For book keeping of the LED pattern presented
ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
time.sleep(slpT)

# get levels:
#from baselevel go up in brightness
# create loop here for number of GtACR silencing
k = 0
while (k <  numStims ):
    #print '%i s. darkness' % stim_dur
    k += 1
    (subpats, dirctn, patNameIdx) = pats[k-1] # 0-based python: k-1
    subpat = int(subpats)
    aout.set_voltage(AOthr, baseLevelPat/2)
    print 'base level'
    ctlr.SetPositions(0,baseLevelPat) # base Level
    time.sleep(stim_dur)
    aout.set_voltage(AOthr, 0.)
    print patNames[int(patNameIdx)] + '. Stim %i out of %i' % (k, numStims)
    LvlPatVoltage = subpats/2.
    aout.set_voltage(AOthr, LvlPatVoltage)
    ctlr.SetPositions(0,subpat) # test Level
    time.sleep(stim_dur)

print 'final base level'
LvlPatVoltage = 6.
aout.set_voltage(AOthr, baseLevelPat/2)
ctlr.SetPositions(0,baseLevelPat) # baseLevel
aout.set_voltage(AOtwo, AOexc)
time.sleep(stim_dur)
aout.set_voltage(AOtwo, GtACR_OFFV)

## Stop Bagfile
if saveBag:
    parent.shutdown() ##vid # ipython command equivalent to Ctrl-C for "recordMT.launch"

elapsed = time.time() - t
print 'Trial time [s]:' + str(elapsed)[0:5]
