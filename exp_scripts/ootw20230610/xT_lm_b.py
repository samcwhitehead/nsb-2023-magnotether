#!/usr/bin/env python
from panel_com import PanelCom
import os, time, sys, datetime
import numpy as np
from phidgets_aout_proxy import PhidgetsAoutProxy

## mode functions

def OpenLpLm(gainOL, signDir, pattIDV):
    gainOLsigned = signDir * gainOL
    ctlr.SetGainOffset(gainOLsigned, 0, 0, 0)
    time.sleep(slpT)
    print fromLoom + 'Stim %i out of %i' % (k, numStims)
    aout.set_voltage(AOthr, pattIDV)
    # run stimulus
    ctlr.Start()
    time.sleep(OLstim_dur)
    ctlr.Stop()
    
trialNum = 1
#rig = 'MT'
rig = 'RT'
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
#driver = ' S000'
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

## script specific from here on
NoPattV, CLVoltage, StatImVoltage, LoomVoltage = 9.99, 9., 0., 8. # OL voltage is pattern-dependent
pattern_id, CLpat, allOffPat = 1, 10, 0 # 11 = stripe
#CLstim_dur, Static_dur, OLstim_dur, slpT  = 1.3, 4.75, 1.53, .05
CLstim_dur, Static_dur, OLstim_dur, slpT  = 1.3, 4.75, 1.53, .1 
gainCL, gainOL_, gainLoom = -.5, 4., 6. 
gainOL = gainLoom
AOthr  = 8.

## Generate array to determine Pat and Dirctn
patss = [[(8.,.75)],[(7.,1.25)],[(8.,.75)],[(7.,1.25)],[(8.,.75)],[(7.,1.25)],[(8.,.75)],[(7.,1.25)],[(8.,.75)],[(7.,1.25)],[(8.,.75)],[(7.,1.25)],[(8.,.75)],[(7.,1.25)],[(8.,.75)],[(7.,1.25)],[(9.,1.)],[(9.,1.)]] # 2 m/s approach speed of .2 m D. object
#patss = [[(8.,.75)],[(7.,1.25)],[(8.,.75)]] # 2 m/s approach speed of .2 m D. object
pats = np.vstack(patss)
## Randomize
np.random.shuffle(pats)
numStims = len(pats)

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
print 'Started at: ' + str(datetime.datetime.now())[11:19]
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
ctlr.SetPositions(0,allOffPat)
aout.set_voltage(AOthr, NoPattV)
time.sleep(.5) 

k = 0
while (k <  numStims ):
    k += 1
    (subpats, dirctn) = pats[k-1] # 0-based python: k-1
    signDir = np.sign(dirctn)
    if subpats == 8.:
	fromLoom = 'Left  loom. '
    elif subpats == 7.:
        fromLoom = 'Right loom. '
    elif subpats == 9.:
        fromLoom = 'No    loom. '
    else:
        fromLoom = 'Undetermined. '
    subpat = int(subpats)
    ctlr.SetPositions(0,subpat)
    aout.set_voltage(AOthr, StatImVoltage)
    time.sleep(Static_dur)
    pattIDV = LoomVoltage * signDir + signDir*(abs(dirctn)-1.)
    OpenLpLm(gainOL, signDir, pattIDV)
    
# final static image
ctlr.SetPositions(0,subpat)
aout.set_voltage(AOthr, StatImVoltage)
time.sleep(Static_dur)
ctlr.AllOff()
aout.set_voltage(AOthr, NoPattV)
time.sleep(.5)

## Stop Bagfile
if saveBag:
    parent.shutdown() ##vid # ipython command equivalent to Ctrl-C for "record.launch"

elapsed = time.time() - t
print 'Trial time [s]:' + str(elapsed)[0:5]
print outFileName[0:-4]
