#!/usr/bin/env python
from panel_com import PanelCom
import os, time, sys, datetime
import numpy as np
from phidgets_aout_proxy import PhidgetsAoutProxy
trialNum = 1
#rig = 'MT'
rig = 'RT'
if rig == 'MT':
    AOexc = 1
    recLaunchDir = "/home/rancher/catkin/src/magnotether/launch/recordMT.launch"
    CsChr_ONV, CsChr_OFFV = 0.82, 0.
    baseLevelPat = 4
    print 'Magno Tether'
elif rig == 'RT':
    AOexc = 0
    recLaunchDir = "/home/rancher/catkin/src/Kinefly/launch/recordRT.launch"
    CsChr_ONV, CsChr_OFFV = 2.42, 0. # for .3mm D lightspot driven by Thorlabs LEDD1B M00407072
    #Vs = np.array([0.5, 1.1, 2.42]) # control voltages that lead to 530 nm light power at the sample of [10, 20, 40] uW/mm^2.
    #CsChr_ONV = 1.1
    #CsChr_ONV = 3.5
    #CsChr_ONV = 5.
    #CsChr_ONV = 7.
    baseLevelPat = 0
    print 'Rigid Tether'
else:
    print 'what rig are you using?'

#AOthr, stimOn_dur, numStims, = 2, .25, 6
#stimOff_dur = 10.-stimOn_dur
#AOthr, stimOn_dur, numStims, sacStat_dur = 2, .25, 6, 10.5
#AOthr, stimOn_dur, numStims, sacStat_dur = 2, 1., 6, 10.5
AOthr, stimOn_dur, numStims, sacStat_dur = 2, .5, 6, 10.5
#AOthr, stimOn_dur, numStims, sacStat_dur = 2, 2., 6, 10.5
stimOff_dur = 10.-stimOn_dur

# to test:
testRun = False
if testRun:
    CsChr_ONV = .2
    AOthr, stimOn_dur, numStims, sacStat_dur = 2, .5, 3, 3.
    stimOff_dur = 2.-stimOn_dur

scriptNmXpPrt = os.path.basename(__file__)[3:5]
reCheck = True
#driver = ' S194'
driver = ' S390'
#driver = ' S015'
#driver = ' S217'
#driver = ' S000'
responder = ' SPRC '

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

bagDir = '/home/rancher/bagfiles/'
while reCheck:
    outFileName= datum + driver + responder + str(trialNum).zfill(3) + ' ' + scriptNmXpPrt + rig + '.bag'
    # check if the destination file exists
    if os.path.isfile(bagDir+outFileName.replace(" ", "_")):
        trialNum += 1
        reCheck = True
    else:
        reCheck = False
outFileName= datum + driver + responder + str(trialNum).zfill(3) + ' ' + scriptNmXpPrt + rig + '.bag'

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

SFOscV, NoPattV, CLV = 8., 9.99, 9.
pattern_id, CLpat, allOffPat, SFOscPat, SFOne, SFTwo, SFThr, SFFr = 1, 10, 0, 1, 3, 4, 5, 6 # 11 = stripe
slpT  = .05
gainCL, gainOL_, gainLoom = -.5, 2., 6.
gainOL = gainOL_
pattIDV = NoPattV

################## Analog outputs ########################
#Vs = np.array([.66, .66, .66]) # control voltages that lead to light power at the sample of [10, 20, 40] uW/mm^2.
#LP = np.array([20, 20, 20])


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

print 'Darkness, .5 s'
aout.set_voltage(AOthr, NoPattV)
ctlr.SetPositions(0,allOffPat)
time.sleep(.5)

# initial static starfield
print 'static star field'
aout.set_voltage(AOthr,baseLevelPat/2) # For book keeping of the LED pattern presented
ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
time.sleep(slpT)

## Initial open loop SF oscillation (.5 Hz)
#aout.set_voltage(AOthr, SFOscV)
#ctlr.SetPositions(0,SFOscPat)
#time.sleep(slpT)
#print 'OL SF .25 Hz oscillations, 8 s'
#ctlr.Start()
#time.sleep(SFOscDur)
#ctlr.Stop()
#time.sleep(slpT)
#aout.set_voltage(AOthr, baseLevelPat/2) # For book keeping of the LED pattern presented
#ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
#time.sleep(slpT)

# initial excitation off period
aout.set_voltage(AOexc, CsChr_OFFV)
print 'pre-excitation period'
time.sleep(sacStat_dur - stimOff_dur)
# create loop here for number of CsChrimson activations
k = 0
while (k <  numStims ):
    k += 1
    aout.set_voltage(AOexc, CsChr_OFFV)
    print '617 nm excitation OFF'
    time.sleep(stimOff_dur)
    aout.set_voltage(AOexc, CsChr_ONV)
    print '617 nm excitation ON : Stim %i out of %i' % (k, numStims)
    #print '617 nm irradiance: ?? uW/mm^2. Stim %i out of %i' % (k, numStims)
    time.sleep(stimOn_dur)
# final excitation off period
aout.set_voltage(AOexc, CsChr_OFFV)
print '617 nm excitation OFF and post period'
time.sleep(stimOff_dur)
print ' '

## Open loop SF oscillation (.5 Hz)
#print 'OL SF .25 Hz oscillations, 8 s'
#aout.set_voltage(AOthr, SFOscV)
#aout.set_voltage(AOexc, CsChr_ONV)
#ctlr.SetPositions(0,SFOscPat)
#time.sleep(slpT)
#ctlr.Start()
#time.sleep(SFOscDur)
#ctlr.Stop()
t#ime.sleep(slpT)
#aout.set_voltage(AOthr, baseLevelPat/2) # For book keeping of the LED pattern presented
#aout.set_voltage(AOexc, CsChr_OFFV)
#ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
#time.sleep(slpT)

print 'Darkness, .5 s'
aout.set_voltage(AOthr, NoPattV)
ctlr.SetPositions(0,allOffPat)
time.sleep(.5)

print 'back to baseline level'
aout.set_voltage(AOthr,baseLevelPat/2) # For book keeping of the LED pattern presented
ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
time.sleep(slpT)
print ' '

## Stop Bagfile
if saveBag:
    parent.shutdown() ##vid # ipython command equivalent to Ctrl-C for "recordMT.launch"

elapsed = time.time() - t
print 'Trial time [s]:' + str(elapsed)[0:5]
print outFileName[0:-4]
