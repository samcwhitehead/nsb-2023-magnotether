#!/usr/bin/env python
from panel_com import PanelCom
import os, time, sys, datetime
import numpy as np
from phidgets_aout_proxy import PhidgetsAoutProxy
trialNum = 1
rig = 'MT'
#rig = 'RT'
if rig == 'MT':
    AOexc        = 1
    recLaunchDir = "/home/rancher/catkin/src/magnotether/launch/recordMT.launch"
    CsChr_ONV    = 1. # range: [0.1 - 1.0]
    CsChr_OFFV   = 0.
    baseLevelPat = 1 #4
    SFOscDur     = 8.  # duration of the star field sinusoidal oscilation
    gainOL       = 2.4 # a panel refresh rate of 48Hz, so that the .25 Hz oscillation shows up correctly
    print 'Magno Tether'
elif rig == 'RT': 
    AOexc        = 0
    recLaunchDir = "/home/rancher/catkin/src/Kinefly/launch/recordRT.launch"
    CsChr_ONV    = .8, # range: [0.1 - 3.3] for .3mm D lightspot driven by Thorlabs LEDD1B M00407072
    #Vs = np.array([0.5, 1.1, 2.42]) # control voltages that lead to 530 nm light power at the sample of [10, 20, 40] uW/mm^2.
    CsChr_OFFV   = 0.
    baseLevelPat = 1
    SFOscDur     = 8.
    gainOL       = 4.0 # 4.0 = a panel refresh rate of 40Hz, so that the .25 Hz oscillation shows up correctly
    print 'Rigid Tether'
else:
    print 'unable to determine which rig are you using with Xt_Cs_b.py ...' 

scriptNmXpPrt    = os.path.basename(__file__)[3:5]
reCheck          = True
driver           = ' SVES'
responder        = ' U095 '

## saveBag
saveBag          = True

## Set variable names and parameters
year             = datetime.datetime.today().strftime('%Y')[2:4]
month            = datetime.datetime.today().strftime('%m')
day              = datetime.datetime.today().strftime('%d')
datum            = year + month + day

try:
  trialNum
except NameError:
  print 'assigned trialNum to 1'
  trialNum       = 1

bagDir           = '/home/rancher/bagfiles/'
while reCheck:
    outFileName  = datum + driver + responder + str(trialNum).zfill(3) + ' ' + scriptNmXpPrt + rig + '.bag'
    # check if the destination file exists
    if os.path.isfile(bagDir+outFileName.replace(" ", "_")):
        trialNum += 1
        reCheck  = True
    else:
        reCheck  = False

outFileName      = datum + driver + responder + str(trialNum).zfill(3) + ' ' + scriptNmXpPrt + rig + '.bag'

## roslaunch prep (from http://wiki.ros.org/roslaunch/API%20Usage) ##vid
if saveBag:
    import roslaunch, rospy ##vid
    uuid         = roslaunch.rlutil.get_or_generate_uuid(None, False) ##vid
    roslaunch.configure_logging(uuid) ##vid
    outFileNameNoSpc = outFileName.replace(" ", "_") ##vid
    pFix         = outFileNameNoSpc[:-4] ##vid
    if pFix: ##vid
        print 'yes' ##vid
    else: ##vid
        pFix    = datetime.datetime.today().strftime('%Y-%m-%d-%H-%M-%S') ##vid
    includepfix = 'prefix:=' + pFix ##vid
    sys.argv.append(includepfix) ##vid
    parent      = roslaunch.parent.ROSLaunchParent(uuid, [recLaunchDir]) ##vid
    rospy.on_shutdown(parent.shutdown) ##vid

SFOscV, NoPattV, CLV = 8., 9.99, 9.
pattern_id, CLpat, allOffPat, SFOscPat, SFOne, SFTwo, SFThr, SFFr = 1, 10, 0, 1, 3, 4, 5, 6 # 11 = stripe
slpT            = .05
gainCL, gainLoom= -.5, 6. 

pattIDV         = NoPattV

################## Analog outputs ########################
#Vs = np.array([.66, .66, .66]) # control voltages that lead to light power at the sample of [10, 20, 40] uW/mm^2.
#LP = np.array([20, 20, 20])
AOthr           = 2

shortRun        = True
if shortRun:
    stimOn_dur, numStims = 15., 3  # 15 s. ON stimulus duration for VES41
    stimOff_dur = 30.-stimOn_dur
else:
    stimOn_dur, numStims = 30., 3 
    stimOff_dur = 60.-stimOn_dur

# to test
if 1:
    stimOn_dur, numStims = 2., 3  # 15 s. ON stimulus duration for VES41
    stimOff_dur = 4.-stimOn_dur

################## Start Bagfile ########################
if saveBag:
    parent.start() ##vid ipython command equivalent to "roslaunch magnotether recordMT.launch"
    print 'Saving bag file: ' + pFix + '.bag' ##vid
    aout        = PhidgetsAoutProxy()

################## Panels related ########################
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
#ctlr.SetPositions(0,allOffPat)
time.sleep(.5)

# initial static starfield
#print 'static star field'
#aout.set_voltage(AOthr,baseLevelPat/2) # For book keeping of the LED pattern presented
#ctlr.SetPositions(0,SFOscPat) # # baseLevelPat #baseLevel, static SF
#time.sleep(slpT)

## Initial open loop SF oscillation (.5 Hz)
aout.set_voltage(AOthr, SFOscV)
ctlr.SetPositions(0,SFOscPat)
time.sleep(slpT)
print 'OL SF .25 Hz oscillations, 8 s'
ctlr.Start()
time.sleep(SFOscDur)
ctlr.Stop()
time.sleep(slpT)
aout.set_voltage(AOthr, baseLevelPat/2) # For book keeping of the LED pattern presented
#ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
#time.sleep(slpT)

# create loop here for number of CsChrimson activations
k = 0
while (k <  numStims ):
    k += 1
    #CsChr_ONV +=.2
    aout.set_voltage(AOexc, CsChr_OFFV)
    print '617 nm excitation OFF'
    time.sleep(stimOff_dur)
    aout.set_voltage(AOexc, CsChr_ONV)
    print '617 nm excitation ON : Stim %i out of %i' % (k, numStims)
    #print '617 nm irradiance: ?? uW/mm^2. Stim %i out of %i' % (k, numStims)
    time.sleep(stimOn_dur)

# final excitation off period
aout.set_voltage(AOexc, CsChr_OFFV)
print '617 nm excitation OFF'
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
#time.sleep(slpT)
#aout.set_voltage(AOthr, baseLevelPat/2) # For book keeping of the LED pattern presented
#aout.set_voltage(AOexc, CsChr_OFFV)
#ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
#time.sleep(slpT)

print 'Static, .5 s'
aout.set_voltage(AOthr, NoPattV)
#ctlr.SetPositions(0,allOffPat)
time.sleep(.5)

#print 'back to baseline level'
#aout.set_voltage(AOthr,baseLevelPat/2) # For book keeping of the LED pattern presented
#ctlr.SetPositions(0,baseLevelPat) # baseLevel, static SF
#time.sleep(slpT)
#print ' '

## Stop Bagfile
if saveBag:
    parent.shutdown() ##vid # ipython command equivalent to Ctrl-C for "recordMT.launch"

elapsed = time.time() - t
print 'Trial time [s]:' + str(elapsed)[0:5]
print outFileName[0:-4]
