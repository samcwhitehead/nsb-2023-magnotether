"""
Created on Fri Sep 13 10:04:18 2021

@author: ivo

Finds transient events, such as saccades, based on a time-varying position trace, such as a fly body angle.
Processes local min-/maxima that are within threshold/ceiling bounds of the trace velocity vector.
Subsequently, retains only those events that supersede a magnitude threshold and determine their magnitude.

Example usage
import Sac_ID_MT_basic as sid
help(sid) # for details
SAmx, SAmn, SVmx, SVmn              = sid.findSacs(angleFilt) 

Plotting example, as dots above or below a trace in a mpl plot called 'ax
yPos                                = 50.
sIdcs                               = np.argwhere(np.abs(SAmx)>0)
for sIdx in sIdcs:
    ax.plot(timeCol[sIdx], yPos, ls='None',  marker='.', color='r',  mec='None', zorder=10, markersize=2.)
sIdcs                               = np.argwhere(np.abs(SAmn)>0)
for sIdx in sIdcs:
    ax.plot(timeCol[sIdx], -yPos,ls='None',  marker='.', color='b',  mec='None',  zorder=10, markersize=2.)
    
INPUTS:
angleFilt:          A filtered magnotether fly body angle vector (e.g. Savitsky-Golay, Kalman, or butterworth filter) [deg]
Optional inputs:
fs:                 The sampling frequency
angVelo:            Time-derivative of angleFilt (can be directly obtained from the Kalman filter)
fVec:               A np.zeros vector with NaN entries where the fly was not flying
stableOnset_s:      Imposed stable onset duration [s]
wsvThresh, wsvCeil: Thresholds and ceilings of body rotation velocity 
magThresh:          Saccade magnitudes threshold

OUPUTS:
SAmx, SAmn, SVmx, SVmn: np.nan vectors with Saccade Angles and Saccade Velocities in positive and negative directions, respectively

For question, feel free to email ivo.ros@gmail.com

"""

import numpy as np
from scipy.signal import argrelextrema

#%%
def calcMagDur(idx, angV, ang, fs): # calculate difference in L-R WSA between nearest local extremes (of opposite sign)
    lastIdx                             = angV.shape[0]-1
    # find the startIndex of the saccade
    k                                   = 0
    onIdx                               = None
    chkMore                             = True
    refSign                             = np.sign(angV[idx])
    while chkMore:
        k                              += 1
        compSign                        = np.sign(angV[idx-k])
        if refSign                     != compSign: # if not equal
            onIdx                       = idx-k+1
            chkMore                     = False
    k                                   = 0
    offIdx                              = None
    chkMore                             = True
    while chkMore and idx+k+1 < lastIdx:
        k                              += 1
        compSign                        = np.sign(angV[idx+k])
        if refSign                     != compSign:
            offIdx                      = idx+k
            chkMore                     = False
    if offIdx == None:
        offIdx                          = lastIdx
    # calculate FWDM, the full width [s] at 1st decimal of the max (10 pct of peak) as a robust estimate of Saccade duration
    if len(angV[onIdx:offIdx]) < 1:
        print (angV[onIdx:offIdx])
        durtn                           = np.nan
    else:
        thresh                          = np.sign(angV[idx])*60.
        FWDM_sttIdx                     = onIdx  + np.argmin(np.abs(angV[onIdx:idx+1]  - thresh))
        FWDM_endIdx                     = idx    + np.argmin(np.abs(angV[idx  :offIdx] - thresh))
        durtn                           = (FWDM_endIdx-FWDM_sttIdx)/fs
    return ang[offIdx] - ang[onIdx], durtn, onIdx, offIdx # integrate: BR (behavioral response) = ang(offIdx) - ang(onIdx)

#%%
def sacOnly(inVec, fVec):
    fVecSqz                             = np.squeeze(fVec)
    outVec                              = np.array( np.sum([inVec, fVecSqz], axis=0) ).transpose()
    outVec[outVec == 0]                 = np.nan
    return outVec

#%%
def findSacs(angleFilt, angVelo=None, fVec=None, fs=None, angVeloThresh=None, angVeloCeil=None, magThresh=None, stableOnset_s=None):

    ## assign default values if non included when the function is called
    if fs is None:
        fs                              = 100.
        
    if angVelo is None:
        angVelo                         = np.zeros_like(angleFilt)
        angVelo[:-1]                    = np.diff(angleFilt)/(1./fs)
        
    if fVec is None:
        fVec                            = np.zeros_like(angleFilt)

    if angVeloThresh is None:
        angVeloThresh                   = 120.
        
    if angVeloCeil is None:
        angVeloCeil                     = 1200.
        
    if magThresh is None:
        magThresh                       = 12.5

    if stableOnset_s is None:
        stableOnset_s                   = 0.25
    
    print('Saccade identifier retains body rotation velocity transients between {} and {} dps, a magnitude > {} deg, and with at least a {} s stable onset.'.format(angVeloThresh, angVeloCeil, magThresh, stableOnset_s))
        
    ## Place holders
    angVeloMxThr,   angVeloMnThr        = np.zeros_like(angVelo), np.zeros_like(angVelo) 
    SangVeloMx,     SangVeloMn          = np.zeros_like(angVelo), np.zeros_like(angVelo)
    lcMxMagTemp,    lcMnMagTemp         = np.zeros_like(angVelo), np.zeros_like(angVelo)
    lcMxDurTemp,    lcMnDurTemp         = np.zeros_like(angVelo), np.zeros_like(angVelo)
    lcMxDur,        lcMnDur             = np.zeros_like(angVelo), np.zeros_like(angVelo) 
    sacMx,          sacMn               = np.zeros_like(angVelo), np.zeros_like(angVelo)
    lcMnonIdx,      lcMnoffIdx          = np.zeros_like(angVelo), np.zeros_like(angVelo)
    lcMxonIdx,      lcMxoffIdx          = np.zeros_like(angVelo), np.zeros_like(angVelo)
    SacAngMx,       SacAngMn            = np.zeros_like(angVelo), np.zeros_like(angVelo)
    SacSttIdMx,     SacSttIdMn          = np.zeros_like(angVelo), np.zeros_like(angVelo)
    
    ## Find local extremes in the rate, that exceed a velocity threshold
    reboundIds                          = int(np.floor(stableOnset_s*fs)) # .25 s refractory period
    lclMxIndices                        = np.squeeze(argrelextrema(angVelo, np.greater))
    lclMnIndices                        = np.squeeze(argrelextrema(angVelo, np.less))
    reboundEdgFr                        = int(1.4*reboundIds)
    reboundEdgTo                        = len(angVelo)-int(1.4*reboundIds)
    lclMxIdcs                           = lclMxIndices[np.logical_and(lclMxIndices>reboundEdgFr,lclMxIndices<reboundEdgTo)]
    lclMnIdcs                           = lclMnIndices[np.logical_and(lclMnIndices>reboundEdgFr,lclMnIndices<reboundEdgTo)]
    
    for idx in lclMxIdcs:
        if angVelo[idx]>angVeloThresh and angVelo[idx]<angVeloCeil: 
            angVeloMxThr[idx]           = angVelo[idx]
    for idx in lclMnIdcs:
        if angVelo[idx]<-angVeloThresh and angVelo[idx]>-angVeloCeil:
            angVeloMnThr[idx]           = angVelo[idx]   
        
    ## Retain extremes that are less than magnitude limit (velocity dependent) [no longer: also exceed the saccade magnitude threshold)
    # Integrate unidirectional wsv sections surrounding local extremes to obtain saccade magnitude
    for idx in range (int(1.4*reboundIds),len(angVelo)-int(1.4*reboundIds)):
        if abs(angVeloMxThr[idx])>0: # for every local max
            lcSteerMag, lcSteerDur, onIdx, offIdx = calcMagDur(idx, angVelo, angleFilt, fs)
            lcMxMagTemp[idx]            = lcSteerMag
            lcMxDurTemp[idx]            = lcSteerDur
            lcMxonIdx[idx]              = onIdx
            lcMxoffIdx[idx]             = offIdx
        if abs(angVeloMnThr[idx])>0: # for every local minimum
            lcSteerMag, lcSteerDur, onIdx, offIdx = calcMagDur(idx, angVelo, angleFilt, fs)
            lcMnMagTemp[idx]            = lcSteerMag
            lcMnDurTemp[idx]            = lcSteerDur
            lcMnonIdx[idx]              = onIdx
            lcMnoffIdx[idx]             = offIdx
        
    ## Special MT issue of 'hutchen peak', or 'double hump': If a double hump is encountered, only retain higher of the two
    ## remove current extrema, if preceded within the rebound period by a higher syndirectional extremus
    removeHutchens                      = 1
    if removeHutchens:
        for idx in range (reboundIds,len(angVelo)-reboundIds):
            if lcMxMagTemp[idx]>0: # for every local maximum that exceeds the magnitude threshold
                if abs(np.sum(lcMxMagTemp[idx-reboundIds:idx]))>magThresh and  ~np.isnan(fVec[idx-reboundIds:idx+reboundIds]).any(axis=0):
                    onsetExtrema        = lcMxMagTemp[idx-reboundIds:idx][abs(lcMxMagTemp[idx-reboundIds:idx]) > 0.]
                    onsetMax            = np.max(abs(np.squeeze(onsetExtrema)))
                    if abs(lcMxMagTemp[idx]) < onsetMax:
                        lcMxMagTemp[idx] = 0.
                    else:
                        lcMxMagTemp[idx-reboundIds:idx] = 0.
            elif lcMnMagTemp[idx]<0: # for every local minimum that exceeds the magnitude threshold
                if abs(np.sum(lcMnMagTemp[idx-reboundIds:idx]))>magThresh and  ~np.isnan(fVec[idx-reboundIds:idx+reboundIds]).any(axis=0):
                    onsetExtrema        = lcMnMagTemp[idx-reboundIds:idx][abs(lcMnMagTemp[idx-reboundIds:idx]) > 0.]
                    onsetMax            = np.max(abs(np.squeeze(onsetExtrema)))
                    if abs(lcMnMagTemp[idx]) < onsetMax:
                        lcMnMagTemp[idx] = 0.
                    else:
                        lcMnMagTemp[idx-reboundIds:idx] = 0.
     
    ## Obtain saccades, retained extremes that are not 'rebounds' (stable pre-onset activity for .5s).
    requireStableOnset                  = 1
    for idx in range (reboundIds,len(angVelo)-reboundIds):
        if lcMxMagTemp[idx]>0: # for every local maximum that exceeds the magnitude threshold
            if requireStableOnset:
                inclThisOne             = 0
                if abs(np.sum(lcMnMagTemp[idx-reboundIds:idx]))<magThresh and ~np.isnan(fVec[idx-reboundIds:idx+reboundIds]).any(axis=0):
                    inclThisOne         = 1
            else:
                inclThisOne             = 1
            if inclThisOne:    
                SangVeloMx[idx]         = angVelo[idx]
                lcMxDur[idx]            = lcMxDurTemp[idx]
                sacMx[int(lcMxonIdx[idx]):int(lcMxoffIdx[idx])] = angleFilt[int(lcMxonIdx[idx]):int(lcMxoffIdx[idx])]
                SacSttIdMx[idx]         = int(lcMxonIdx[idx])
                sacEndIdx               = int(lcMxoffIdx[idx])
                SacAngMx[sacEndIdx]     = angleFilt[sacEndIdx]                      # such that saccades can be indicated at peaks 
        elif lcMnMagTemp[idx]<0: # for every local minimum that exceeds the magnitude threshold
            if requireStableOnset:
                inclThisOne             = 0
                if abs(np.sum(lcMxMagTemp[idx-reboundIds:idx]))<magThresh and ~np.isnan(fVec[idx-reboundIds:idx+reboundIds]).any(axis=0):
                    inclThisOne         = 1
            else:
                inclThisOne             = 1
            if inclThisOne:  
                SangVeloMn[idx]         = angVelo[idx]
                lcMnDur[idx]            = lcMnDurTemp[idx]
                sacMn[int(lcMnonIdx[idx]):int(lcMnoffIdx[idx])] = angleFilt[int(lcMnonIdx[idx]):int(lcMnoffIdx[idx])]
                SacSttIdMn[idx]         = int(lcMnonIdx[idx])
                sacEndIdx               = int(lcMnoffIdx[idx])
                SacAngMn[sacEndIdx]     = angleFilt[sacEndIdx]
             
    ## only when fly is flying, set all other instances to nan    
    SAmx,    SAmn                       = sacOnly(SacAngMx,     fVec), sacOnly(SacAngMn,     fVec)
    SVmx,    SVmn                       = sacOnly(SangVeloMx,   fVec), sacOnly(SangVeloMn,   fVec)
        
    return SAmx, SAmn, SVmx, SVmn