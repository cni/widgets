
# coding: utf-8

# In[4]:

#       This code detects the QRS complex from a pre-recorded ECG signal read from a .txt file. 
#       The method is based off of that described by Jinkwon Kim and Hangsik Shin in their article
# 'Simple and Robust Realtime QRS Detection Algorithm Based on Spatiotemporal Characteristic of the QRS Complex'. 
#       The source article can be found here: 
# http://journals.plos.org/plosone/article?id=10.1371/journal.pone.0150144
#       This code has been adapted from the original algorithm source code, found here:
# https://github.com/HangsikShin/QRS-Detection/blob/master/Simple-and-Robust-Realtime-QRS-Detection-Algorithm-based-on-Spatiotemporal-Characteristic-of-the-QRS-Complex/QRSdetection.m

#       This algorithm takes in data from a signle ECG lead and detects QRS complexes based on physical characteristics of the complexes. 
# It filters the signal through a bandpass filter, uses moving average filters to get characteristics such as duration and rate of change,
# compares them to pre-set thresholds, and adjusts weights of thresholds before iterating again. 
# It is designed to use relatively simple calculations to provide a robust, noise-resistant method of QRS detection. 

#       This code is currently set up to test the algorithm on 30,000 samples from ECG sample #100 from the MIT-BIH database. 
# There is a main function at the bottom that reads in a text file of data, runs the algorithm on it, and plots both the raw data 
# and the detected QRS points. The text file, sampling frequency, length of raw data, and details of the plot are all adjustable in-code. 
# The program will also display the total amount of time spent processing the signal and the average time spent on each point. 

#       The goal is for this code/algorithm to be modified to detect QRS locations in real-time. 

# Joe Sommer, 2017 


import numpy as np
from scipy import signal, io 
import matplotlib.pyplot as plt

import time


#       The function QRSdetection is adapted from the Matlab file QRSdetection.m and does the actual detection of the QRS complex.
#       Inputs: wdata (ECG waveform data), Fs (sampling frequency)
#       The original source code also takes in a variable 'ftype', which modifies coefficients used in the bandpass filter depending 
# on which database (MIT-BIH / AHA) is being tested by the algorithm. In order to use the algorithm on any signal,
# this code omits 'ftype' and instead implements a general-purpose 64-tap bandpass filter with a passband from 5 to 25 Hz. 
#       The original code also takes in 'wtime', an array of time indices. This variable is contained in the matlab file of the waveform,
# so since this code's input waveform file doesn't contain this variable, it is calculated separately. 
#       This function returns an array [loc, time], respectively indicating the indices and times of QRS locations. 

def QRSdetection(wdata, Fs):           
# Algorithm begins with initializing parameters and arrays, these can be adjusted between tests by a user. 
        
        ### Parameters ###
        wsize1 = 0.15            # MAF size for Energy Level Detection, size of 1st MAF
        wsize2 = 0.2             # MAF size for Energy Variation Detection, size of 2nd MAF
        refractory_time = 0.15   # Refractory Period 
        thEL0 = 0.1              # Initial value for EL threshold
        stabLevel = 0.5          # Stabilization Reference Voltage
        r_a = 0.1                # application rate for weight adjustment
        r_b = 0.05               # application rate for weight adjustment
        r_nr = 1.75              # application rate of noise level (for signal threshold)
        r_s = 0.001              # application rate of signal level (for noise threshold)
        r_d = 0.05               # decay rate for adaptive threshold
        r_n = 0.03               # application rate of noise level (for signal threshold)
        Weight = 1               # weight for adjustment signal level 
    
        ### Window Configuration ###
        winsizeEL = round(wsize1*Fs)             # window size for Energy Level (EL)
        winsizeEV = round(wsize2*Fs)             # window size for Energy Variation (EV)
        diffWinsize = winsizeEV - winsizeEL      # difference in window sizes 
        refractoryP = round(refractory_time*Fs)  # refractory period 

        thEVlimit = 1*Fs/(0.2*Fs*20)            
        thEVub = 0.45*Fs/(0.2*Fs*20)            
        thEVlb = -0.45*Fs/(0.2*Fs*20)   
        thEVub2 = 20*Fs/(0.2*Fs*20)
        thEVlb2 = -20*Fs/(0.2*Fs*20)

        ArrayL = 5 
        decayF = 1-1/( (0.40-refractory_time)*Fs ) 

        checker2=0 
        loc = np.asarray([])
        isStart=0
        
        maxV_Buf = None 
        
        ### Preprocessing (bandpass filter, taking the absolute value, and differentiating) ###
        
        cutoffs = np.array([(5/(Fs/2)),(25/(Fs/2))]) 
        b = signal.firwin(64,cutoffs,pass_zero=False) 
        fSig = signal.filtfilt(b, 1, wdata, axis=0)   # Signal after bandpass filter
        sSig = np.sqrt(fSig**2)               # Signal after squaring
        dSig = Fs*np.append([0], np.diff(sSig,axis=0),axis=0) 
        sigLen = len(sSig)
    
        ### Memory Allocation ###
        ELQRS = np.zeros((sigLen,1))
        EVQRS = np.zeros((sigLen,1))
        thEL = np.ones((sigLen,1)) * thEL0    # Threshold for EL (Adaptive threshold)
        thEV = np.zeros((sigLen-1,1))         # Threshold for EV (Hard threshold)
        thN = np.zeros((sigLen,1))
        maxVArray = np.zeros((ArrayL,1))
        maxDifBuf = np.zeros((ArrayL,1))
        minDifBuf = np.zeros((ArrayL,1))
        BUF1 = np.zeros((winsizeEL,1))
        BUF2 = np.zeros((winsizeEV,1))
    
        LargeWin = winsizeEV
    
        maxV = 1
        QRScount = 0
    
        ### Main loop of the algorithm ### 
        for kk in np.arange(LargeWin, sigLen):
            ### Moving average with weight ###            
                if kk == LargeWin:   # Initializes BUF1 to the appropriate length
                        for i in np.arange(len(BUF1), kk-1): 
                                BUF1 = np.append(BUF1, [[0]],axis=0)
                BUF1 = np.append(BUF1, [[(np.sum( sSig[kk-winsizeEL:kk],axis=0 )/winsizeEL)]],axis=0) 
                BUF2 = np.append(BUF2, [[(np.sum( dSig[kk-winsizeEV:kk],axis=0 )/winsizeEV)]],axis=0)
                ELQRS[kk-1] = np.sum( np.copy(BUF1[kk-winsizeEL:kk]),axis=0 )/winsizeEL 
                EVQRS[kk-1] = np.sum( np.copy(BUF2[kk-winsizeEV:kk]),axis=0 )/winsizeEV 
        
            ### Step 1: Energy Level Detection ###
                if isStart == 0 and ELQRS[kk-1] >= thEL[kk-1]:
                        thEL[kk-1] = np.copy(ELQRS[kk-1])  
                        maxV = np.copy(ELQRS[kk-1])       
                        maxP = kk - 1 
                        isStart = 1
                if ELQRS[kk-1] < thN[kk-1]:
                        thN[kk-1] = np.copy(ELQRS[kk-1]) 
                if isStart == 1: 
                        if ELQRS[kk-1] >= maxV:
                                thEL[kk-1] = np.copy(ELQRS[kk-1]) 
                                maxV = np.copy(ELQRS[kk-1])  
                                maxP = kk - 1 
                                Timer = refractoryP
                        else: 
                                Timer = Timer - 1
                                thEL[kk-1] = maxV
                                
                                if Timer == 0:
                                            isStart = 0
                                            checker2 = 1
                                            TimerOfPeak = winsizeEV-(refractoryP-winsizeEL)
                                            maxP_Buf = maxP
                                            maxV_Buf = maxV
            
            ### Step 2: Energy Variation Detection ###
                if checker2 == 1:
                        TimerOfPeak = TimerOfPeak-1
                        if TimerOfPeak == 0:
                                checker2 = 0
                                if maxP_Buf-winsizeEL < 1: 
                                        BufStartP2 = 1
                                else: 
                                        BufStartP2 = maxP_Buf - winsizeEL
                                if maxP_Buf + 2 * diffWinsize > sigLen:                  
                                        BufEndP2 = wdata.size                
                                else: 
                                        BufEndP2 = maxP_Buf + 2*diffWinsize*2
                                DiffSumCheck1 = np.amax(np.copy(EVQRS[(BufStartP2-1):(maxP_Buf+diffWinsize)]),axis=0)  
                                DiffSumCheck2 = np.amin(np.copy(EVQRS[(maxP_Buf+diffWinsize-1):BufEndP2]),axis=0)   
                                if loc.size == 0 or (DiffSumCheck1-DiffSumCheck2>thEVlimit and DiffSumCheck1*DiffSumCheck2<0 and DiffSumCheck1>thEVub and DiffSumCheck2<thEVlb and DiffSumCheck1<thEVub2 and DiffSumCheck2>thEVlb2):  
                                        QRScount = QRScount + 1
                                        loc = np.append(loc, np.copy([maxP_Buf-winsizeEL+2]),axis=0)  
            
                        ### Step 3: Weight Adjustment ### 
                                        maxVArray[(QRScount % ArrayL)] = maxV_Buf
                                        maxDifBuf[(QRScount % ArrayL)] = np.amax(np.copy(EVQRS[(BufStartP2-1):BufEndP2]),axis=0) 
                                        minDifBuf[(QRScount % ArrayL)] = np.amin(np.copy(EVQRS[(BufStartP2-1):BufEndP2]),axis=0) 
                                        if stabLevel > np.mean(maxVArray,axis=0): 
                                                AdujR1 = np.amin([r_a*(stabLevel - np.median(np.copy(maxVArray),axis=0)),r_b*stabLevel],axis=0)  
                                        else: 
                                                AdujR1 = np.amin([r_a*(stabLevel - np.median(np.copy(maxVArray),axis=0)),-1*r_b*stabLevel],axis=0) 
                                        Weight = Weight + AdujR1
                thN[kk] = np.copy(thN[kk-1]) + r_s*np.copy(ELQRS[kk-1]) 
                if maxV_Buf is not None: 
                        thEL[kk] = np.copy(thEL[kk-1]) * ( decayF * (1-r_d*(np.copy(thN[kk-1])/maxV_Buf))) + r_n*np.copy(thN[kk-1])  
                else: 
                        thEL[kk] = np.copy(thEL[kk-1]) * decayF  
                if thEL[kk] < r_nr * thN[kk-1]:
                        thEL[kk] = r_nr * np.copy(thN[kk-1]) 

        return loc 
        


# This is the main part of the program, it tests the algorithm w/ ECG #100 from MIT-BIH database.
# It displays a plot of the raw data and QRS locations, total time elapsed, and average time elapsed per point. 

# Adjust sampling frequency, amount of data to process, and file 
testFs = 360  # Sampling frequency of ECG #100
testlength = 30000  # Number of points from the file to read in 
FILENAME = 'ecg100.txt'

testwave = np.asarray([])
testcounter = 0
with open(FILENAME) as f2: 
        for line in f2: 
                    if testcounter == testlength:
                            break # only reads in first 30,000 points 
                    testcounter = testcounter + 1
                    value = line
                    testwave = np.append(testwave, float(value))
time_start = time.process_time()
testloc = QRSdetection(testwave, testFs)
time_end = time.process_time()
time = np.arange(len(testwave)) / testFs
QRS_time = time[np.ix_(testloc.astype(int))]
QRS_locations = testwave[np.ix_(testloc.astype(int))]
plt.plot(time, testwave, 'b-', QRS_time, QRS_locations, 'ro')
plt.title('MIT-BIH DB #100')
plt.show()
print('Total elapsed time: ' + str(time_end - time_start) + ' seconds')
print('Average time elapsed per data point: ' + str((time_end - time_start)/len(testwave)) + ' seconds') 
    
