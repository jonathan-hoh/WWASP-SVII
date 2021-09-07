# -*- coding: utf-8 -*-
"""
Created on Fri Aug 27 08:55:53 2021

@author: hohjo
"""

import pandas as pd
import allantools as AT
import numpy as np
import matplotlib.pyplot as plt
#########################################
#                                       #
# Data formatting for readout:          #
# timestamp, runtime, count(secs), bins #
# 4096 bins in spectrometer             #
# But we throw away 2 edge bins         # 
# Total of 4094 bins in data            #
#                                       #
#########################################
def allan_test(testbin, skips):
    df = pd.read_csv("allan_spec_data_big.csv",skiprows=[i for i in range(1,skips)]
    )
    # Rows are skipped to throw out junk data from adrians spectrometer
    # Total of N rows of useful info 
    # Columns labeled and referenced as 'Bin XXXX' where X is between 1 and N
    
    dead_cols = 3 # number of useless columns in dataset
    bins = len(df.columns) - dead_cols
    num_sec = len(df)
    
    tau = np.logspace(0, (np.log10(num_sec/10)), 12)  # Create a list of taus from 1 to N seconds
    testbinval = df['Bin %d'%(testbin)].values   # Do allan variance on central bin like bin 2000
    #cen_bin = df['Bin 3000'].values
    rate = 1                              # Data is taken once per second
    
    # Make sure to use overlapping alan variance to make all of the data pretty
    
    ##################################################
    #                                                #
    # Overlapping Allan deviation function:          #
    # Inputs: (data, sample rate, data type, taus)   #
    # Outputs: (taus used, deviations, error, pairs) #
    #                                                #
    ################################################## 
    
    (tau2, adevs, adev_err, n) = AT.oadev(testbinval, rate, data_type="freq", taus=tau)
    
    avars = np.square(adevs)  #square allan dev to get allan var
    
    # Now make white noise set for comparison line with power density equivalent to the data values
    
    noise = AT.noise.white(100000, b0=(2*avars[0]))
    (tau3, wh_devs, wh_adev_err, wh_n) = AT.oadev(noise, rate, data_type="freq", taus=tau)
    wh_vars = np.square(wh_devs)
    
    
    plot = plt.loglog(tau2, avars)
    plt.loglog(tau2,(avars[0]*(tau2**-1))) # actually we finna cheat and just plot t^-1 line :P
    plt.loglog(tau2, wh_vars)
    
    plt.errorbar(tau2, avars, yerr = (np.square(adev_err)), ecolor='g')
    #plt.show()
    plt.title('Allan Variance for SVII (Bin %d)(%d samples tossed)'%(testbin, skips))
    plt.xlabel('Integration Times (s)')
    plt.ylabel('Probably Power or Power^2 or something IDK, sue me')
    #plot = plt.loglog(tau2, avars)
    #plt.loglog(tau2,(avars[0]*(tau2**-0.5)))
    plt.show()