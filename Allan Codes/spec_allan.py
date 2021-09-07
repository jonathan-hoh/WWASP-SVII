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
skips = 100
df = pd.read_csv("Hot.csv",skiprows=[i for i in range(1,skips)]
)
# Rows are skipped to throw out junk data from adrians spectrometer
# Total of N rows of useful info 
# Columns labeled and referenced as 'Bin XXXX' where X is between 1 and N

dead_cols = 3 # number of useless columns in dataset
bins = len(df.columns) - dead_cols
num_sec = len(df)

tau = np.logspace(0, (np.log10(num_sec/5)), 30)  # Create a list of taus from 1 to N seconds
#cen_bin = df['Bin %d'%(bins/2)].values   # Do allan variance on central bin like bin 2000
cen_bin = df['Bin 1000'].values
off_bin = df['Bin 500'].values
rate = 1                             # Data is taken once per second

row = df.iloc[500]

row.plot()
plt.ylim(0,2*10**9)
# Make sure to use overlapping alan variance to make all of the data pretty

##################################################
#                                               #
# Overlapping Allan deviation function:          #
# Inputs: (data, sample rate, data type, taus)   #
# Outputs: (taus used, deviations, error, pairs) #
#                                                #
################################################## 

(tau2, adevs, adev_err, n) = AT.oadev(off_bin/cen_bin, rate, data_type="freq", taus=tau)

avars = np.square(adevs)  #square allan dev to get allan var

# Now make white noise set for comparison line with power density equivalent to the data values

#noise = AT.noise.white(100000, b0=(2*avars[0]))
#(tau3, wh_devs, wh_adev_err, wh_n) = AT.oadev(noise, rate, data_type="freq", taus=tau)
#wh_vars = np.square(wh_devs)


###plot = plt.loglog(tau2, avars)
###plt.loglog(tau2,(avars[0]*(tau2**-1))) # actually we finna cheat and just plot t^-1 line :P
#plt.loglog(tau2, wh_vars)

#plt.errorbar(tau2, avars, yerr = 10**(np.square(adev_err)/10), ecolor='g')
#plt.show()
#plt.title('Allan Variance for SVII (Bin %d)(%d samples tossed)(external clock)'%(1000, skips))
#plt.title('Allan Variance for SVII dividing bin 500 by bin 1000')
plt.title('Allan Variance for SVII with Ext Clk (Bin 1000)')
plt.xlabel('Integration Times (s)')
plt.ylabel('Probably Power^2')
#plot = plt.loglog(tau2, avars)
#plt.loglog(tau2,(avars[0]*(tau2**-0.5)))
###plt.show()

#Find the actual allan time by finding when the allan variation deviates by 10% from t^-1 line
##white_line = (avars[0]*(tau2**-1))
##log_diff_pct = ((np.log10(avars) - np.log10(white_line))/np.log10(white_line))
##plt.figure(121)
##plt.plot(np.log10(tau2), log_diff_pct)

##plt.figure(131)
##diff_pct = ((avars - white_line)/white_line)
##plt.loglog(tau2, diff_pct)
##allan_time = np.argmin(np.abs(log_diff_pct-0.10))
##allan_time2 = np.argmin(np.abs(diff_pct-10))
#allan_time = list(log_diff_pct).index()
##print(tau2)
##print(tau2[allan_time])
##print(tau2[allan_time2])

