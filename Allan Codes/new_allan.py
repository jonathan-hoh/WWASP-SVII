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
df = pd.read_csv("allan_spec_data_big.csv")
#print(len(df['Bin 500']))



# tau = np.logspace(0, 1.773, 50)  # Create a list of taus from 1 to 60 seconds
# timestream = df['Bin 2000'].values   # Do allan variance on central bin like bin 2000
# myrate = 1  
# data = AT.Dataset(timestream, rate=myrate, data_type='freq', taus = tau)

 
# # Make sure to use overlapping alan variance to make all of the data pretty

# data.compute("oadev")
# #noise = AT.Dataset(data=AT.noise.white(100000), rate=(myrate*(100000/len(timestream))), data_type='freq', taus=tau)
# #noise.compute("oadev")
# var = AT.Plot()
# data.out["stat"] = np.square(data.out["stat"])
# data.out["stat_err"] = np.square(data.out["stat_err"])
# var.plot(data, True, False)
# #noise.out["stat"] = noise.out["stat"]*data.out["stat"][0]
# #var.plot(noise)
# var.show()

#plt.plot(df['Bin 2000'].values-np.mean(df['Bin 2000'].values))
#plt.plot(df['Bin 1000'].values-np.mean(df['Bin 1000'].values))
plt.plot(df['Bin 3500'].values)
plt.title('Timestream of bins 1000 and 2000')
plt.xlabel('Time(s)')
plt.ylabel('Power in bin')
#plot = plt.loglog(tau2, avars)
#plt.plot(df['Bin 1000'].values-df['Bin 2000'].values))
plt.show()

## TODO plot the slope of every bin and plot the slopes of the bins vs bin number