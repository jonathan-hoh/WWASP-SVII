import allantools as allan
import numpy as np
import matplotlib.pyplot as plt

tau = np.logspace(-1, 3, 50) #create tau values between .1 and 1000
noise = allan.noise.white(10000) #create large array of white noise
rate = 10 #sample rate of input data in Hz

# use overlapping allen deviation to create a continuous plot

##################################################
#                                                #
# Allan deviation function:                      #
# Inputs: (data, sample rate, data type, taus)   #
# Outputs: (taus used, deviations, error, pairs) #
#                                                #
################################################## 


(tau2, adevs, adev_err, n) = allan.adev(noise, rate, data_type="freq", taus=tau)


plot = plt.loglog(tau2, adevs**2)
plt.errorbar(tau2, adevs**2, yerr = (adev_err**2), ecolor='g')

plt.show()