import csv
import allantools as allan
import numpy as np
import matplotlib.pyplot as plt

stream = []
def noise2file():
    noise = allan.noise.white(10000) #create large array of white noise
    file = open('test_data.csv', 'w')
    writer = csv.writer(file)
    writer.writerow(noise)
    file.close()
    
def readnoise():
    tau = np.logspace(-1, 3, 50)
    rate = 100
    with open('test_data.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
        data = next(csv_reader)
        (tau2, adevs, adev_err, n) = allan.oadev(data, rate, data_type="freq", taus=tau)
        plot = plt.loglog(tau2, adevs)
        plt.show()
        #stream = data
    
    
def allantest(data):
    tau = np.logspace(-1, 3, 50)
    rate = 100
    (tau2, adevs, adev_err, n) = allan.oadev(data, rate, data_type="freq", taus=tau)
    plot = plt.loglog(tau2, adevs)
    plt.show()