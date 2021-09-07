# -*- coding: utf-8 -*-
"""
@author: Jonathan Hoh (Based off of kidpy by Adrian Sinclair)
Initial template for photo-mixer readout on ROACH2 system with 512MHz digizers

"""
import casperfpga
import time
import matplotlib.pyplot as plt
import struct
import numpy as np
import csv

katcp_port=7147
roach = '192.168.40.79'
firmware_fpg = 'lock_in_v2_2021_Apr_29_1629.fpg'
#firmware_fpg = 'lock_in_v1_2021_Mar_25_1417.fpg'
fpga = casperfpga.katcp_fpga.KatcpFpga(roach, timeout = 3.)
time.sleep(1)
if (fpga.is_connected() == True):
    print 'Connected to the FPGA '
else:
    print 'Not connected to the FPGA'

if (fpga.upload_to_ram_and_program(firmware_fpg) == True):
    print 'Uploaded firmware'
else:
    print 'Failed to upload firmware or already uploaded'

# Initializing registers

fpga.write_int('fft_shift', 2**9)
fpga.write_int('mux_select', 0) # 0 for constant, 1 for multiply
fpga.write_int('cordic_freq', 2) # 
fpga.write_int('sync_accum_len', 2**24-1) # 2**19/2**9 = 1024 accumulations
fpga.write_int('sync_accum_reset', 0) #
fpga.write_int('sync_accum_reset', 1) #
fpga.write_int('sync_accum_reset', 0) #
fpga.write_int('start_dac', 0) #
fpga.write_int('start_dac', 1) #

sec_convert = (0.0625)**(-1) #number of 2^24 accum dumps needed for one sec of integration


#t_1ms = int(round(1/ms_convert))
#t_30ms = int(round(30/ms_convert))
#t_300ms = int(round(300/ms_convert))
t_1s = 16

plt.ion()

def plotFFT():
        fig = plt.figure()
        plot1 = fig.add_subplot(111)
        line1, = plot1.plot(np.arange(0,1024,2), np.zeros(1024/2), '#FF4500', alpha = 0.8)
        line1.set_marker('.')
        plt.grid()
        plt.ylim(-10, 100)
        plt.tight_layout()
        count = 0
        stop = 1.0e6
        while(count < stop):
            fpga.write_int('fft_snap_fft_snap_ctrl',0)
            fpga.write_int('fft_snap_fft_snap_ctrl',1)
            fft_snap = (np.fromstring(fpga.read('fft_snap_fft_snap_bram',(2**9)*8),dtype='>i2')).astype('float')
            I0 = fft_snap[0::4]
            Q0 = fft_snap[1::4]
            mag0 = np.sqrt(I0**2 + Q0**2)
            mag0 = 20*np.log10(mag0)
            line1.set_ydata(mag0)
            fig.canvas.draw()
            count += 1
        return

def plotAccum():
        # Generates a plot stream from read_avgIQ_snap(). To view, run plotAvgIQ.py in a separate terminal
        fig = plt.figure(figsize=(10.24,7.68))
        plt.title('TBD, Accum. Frequency = ')# + str(accum_freq), fontsize=18)
        plot1 = fig.add_subplot(111)
        line1, = plot1.plot(np.arange(1016),np.ones(1016), '#FF4500')
        line1.set_linestyle('None')
        line1.set_marker('.')
        plt.xlabel('Channel #',fontsize = 18)
        plt.ylabel('dB',fontsize = 18)
        plt.xticks(np.arange(0,1016,100))
        plt.xlim(0,1016)
	#plt.ylim(-40, 5)
	plt.ylim(-40, 100)
        plt.grid()
        plt.tight_layout()
        plt.show(block = False)
        count = 0
        stop = 10000
        while(count < stop):
            I, Q = read_accum_snap()
            I = I[2:]
            Q = Q[2:]
            mags =(np.sqrt(I**2 + Q**2))[:1016]
            #mags = 20*np.log10(mags/np.max(mags))[:1016]
            mags = 10*np.log10(mags+1e-20)[:1016]
            line1.set_ydata(mags)
            fig.canvas.draw()
            count += 1
        return
        
def read_accum_snap():
        # 2**9 64bit wide 32bits for mag0 and 32bits for mag1    
        fpga.write_int('accum_snap_accum_snap_ctrl', 0)
        fpga.write_int('accum_snap_accum_snap_ctrl', 1)
        accum_data = np.fromstring(fpga.read('accum_snap_accum_snap_bram', 16*2**9), dtype = '>i').astype('float')
        #data type for accum_data may have to be wonked with
		#I = accum_data[0::2]
        #Q = accum_data[1::2]
        return accum_data

def plotADC():
        # Plots the ADC timestream
        fig = plt.figure(figsize=(10.24,7.68))
        plot1 = fig.add_subplot(211)
        line1, = plot1.plot(np.arange(0,2048), np.zeros(2048), 'r-', linewidth = 2)
        plot1.set_title('I', size = 20)
        plot1.set_ylabel('mV', size = 20)
        plt.xlim(0,1024)
        plt.ylim(-600,600)
        plt.yticks(np.arange(-600, 600, 100))
        plt.grid()
        plot2 = fig.add_subplot(212)
        line2, = plot2.plot(np.arange(0,2048), np.zeros(2048), 'b-', linewidth = 2)
        plot2.set_title('Q', size = 20)
        plot2.set_ylabel('mV', size = 20)
        plt.xlim(0,1024)
        plt.ylim(-600,600)
        plt.yticks(np.arange(-600, 600, 100))
        plt.grid()
        plt.tight_layout()
        plt.show(block = False)
        count = 0
        stop = 1.0e8
        while count < stop:
            time.sleep(0.1)
            fpga.write_int('adc_snap_adc_snap_ctrl', 0)
            time.sleep(0.1)
            fpga.write_int('adc_snap_adc_snap_ctrl', 1)
            time.sleep(0.1)
            fpga.write_int('adc_snap_adc_snap_ctrl', 0)
            time.sleep(0.1)
            fpga.write_int('adc_snap_adc_snap_trig', 1)
            time.sleep(0.1)
            fpga.write_int('adc_snap_adc_snap_trig', 0)
            time.sleep(0.1)
            adc = (np.fromstring(fpga.read('adc_snap_adc_snap_bram',(2**10)*8),dtype='>h')).astype('float')
            adc /= (2**15)
            adc *= 550.
            I = np.hstack(zip(adc[0::4],adc[2::4]))
            Q = np.hstack(zip(adc[1::4],adc[3::4]))
            line1.set_ydata(I)
            line2.set_ydata(Q)
            fig.canvas.draw()
            count += 1
        return
        
def dataCollect(chan):
    rate = 16
    cycles = 10000
    tau = np.logspace(-1, 3, 50)
    vals = np.zeros(rate*cycles)
    count = 0
    while (count < rate*cycles):
        fpga.write_int('accum_snap_accum_snap_ctrl', 0)
        fpga.write_int('accum_snap_accum_snap_ctrl', 1)
        accum_data = np.fromstring(fpga.read('accum_snap_accum_snap_bram', 16*2**9), dtype = '>i').astype('float')
        val = accum_data[chan]
        vals[count] = val
        count += 1
    file = open('spec_data.csv', 'w')
    writer = csv.writer(file)
    writer.writerow(vals)
    file.close()
    
    
        
