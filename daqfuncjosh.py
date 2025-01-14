# ==============================================================
# DAQuiri code functions
#
# Last updated: Ben Schneider 7/2/2024
# Revised by  : Joshua Nuka   14/01/2025
#
# Contains all functions called by "runme.py". Organized into 
# Interface Functions and File Saving Functions
#
#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# ==============================================================

# Import necessary functions
from time import sleep
from daqhats import (hat_list, mcc172, OptionFlags, TriggerModes, HatError, SourceType)
from daqhats_utils import (enum_mask_to_string, validate_channels)
import time
import math
import datetime
import RPi.GPIO as GPIO
import pigpio
import csv
import serial
import subprocess
import os
import shutil
import numpy as np
import matplotlib.pyplot as plt


# Create and initialize global variables
pig = pigpio.pi()
tstart = time.gmtime()
ALL_GOOD = 1
TIMING_TICKS_0 = []
TIMING_TICKS_1 = []
TRIG_RISE_TICK = 0
TRIG_FALL_TICK = 0
TAP_TICK = []
filestub_SD = '/home/pi/DAQuiri/Data/'
filestub_USB = '/media/pi/DATA/'
sample_sizes_0 = []
sample_sizes_1 = []
total_data_0 = []
total_data_1 = []
start_time_0 = 0
stop_time_0 = 0
start_time_1 = 0
stop_time_1 = 0

#===================================================================================================================================
#
#   DAQ / GPIO Interface Functions
#
#===================================================================================================================================

# Function to configure GPIO pins inherently used by MCC172 DAQ HATs
def setup_gpiopins(): 
    # Set up GPIO pin mode to use the chip's native numbering (BCM)
    GPIO.setmode(GPIO.BCM) 
    # Stop it from warning us about overwriting settings
    GPIO.setwarnings(False)
    # GPIO 5, 6, 19, 20 - INPUT for measuring: Trigger, Sync, Clock, IRQ
    GPIO.setup([5,6,19,20,21], GPIO.IN)
    # GPIO 16- OUTPUT for controlling: RESET of the MCC172 board, Desktop Mode Trigger
    GPIO.setup([16,4], GPIO.OUT)
    # GPIO 18 - INPUT for measuring PWM tapper pulses
    GPIO.setup([18], GPIO.OUT)

# Function to toggle Pin GPIO16 high to reset the MCC172 board 
def reset_MCC172(): 
    # global ALL_GOOD
    # ALL_GOOD = 1
    GPIO.output(16,1)
    sleep(0.1)
    GPIO.output(16,0)
    sleep(1.0)

# Function to open GPS serial ports and return status flag
def open_GPS():
    try:
        cmd=["str2str","-in","serial://ttyACM0:230400#ubx","-out", "rover.ubx", "-c", "/home/pi/DAQuiri/Programs/m8t_10hz_usb.cmd"]
        result = subprocess.run(cmd,timeout = 1)
        result.check_returncode()    
    except subprocess.TimeoutExpired:
        print("GPS config loaded")
        gps_trig = 1
        pass
    except subprocess.CalledProcessError:
        #print("GPS module not connected")
        gps_trig = 0
        pass

    return gps_trig

# Function to open Calibration device serial ports and return status flag
def open_calibration_device():
    ser_cal = None
    try:
        ser_cal = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)
        print("Calibration device connected and loading")
        time.sleep(4) # Sleep time needed to allow calibration device arduino to power on and run code
        print("Calibration device loaded")
        cal_trig = 1
    except:
        cal_trig = 0
        #print("Calibration device not connected")
        pass
    
    return [cal_trig,ser_cal]

# Function that holds off data collection until external trigger is received
def wait_for_trigger(level, TIMED_TRIAL, RUN_TIME):   
    
    # Global variables
    global pig
    global TRIG_RISE_TICK
    global TRIG_FALL_TICK

    # Read status of GPIO 5 and move forward after low-to-high change
    if level == 1:
        if TIMED_TRIAL:
            print(RUN_TIME,"second timed trial beginning...")
            sleep(1.0)
            GPIO.output(4, GPIO.HIGH)
            TRIG_RISE_TICK = pig.get_current_tick()
            print('Trigger received!')
        else:
            print('Waiting for trigger...')
            while GPIO.input(5) == 0:
                pass
            TRIG_RISE_TICK = pig.get_current_tick()
            print('Trigger received!')
    else:
        if TIMED_TRIAL:
            sleep(RUN_TIME)
            GPIO.output(4,GPIO.LOW)
            TRIG_FALL_TICK = pig.get_current_tick()
            #generate_sine_pulse(dac, duration=RUN_TIME, freq=100,level=0)
            configure_PWM(18, 100, 0.03, 0)
            print('Trigger removed!')
        else:
            while GPIO.input(5) == 1:
                pass
            TRIG_FALL_TICK = pig.get_current_tick()
            #generate_sine_pulse(dac, duration=RUN_TIME, freq=100,level=0)
            configure_PWM(18, 100, 0.03, 0)
            print('Trigger removed!')

# Callback function for tap events
def tap_callback(gpio, level, tick):
    global TAP_TICK
    TAP_TICK.append(tick)

# Function that turns PWM on/off
def configure_PWM(PWMCHAN, PWMFREQ, VCDC, level):
    global pig
    pig.hardware_PWM(PWMCHAN, PWMFREQ, int(VCDC*1e6*level) ) # GPIO 18, 100Hz, dutycycle (DC*10e6) ==> DC = 0.03 yields 0.3 ms pulses

def generate_sine_pulse(dac, duration, freq, level):
    """
    Generates sine wave pulses through DAC.
    dac: DAC object to output the analog signal.
    duration: Total duration of the pulse in seconds.
    freq: frequency of the sine wave pulses(Hz).
    level: controls whether the output is active91) or stopped(0)
    """
    V_MAX = 4095           # Max value for 12-bit DAC
    NUM_SAMPLES = 100      #Number of samples per sine wave cycle
    PERIOD = 1/freq        #time for one sine wave cycle

    #precompute sine wave values for one cycle
    sine_wave= [
        int((V_MAX/2)*(1+math.sin(2*math.pi*i/NUM_SAMPLES)))
        for i in range(NUM_SAMPLES)
    ]
    start_time = time.time()
    while (time.time() - start_time) < duration:
        if level == 1:
            # Generate the waveform
            for value in sine_wave:
                dac.value = value
                time.sleep(PERIOD/NUM_SAMPLES)
        else:
            # Stop PWM if level == 0
            dac.value = 0
            break

# Function that identifies and selects connected Hats
def select_hat_devices(filter_by_id, number_of_devices):
    
    selected_hats = []

    # Get descriptors for all of the available HAT devices.
    hats = hat_list(filter_by_id=filter_by_id)
    number_of_hats = len(hats)

    # Verify at least one HAT device is detected.
    if number_of_hats < number_of_devices:
        error_string = ('Error: This example requires {0} MCC 172 HATs - '
                        'found {1}'.format(number_of_devices, number_of_hats))
        raise HatError(0, error_string)
    for i in range(number_of_devices):
        selected_hats.append(mcc172(hats[i].address))

    return selected_hats

# Function that configures both MCC172 DAQ HATs to run synchronously
def configure_hats(MASTER,SAMPLE_RATE, hats, chans, options):
    
    # Define necessary variables for Hat config
    iepe_enable_bychan = [[1,1],[1,1]]
    trigger_mode = TriggerModes.RISING_EDGE
    
    # Validate the selected channels.
    for i, hat in enumerate(hats):
        validate_channels(chans[i], hat.info().NUM_AI_CHANNELS)

    # If we only have 1 device, configure just Hat 1.
    if len(hats) == 1:
        hat = hats[0]
        for j, channel in enumerate(chans[0]):
            # Configure IEPE.
            hat.iepe_config_write(channel, iepe_enable_bychan[0][j])
        # Configure the master clock and trigger
        hat.a_in_clock_config_write(SourceType.MASTER, SAMPLE_RATE)
        hat.trigger_config(SourceType.MASTER, trigger_mode)
	
	# Set the device options for the single HAT
        options_str = enum_mask_to_string(OptionFlags, options[0])
        #hat.option_flags_write(options[0])
    else:
        # Otherwise, configure both HATs
        for i, hat in enumerate(hats):
            for j, channel in enumerate(chans[i]):
                hat.iepe_config_write(channel, iepe_enable_bychan[i][j])
            if hat.address() != MASTER:
                # Configure the slave clocks.
                hat.a_in_clock_config_write(SourceType.SLAVE, SAMPLE_RATE)
                hat.trigger_config(SourceType.SLAVE, trigger_mode)
	    # Set the device options for each HAT
            options_str = enum_mask_to_string(OptionFlags, options[i])
            #hat.option_flags_write(options[i])  # This line replaces configure_hat if needed

        # Configure the master clock and start the Master HAT sync.
        hats[MASTER].a_in_clock_config_write(SourceType.MASTER, SAMPLE_RATE)
        synced = False
        while not synced:
            (_source_type, actual_rate, synced) = hats[MASTER].a_in_clock_config_read()
            if not synced:
                sleep(0.005)

        # Configure the master trigger
        hats[MASTER].trigger_config(SourceType.MASTER, trigger_mode)
	
	# Set the master options
        options_str = enum_mask_to_string(OptionFlags, options[MASTER])
        #hats[MASTER].option_flags_write(options[MASTER])  # This line replaces configure_hat if needed

# Function that decodes GPS data
def decode_gga(gga_msg,rover_timestamp):
    msg_list=gga_msg.split(',')
    if len(msg_list)<10:
        return [0,rover_timestamp,0,0,0]
    if not msg_list[2] == "" and not msg_list[4] == "":
        time_str=msg_list[1]
        gps_time1 = float(time_str[2:4])*60 + float(time_str[4:-1])
        gps_time = datetime.datetime.fromtimestamp(gps_time1)
        latitude = msg_list[2]
        latitude = int(latitude[0:2])+float(latitude[2:-1])/60
        if msg_list[3] == 'S':
            latitude = -latitude
        longitude = msg_list[4]
        longitude = int(longitude[1:3])+float(longitude[3:-1])/60
        if msg_list[5] == 'W':
            longitude = -longitude
        height = msg_list[9]
        return [gps_time.strftime("%H-%M-%S.%f"),rover_timestamp,longitude,latitude,height]
    return [0,rover_timestamp,0,0,0]

# Function that pulls relevant GPS data
def find_gga(rover_data):
    gga_data=""
    for i in range(len(rover_data)-6):
        if rover_data[i:i+6] == b"$GNGGA":
            raw_gga_msg = ""
            j = i
            while not rover_data[j] == '*' and j<len(rover_data)-4:
                raw_gga_msg = raw_gga_msg+chr(rover_data[j])
                j = j+1
            raw_gga_msg = raw_gga_msg+chr(rover_data[j])+chr(rover_data[j+1])+chr(rover_data[j+2])
            j = j+3
            gga_data = decode_gga(raw_gga_msg,str(datetime.datetime.now()))
    return gga_data


#===================================================================================================================================
#
#   File Saving Functions
#
#===================================================================================================================================

# Read and save HAT data (accelerometer data)
def read_and_save_data_threaded(tid, hat, chans, pig, DEVICE_COUNT):
    
    # Global Variables
    global sample_sizes_0
    global sample_sizes_1
    global total_data_0
    global total_data_1
    global start_time_0
    global stop_time_0
    global start_time_1
    global stop_time_1
    global filestub_tensio_SD
    global filestub_tensio_USB
    filestub_tensio_SD = filestub_SD + 'Tensio/Tensio_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart) + '_Hat{}'.format(tid)
    filestub_tensio_USB = filestub_USB + 'Tensio/Tensio_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart) + '_Hat{}'.format(tid)
    
    # Internal Variables
    samples_to_read = -1 # read all available samples
    timeout = 0  # pull samples as soon as they are available
    samples_per_chan_read = 0
    ALL_GOOD = 1

    with open(filestub_tensio_SD + '.bin', 'wb') as f:
        
        if DEVICE_COUNT == 1:
            if tid == 1:
                start_time_0 = pig.get_current_tick()
                while GPIO.input(5) == 1:
                    # Initialize data array for Hat 1 only
                    data = [None] * 1
                    read_result = hat.a_in_scan_read(samples_to_read, timeout)
                    data = read_result.data
                    samples_per_chan_read = int(len(data) / len(chans))
                    if samples_per_chan_read > 0:
                        sample_sizes_0.append(samples_per_chan_read)
                        total_data_0 += data
                        b = np.int32(data)
                        f.write(b)

        elif DEVICE_COUNT == 2:
            # Handle for both HATs
            if tid == 0:
                start_time_0 = pig.get_current_tick()
            else:
                start_time_1 = pig.get_current_tick()

            while GPIO.input(5) == 1:
                # Initialize data array
                data = [None] * DEVICE_COUNT
                # Read the data from each HAT device (all available samples)
                read_result = hat.a_in_scan_read(samples_to_read, timeout)
                data = read_result.data
                samples_per_chan_read = int(len(data) / len(chans))

                if samples_per_chan_read > 0:
                    if tid == 0:
                        sample_sizes_0.append(samples_per_chan_read)
                        total_data_0 += data
                    else:
                        sample_sizes_1.append(samples_per_chan_read)
                        total_data_1 += data

                    b = np.int32(data)
                    f.write(b)

            if tid == 0:
                stop_time_0 = pig.get_current_tick()
            else:
                stop_time_1 = pig.get_current_tick()

    print('Hat{}'.format(tid) + ' tensio data saved \n')

# Plot accelerometer data with respect to taps
def plot_accel_data(DEVICE_COUNT):
    
    # Check if DEVICE_COUNT == 1 or 2 to plot data accordingly
    global total_data_0
    global total_data_1
    if DEVICE_COUNT == 1:
        # Plot data for Hat 1 only
        if len(total_data_0) % 2 == 1:
            total_data_0.pop()
        total_samples_0 = int(len(total_data_0) / 2)
        run_time_0 = (total_samples_0 / 51200)
        time0 = np.arange(0, run_time_0, 1 / 51200)

        data0ch0 = []
        data0ch1 = []
        for i in range(len(total_data_0)):
            if i % 2 == 0:
                data0ch0.append(total_data_0[i])
            else:
                data0ch1.append(total_data_0[i])

        y00 = np.array(data0ch0)
        y01 = np.array(data0ch1)

        fig, axs = plt.subplots(1, 1, constrained_layout=True)
        axs.plot(time0, y00, 'b', time0, y01, 'g')
        axs.set_title('Hat 1 Data')
        fig.suptitle('Accelerometer Data with Tap Rising Edges')
        plt.show()

    elif DEVICE_COUNT == 2:
        # Plot data for both HATs (Hat 0 and Hat 1)
        if len(total_data_0) % 2 == 1:
            total_data_0.pop()
        total_samples_0 = int(len(total_data_0) / 2)
        run_time_0 = (total_samples_0 / 51200)
        time0 = np.arange(0, run_time_0, 1 / 51200)

        if len(total_data_1) % 2 == 1:
            total_data_1.pop()
        total_samples_1 = int(len(total_data_1) / 2)
        run_time_1 = (total_samples_1 / 51200)
        time1 = np.arange(0, run_time_1, 1 / 51200)

        data0ch0 = []
        data0ch1 = []
        for i in range(len(total_data_0)):
            if i % 2 == 0:
                data0ch0.append(total_data_0[i])
            else:
                data0ch1.append(total_data_0[i])

        data1ch0 = []
        data1ch1 = []
        for i in range(len(total_data_1)):
            if i % 2 == 0:
                data1ch0.append(total_data_1[i])
            else:
                data1ch1.append(total_data_1[i])

        y00 = np.array(data0ch0)
        y01 = np.array(data0ch1)
        y10 = np.array(data1ch0)
        y11 = np.array(data1ch1)

        fig, axs = plt.subplots(2, 1, constrained_layout=True)
        axs[0].plot(time0, y00, 'b', time0, y01, 'g')
        axs[0].set_title('Hat 0 Data')
        axs[1].plot(time1, y10, 'b', time1, y11, 'g')
        axs[1].set_title('Hat 1 Data')
        fig.suptitle('Accelerometer Data with Tap Rising Edges')
        plt.show()
    
# Save HAT, tap, and trig data into .csv files
def save_csv_data(hats):
    
    # Global variables
    global filestub_ticks_SD
    global filestub_ticks_USB
    global start_time_0
    global stop_time_0
    global start_time_1
    global stop_time_1
    filestub_ticks_SD = filestub_SD + 'Tensio/Tensio_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart)
    filestub_ticks_USB = filestub_USB + 'Tensio/Tensio_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart)

    # Handle saving timing data for DAQ 1 (Hat1) if only one DAQ is available
    if len(hats) == 1:  # Only Hat1 (DAQ 1)
        with open(filestub_ticks_SD + '_Timing_Hat1.csv', 'w') as f:
            # Append relevant ticks in the order in which they occur
            TIMING_TICKS_1.append(TRIG_RISE_TICK)
            TIMING_TICKS_1.append(start_time_1)
            for i in TAP_TICK:
                TIMING_TICKS_1.append(i)
            TIMING_TICKS_1.append(TRIG_FALL_TICK)
            TIMING_TICKS_1.append(stop_time_1)
            # create the csv writer
            writer = csv.writer(f)
            # write a row to the csv file
            writer.writerow(TIMING_TICKS_1)

    # Handle saving timing data for both DAQ 0 (Hat0) and DAQ 1 (Hat1) if two DAQs are available
    elif len(hats) == 2:  # Both Hat0 and Hat1 (DAQ 0 and DAQ 1)
        # Save timing data for DAQ 0 (Hat0)
        with open(filestub_ticks_SD + '_Timing_Hat0.csv', 'w') as f:
            # Append relevant ticks in the order in which they occur
            TIMING_TICKS_0.append(TRIG_RISE_TICK)
            TIMING_TICKS_0.append(start_time_0)
            for i in TAP_TICK:
                TIMING_TICKS_0.append(i)
            TIMING_TICKS_0.append(TRIG_FALL_TICK)
            TIMING_TICKS_0.append(stop_time_0)
            # create the csv writer
            writer = csv.writer(f)
            # write a row to the csv file
            writer.writerow(TIMING_TICKS_0)

        # Save timing data for DAQ 1 (Hat1)
        with open(filestub_ticks_SD + '_Timing_Hat1.csv', 'w') as f:
            # Append relevant ticks in the order in which they occur
            TIMING_TICKS_1.append(TRIG_RISE_TICK)
            TIMING_TICKS_1.append(start_time_1)
            for i in TAP_TICK:
                TIMING_TICKS_1.append(i)
            TIMING_TICKS_1.append(TRIG_FALL_TICK)
            TIMING_TICKS_1.append(stop_time_1)
            # create the csv writer
            writer = csv.writer(f)
            # write a row to the csv file
            writer.writerow(TIMING_TICKS_1)
# Save GPS data
def save_gps():
    global tstart
    global filestub_gps_SD
    global filestub_gps_USB
    filestub_gps_SD = filestub_SD + 'GPS/GGA_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart)
    filestub_gps_USB = filestub_USB + 'GPS/GGA_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart)

    ser = serial.Serial(
            port = '/dev/ttyACM0',
            baudrate = 230400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout = 1
            )
    
    with open(filestub_gps_SD + '.txt', 'w') as gga_file:
        while ALL_GOOD & GPIO.input(5):
            timestamp = time.time()
            string_timestamp = "$GPGST,"+str(timestamp)+",,,,,,,*FF\r\n" 
            if ser.inWaiting()>0:

                raw_data = ser.read(ser.inWaiting())
                gga_msg = find_gga(raw_data)
                if not gga_msg=="":
                    gga_file.write(str(gga_msg)+"\n")
                    gga_file.flush()
        ser.close()
    print('GPS data saved \n')

# Calibration Functions
def cal_main(ser_cal):
    global tstart
    global filestub_scale_SD
    global filestub_scale_USB
    flag_start_running = 0
    count = 0
    while ALL_GOOD:
        if ser_cal.inWaiting()>0:
            try:
                raw_cal_data = ser_cal.read(ser_cal.inWaiting())
                #print(raw_cal_data)
                raw_cal_data = raw_cal_data.decode('utf-8')
            except:
                raw_cal_data=""
            if (not raw_cal_data==""):
                flag_start_running = 1
                if count==0:
                    filestub_scale_SD = filestub_SD + 'Scale/Scale_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart)
                    filestub_scale_USB = filestub_USB + 'Scale/Scale_' + time.strftime('%Y-%m-%d_%H-%M-%S',tstart)
                    cal_file = open(filestub_scale_SD + '.txt', 'w')
                    timestamp = str(datetime.datetime.now())
                    cal_file.write(timestamp+'\n')
                cal_file.write(raw_cal_data)
                cal_file.flush()
                count = count+1
            else:
                ser_cal.reset_input_buffer()
        if GPIO.input(5)==0 and flag_start_running==1:
            timestamp = str(datetime.datetime.now())
            cal_file.write(timestamp+'\n')
            cal_file.close()
            break
        # time.sleep(0.01)

    print('Scale data saved \n')

# Remove any existing USB directory
def remove_USB_dir():
    USB_dir_exists = os.path.exists(filestub_USB)
    if USB_dir_exists: 
        os.system('sudo umount ' + filestub_USB)     #unmount that mount point as well
        os.system('sudo rmdir ' + filestub_USB)

# Copy data to USB
def copy_data_to_USB(DEVICE_COUNT, gps_trig, cal_trig,hats):
    # Create, mount, save, and remove USB directory
    # This prevents errors due to multiple ghost "DATA" USB drives being present in /media/pi
    # Note: Automounting of USB drives has been disabled for this to work correctly.

    # Check if USB is connected
    usb_connected = []
    usb_connected = os.path.exists('/dev/sda1')

    if usb_connected:
        try:
            # Create directory for mount point and make user "pi" the folder owner
            os.system('sudo mkdir ' + filestub_USB)
            os.system('sudo chown pi:pi ' +filestub_USB)
            # Mount USB drive to directory
            os.system('sudo mount -o uid=pi,gid=pi /dev/sda1 ' + filestub_USB)
            
            # Copy Tensio data to USB
            for i in range(DEVICE_COUNT):
                if DEVICE_COUNT==2:
                    shutil.copy2(filestub_tensio_SD[:-1] + '{}.bin'.format(i), filestub_tensio_USB[:-1] + '{}.bin'.format(i))
                else:
                    shutil.copy2(filestub_tensio_SD[:-1] + '{}.bin'.format(1), filestub_tensio_USB[:-1] + '{}.bin'.format(1))
            # Copy Timing data based on number of DAQs
            if len(hats) == 1:  # Only one DAQ, save data for Hat1
                shutil.copy2(filestub_ticks_SD + '_Timing_Hat1.csv', filestub_ticks_USB + '_Timing_Hat1.csv')
            elif len(hats) == 2:  # Two DAQs, save data for both Hat0 and Hat1
                shutil.copy2(filestub_ticks_SD + '_Timing_Hat0.csv', filestub_ticks_USB + '_Timing_Hat0.csv')
                shutil.copy2(filestub_ticks_SD + '_Timing_Hat1.csv', filestub_ticks_USB + '_Timing_Hat1.csv')
            # Copy GPS file to USB
            if gps_trig == 1:
                shutil.copy2(filestub_gps_SD + '.txt', filestub_gps_USB + '.txt')
            # Copy Scale file to USB
            if cal_trig == 1:
                shutil.copy2(filestub_scale_SD + '.txt', filestub_scale_USB + '.txt')
            print('Data saved to USB drive.')
            
            # Unmount USB drive
            sleep(0.5)
            os.system('sudo umount -f ' + filestub_USB)
            # Remove directory
            os.system('sudo rmdir ' + filestub_USB)
        except Exception as error:
            print(error)    
    else:
        print("USB not connected")
