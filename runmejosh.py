# ==============================================================
# DAQuiri code executable file
#
# Last updated: Ben Schneider 7/2/2024
# Revised by  : Joshua Nuka   14/01/2025
#
# Calls necessary functions from daqfunc.py to do the following:
#   - Configure Pi GPIO pins and DAQ hat(s)
#   - Initialize GPS and cal devices if applicable
#   - Continuously read and save data from hats while trigger
#     is high (configurable w TIMED_TRIAL and RUN_TIME)
#   - Plots raw accelerometer data (configurable w PLOT_FLAG)
#   - Write and save .csv files containing timing ticks
#   - Write and save .bin files containing raw accelerometer data
#   - Copy data from SD card to removeble USB drive when present
#   - Resets the hats
#
#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# ==============================================================

# Import necessary functions
from daqfuncjosh import *
from daqhats import (OptionFlags, HatIDs, HatError)
from daqhats_utils import chan_list_to_mask
import threading
import os
import pigpio
import time
import board
import busio
import adafruit_mcp4725
os.system("sudo systemctl start pigpiod")


# Constants and Flags
TIMED_TRIAL = True          # Switch between "desktop" time-trials and EXT TRIG trials
PLOT_FLAG = True            # Turns accelerometer data plotting ON/OFF
RUN_TIME = 5              # PWM run time in seconds (only applicable in desktop mode)
DEVICE_COUNT = 2         # Number of Hats (Adjust this as needed)
MASTER = 1                  # Address of "Master" hat
SAMPLE_RATE = 51200         # Sample Rate of DAQ HATs (must be a multiple of 256)
PWMCHAN = 18                # BCM pin number of PWM output (GPIO 18)
PWMFREQ = 100              # Frequency of PWM output (Hz)
VCDC = 0.03                 # Duty Cycle of PWM output
CURSOR_BACK_2 = '\x1b[2D'
ERASE_TO_END_OF_LINE = '\x1b[0K'

# Try to remove USB directory in case it already exists (must be empty)
remove_USB_dir()

def main(): # pylint: disable=too-many-locals, too-many-statements, too-many-branches
    """
    Main function executed when the module runs directly.
    """
    # Initialize necessary libraries and configure external devices
    pig = pigpio.pi()
    setup_gpiopins()

    gps_trig = open_GPS()
    [cal_trig, ser_cal] = open_calibration_device()

    #for DAC board initialization
    #i2c= busio.I2C(board.GPIO28, board.GPIO29)           #secondary I2C pins
    #dac= adafruit_mcp4725.MCP4725(i2c)

    # Define necessary components for the DAQ HATs
    samples_per_channel = 0
    hats = []
    chans = []
    options = []

    # Adjust the configuration based on DEVICE_COUNT
    if DEVICE_COUNT == 1:
        hats.append(select_hat_devices(HatIDs.MCC_172, 2)[1])  # Select top HAT (HAT1)
        chans.append({0, 1})  # Channels for HAT1
        options.append(OptionFlags.EXTTRIGGER | OptionFlags.CONTINUOUS | OptionFlags.NOSCALEDATA)

    elif DEVICE_COUNT == 2:
        hats.append(select_hat_devices(HatIDs.MCC_172, 2)[0])  # Select HAT0
        hats.append(select_hat_devices(HatIDs.MCC_172, 2)[1])  # Select HAT1
        chans.append({0, 1})  # Channels for HAT0
        chans.append({0, 1})  # Channels for HAT1
        options.append(OptionFlags.EXTTRIGGER | OptionFlags.CONTINUOUS | OptionFlags.NOSCALEDATA)
        options.append(OptionFlags.EXTTRIGGER | OptionFlags.CONTINUOUS | OptionFlags.NOSCALEDATA)

    try:
        # Configure and sync the DAQ HATs
        configure_hats(MASTER, SAMPLE_RATE, hats, chans, options)

        # Set up GPIO callback for PWM trigger
        pig.set_mode(PWMCHAN, pigpio.INPUT)
        cb18 = pig.callback(PWMCHAN, pigpio.RISING_EDGE, tap_callback)

        # Start the scan
        for i, hat in enumerate(hats):
            chan_mask = chan_list_to_mask(chans[i])
            hat.a_in_scan_start(chan_mask, samples_per_channel, options[i])

        try:
            # Monitor trigger status (low to high)
            wait_for_trigger(1, TIMED_TRIAL, RUN_TIME)

            # Actively save HAT data
            if len(hats)==1:        
                threads = []
                for i in range(len(hats)):
                    t = threading.Thread(target=read_and_save_data_threaded, args=(1, hats[0], chans[0], pig, DEVICE_COUNT)) #the first arg is '1' to select top HAT
                    threads.append(t)
                    t.start()
            else:        
                threads = []
                for i in range(len(hats)):
                    t = threading.Thread(target=read_and_save_data_threaded, args=(i, hats[i], chans[i], pig, DEVICE_COUNT))
                    threads.append(t)
                    t.start()

            # Actively save GPS data
            if gps_trig == 1:
                g = threading.Thread(target=save_gps)
                threads.append(g)
                g.start()
            else:
                print("No GPS data")

            # Actively save CAL data
            if cal_trig == 1:
                c = threading.Thread(target=cal_main, args=(ser_cal,))
                threads.append(c)
                c.start()
            else:
                print("No scale data")

            # Start PWM once trigger is received
            sleep(0.03)
            
            configure_PWM(PWMCHAN, PWMFREQ, VCDC, 1)
            #generate_sine_pulse(dac, duration=RUN_TIME, freq=100,level=1)

            # Monitor trigger status (high to low)
            wait_for_trigger(0, TIMED_TRIAL, RUN_TIME)

            # Stops all threaded data collection
            for thread in threads:
                thread.join()

        except KeyboardInterrupt:
            print(CURSOR_BACK_2, ERASE_TO_END_OF_LINE, '\nAborted\n')

    except (HatError, ValueError) as error:
        print('\n', error)

    finally:
        pig.stop()
        
        # Stop the scan
        for hat in hats:
            hat.a_in_scan_stop()
            hat.a_in_scan_cleanup()
    
        # Close calibration device serial port
        if cal_trig == 1:
            ser_cal.close()

        # Save .csv data to /home/pi/DAQuiri/Data/...
        save_csv_data(hats)

        # Plot data
        if PLOT_FLAG:
            plot_accel_data(DEVICE_COUNT)
        
        # Reset the DAQ Hats
        reset_MCC172()

    # Copy saved data over to USB if present
    copy_data_to_USB(DEVICE_COUNT, gps_trig, cal_trig,hats)

if __name__ == '__main__':
    if TIMED_TRIAL:
        main()
    else:
        while True:
            main()

