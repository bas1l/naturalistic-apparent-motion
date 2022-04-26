#!/usr/bin/env python

''' Description:
Converts vibrations from accelerometers to vibrotactile actuators behaviors in real time
1. each accelerometer is associated to an actuator
2. only the Z-axis is taken into account
3. digest Z-axis values into actuator behavior
'''
# usage:
#   python demo.py 


import serial
import shlex, subprocess

from multiprocessing import Process, Pipe

def init():
    # Ensure the resting state (0V) of the actuators
    subprocess.run(["/home/pi/Software/HaptiComm/build/0_executables/neutral/neutral"])


def process_acceleration(child_conn):
    # Open the USB port to read accelerometer values from the Teensy
    ser=serial.Serial('/dev/ttyACM0', 9600)
    # initializing Delimiter
    delim = ","
    # initializing time and Z-values index
    idx = [0, 3, 6, 9, 12, 15, 18]

    cpt = 100
    while cpt:
        readedByte = ser.readline()
        # From Byte to String
        readedText = readedByte.decode()
        # Convert into a list 
        textList = readedText.split(delim)
        # if the entire message failed to reach the Raspberry PI
        if (len(textList)<18):
            continue
        # conserve only interesting values
        textListShort = []
        for i in idx:
            textListShort.append(textList[i])

        # converting list of string into list of numbers
        intList = list(map(int, textListShort))
        print ("Modified list is : " + str(intList))
        
        child_conn.send(intList)
        cpt = cpt-1

    # Close the USB port and pipe
    ser.close()
    child_conn.close()

    
def send_instruction_actuators(parent_conn):
    cpt=3
    while cpt:
        subprocess.run(["/home/pi/Software/HaptiComm/build/0_executables/neutral/neutral"])
        cpt=cpt-1


if __name__ == '__main__':
    init()

    # create pipes between acc/process and process/act
    bidirectional_val = False
    parent_conn, child_conn = Pipe(bidirectional_val)

    # create threads for the sensors and actuators
    p_processing = Process(target=process_acceleration, args=(child_conn,))
    p_act = Process(target=send_instruction_actuators, args=(parent_conn,))

    # start workers
    p_processing.start()
    p_act.start()
    
    # wait for all workers to finish their jobs
    p_processing.join()
    p_act.join()
    