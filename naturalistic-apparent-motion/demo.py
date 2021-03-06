#!/usr/bin/env python

''' Description:
Converts vibrations from accelerometers to vibrotactile actuators behaviors in real time
1. each accelerometer is associated to an actuator
2. only the Z-axis is taken into account
3. digest Z-axis values into actuator behavior
'''
# usage:
#   python demo.py 


from multiprocessing.connection import wait
import serial
import shlex
import subprocess
import time
import pathlib


from multiprocessing import Process, Value
# The multiprocessing Queue class is a near clone of Queue.Queue (https://docs.python.org/2/library/multiprocessing.html#multiprocessing.Queue)
from multiprocessing import Queue
# Necessary to handle Empty exceptions (https://stackoverflow.com/questions/6491942/cannot-access-queue-empty-attributeerror-function-object-has-no-attribute)
from queue import Empty

class StateToken:
   def __init__(self):
       self.is_working = True

   def stop(self):
       self.is_working = False

def init(ad5383_driver_bin_path):
    # Ensure the resting state (0V) of the actuators
    subprocess.run([ad5383_driver_bin_path + "neutral/neutral"])


''' Description:
'''
def process_acceleration(q, is_working):
    # Open the USB port to read accelerometer values from the Teensy
    ser=serial.Serial('/dev/ttyACM0', 9600)
    # initializing Delimiter
    delim = ","
    # initializing Z-values index
    idx = [3, 6, 9, 12, 15, 18]

    cpt = 10
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
        
        # time and z-values
        q.put([int(textList[0]), intList])
        cpt = cpt-1

    # Close the USB port and pipe
    ser.close()
    # Send working to False to trigger the end of the other thread
    is_working.value = False
    print ("process_acceleration : done.")
    

''' Description:
'''
def send_instruction_actuators(q, is_working):
    while is_working.value:
        try:
            t, z_values = q.get(block=True, timeout=0.01) #seconds
        except Empty:
            # do nothing: check if is_working is still True
            pass
        else:
            # Handle task here and call q.task_done()    
            print ("Modified list is : " + str(z_values))
    print ("send_instruction_actuators : done.")


''' Description:
'''
if __name__ == '__main__':
    ad5383_driver_bin_path = str(pathlib.Path(__file__).parent.resolve()) + "/../AD5383_driver/build/bin/"
    init(ad5383_driver_bin_path)

    q = Queue()
    is_working = Value('b', True)

    # create threads for the sensors and actuators
    p_processing = Process(target=process_acceleration, args=(q,is_working,))
    p_act = Process(target=send_instruction_actuators, args=(q,is_working,))

    # start workers
    p_processing.start()
    p_act.start()
    
    # wait for all workers to finish their jobs
    p_processing.join()
    p_act.join()
    