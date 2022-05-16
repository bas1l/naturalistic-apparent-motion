#!/usr/bin/env python

''' Description:
Converts vibrations from accelerometers to vibrotactile actuators behaviors in real time
1. each accelerometer is associated to an actuator
2. only the Z-axis is taken into account
3. digest Z-axis values into actuator behavior
'''
# usage:
#   python demo.py 

from ast import arg
from multiprocessing.connection import wait
from urllib.parse import parse_qs
import numpy as np
import serial
import shlex
import subprocess
import sys
import time
import pathlib
import zmq


from multiprocessing import Process, Value
# The multiprocessing Queue class is a near clone of Queue.Queue (https://docs.python.org/2/library/multiprocessing.html#multiprocessing.Queue)
from multiprocessing import Queue
# Necessary to handle Empty exceptions (https://stackoverflow.com/questions/6491942/cannot-access-queue-empty-attributeerror-function-object-has-no-attribute)
from queue import Empty

import sys
sys.path.append('../../modules')
from ActuLine import ActuLine, Actuator


class States:
    is_collecting : Value('b', True)
    is_comm_ready : Value('b', False)


# Ensure the resting state (0V) of the actuators
def neutral(ad5383_path):
    subprocess.run([ad5383_path + "neutral"])


# AD3583 driver listening to the socket 5556
def start_actuators_driver(ad5383_path):
    subprocess.run([ad5383_path + "listener_AD5383"])


''' Description:
'''
if __name__ == '__main__':
    #  Socket to talk to server
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    # Get configuration from config file
    aL = ActuLine("../../cfg/config.json")
    aL.configure()
    actuators = aL.get_actuators()
    chan = aL.get_dacChannels()

    # AD5383 driver initialisation
    script_directory = str(pathlib.Path(__file__).parent.resolve())
    ad5383_path = script_directory + "/../../../AD5383_driver/build/bin/"
    neutral(ad5383_path)
    p_act_driver = Process(target=start_actuators_driver, args=(ad5383_path,))
    p_act_driver.start()

    time.sleep(1) # let some time to the AD5383 driver to set up the socket; it must be done with a duplex socket

    instruction = "dacChannels:"+ chan[0] +","+ chan[1] +","+ chan[2] +","+ chan[3] +","+ chan[4] +","+ chan[5] +"\n" 
    socket.send(instruction.encode('utf-8'))
    instruction = "frequency:" + aL.get_freqRefresh() + "\n" # in Hz
    socket.send(instruction.encode('utf-8'))

    # Open the USB port to read accelerometer values from the Teensy
    teensy_ser = serial.Serial()
    teensy_ser.port = '/dev/ttyACM0'
    teensy_ser.baudrate = 115200
    teensy_ser.open()

    delim = "," # initializing Delimiter
    delim_CR = "\n" # initializing Delimiter
    leftover = ""
    idx_z = [3, 6, 9, 12, 15, 18] # initializing Z-values index
    acc_neutral = 377
    acc_max = 1023

    modulate = np.zeros(6)
    m = np.zeros(6)
    msgs_str = [""]
    values_str = [""]
    instruction = ""
    t_prev = 0
    nb_msg = 0
    nb_while_loop = 0

    print ("[PY]I read Teensy: Start.")
    if len(sys.argv) == 2:
        end_working_sec = time.process_time()+ int(sys.argv[1]) # now+X seconds
    else:
        end_working_sec = time.process_time()+ 5 # now+X seconds
        
    start = time.process_time_ns()
    while time.process_time()<end_working_sec:

        readedByte = teensy_ser.read(teensy_ser.in_waiting)
        readedText = readedByte.decode() # From Bytes to String
        messages = msgs_str[-1] + readedText # add a potential previous truncated message
        msgs_str = messages.split(delim_CR) # Convert into a list of messages
        
        for msg in msgs_str[:-1]: # the last part will be added as leftover
            values_str = msg.split(delim) # Convert into a list of values included in the message
            # if the entire message failed to reach the Raspberry PI
            if (len(values_str)<19): continue
            
            t_diff = int(values_str[0])-t_prev # unit:microsecond
            # conserve only interesting values (Z axis) 
            m[0] = (int( values_str[idx_z[0]])-acc_neutral)/(acc_max/3) # converting valuable string into numbers
            m[1] = (int( values_str[idx_z[1]])-acc_neutral)/(acc_max/3) # converting valuable string into numbers
            m[2] = (int( values_str[idx_z[2]])-acc_neutral)/(acc_max/3) # converting valuable string into numbers
            m[3] = (int( values_str[idx_z[3]])-acc_neutral)/(acc_max/3) # converting valuable string into numbers
            m[4] = (int( values_str[idx_z[4]])-acc_neutral)/(acc_max/3) # converting valuable string into numbers
            m[5] = (int( values_str[idx_z[5]])-acc_neutral)/(acc_max/3) # converting valuable string into numbers
            m = aL.define_trajectories(m)
            instruction = str(t_diff)+","\
                        + str(m[0]) +"," \
                        + str(m[1]) +"," \
                        + str(m[2]) +"," \
                        + str(m[3]) +"," \
                        + str(m[4]) +"," \
                        + str(m[5]) +"\n"
            socket.send(instruction.encode('utf-8'))

            t_prev = int(values_str[0])
            nb_msg = nb_msg +1
        nb_while_loop = nb_while_loop+1
    end = time.process_time_ns()

    consumption_avg = int((end-start)/nb_msg)
    consumption_avg_loop = int((end-start)/nb_while_loop)
    print("[PY] AVG consumption time per message: \t\t"+ str(consumption_avg) +" ns")
    print("[PY] AVG consumption time per loop: \t\t"+ str(consumption_avg_loop) +" ns")
    print("------------------------")
    print("nb_loop="+str(nb_while_loop) +" and nb_msgs="+str(nb_msg))
    print("------------------------")
    
    for i in range(10): socket.send(b'0,2048,2048,2048,2048,2048,2048\n')
    socket.send(b'SIG_END_PROGRAM')
    p_act_driver.join()
    print ("[PY]I AD5383 ended.")
    teensy_ser.close() # Close the USB port and pipe
    print ("[PY] read Teensy: exit.")    

    print ("[PY] done.")

    