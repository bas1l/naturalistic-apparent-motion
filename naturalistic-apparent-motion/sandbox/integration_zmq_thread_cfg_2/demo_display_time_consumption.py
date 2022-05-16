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
from urllib.parse import parse_qs
import numpy as np
import serial
import shlex
import subprocess
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
def init(ad5383_path):
    subprocess.run([ad5383_path + "neutral"])


# AD3583 driver listening to the socket 5556
def start_actuators_driver(ad5383_path):
    subprocess.run([ad5383_path + "zmq_recv_timer_sandbox"])


''' Description:
'''
if __name__ == '__main__':
    script_directory = str(pathlib.Path(__file__).parent.resolve())
    ad5383_path = script_directory + "/../../../AD5383_driver/build/bin/"

    init(ad5383_path)


    #  Socket to talk to server
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    p_act_driver = Process(target=start_actuators_driver, args=(ad5383_path,))
    p_act_driver.start()

    aL = ActuLine("../../cfg/config.json")
    aL.configure()
    actuators = aL.get_actuators()
    chan = aL.get_dacChannels()

    instruction = ""
    #acc_neutral = [377,450,380,358,497,388]
    acc_neutral = 377
    acc_max = 1023

    time.sleep(1) # let some time to the AD5383 driver to set up the socket; it must be done with a duplex socket

    instruction = "dacChannels:"+ chan[0] +","+ chan[1] +","+ chan[2] +","+ chan[3] +","+ chan[4] +","+ chan[5] +"\n" 
    socket.send(instruction.encode('utf-8'))
    instruction = "frequency:" + aL.get_freqRefresh() + "\n" # in Hz
    socket.send(instruction.encode('utf-8'))


    # Open the USB port to read accelerometer values from the Teensy
    #teensy_ser = serial.Serial('/dev/ttyACM0', 9600)
    teensy_ser = serial.Serial()
    teensy_ser.port = '/dev/ttyACM0'
    teensy_ser.baudrate = 115200

    delim = "," # initializing Delimiter
    delim_CR = "\n" # initializing Delimiter
    leftover = ""
    idx_z = [3, 6, 9, 12, 15, 18] # initializing Z-values index

    teensy_ser.open()
    time.sleep(3) # let some time to the AD5383 driver to set up the socket; it must be done with a duplex socket


    print ("[PY]I read Teensy: Start.")
    t_prev = 0
    nb_msg = 0
    nb_while_loop = 0
    end_working_sec = time.process_time()+30 # now+X seconds
    modulate = np.zeros(6)
    m = np.zeros(6)
    t_prev = 0
    instruction = ""
    msgs_str = [""]
    values_str = [""]

    consumption_0 = 0
    consumption_1 = 0
    consumption_2 = 0
    consumption_3 = 0

    i_max = 100000
    starttt = time.process_time_ns()
    for i in range(i_max):
        enddd = time.process_time_ns()
    print("[PY] AVG consumption time for process_time_ns(): \t\t"+ str(int(time.process_time_ns()-starttt)/i_max) +" ns")

    whew = 0
    loop_duration = time.process_time_ns()

    time.sleep(3) # let some time to the AD5383 driver to set up the socket; it must be done with a duplex socket
    start = time.process_time_ns()
    while time.process_time()<end_working_sec:

        start_time = time.process_time_ns()
        readedByte = teensy_ser.read(teensy_ser.in_waiting)
        consumption_0 = consumption_0 + (time.process_time_ns()-start_time)
        
        start_time = time.process_time_ns()
        readedText = readedByte.decode() # From Bytes to String
        messages = msgs_str[-1] + readedText # add a potential previous truncated message
        msgs_str = messages.split(delim_CR) # Convert into a list of messages
        consumption_1 = consumption_1 + (time.process_time_ns()-start_time)
        
        #instruction = ""
        for msg in msgs_str[:-1]: # the last part will be added as leftover

            start_time = time.process_time_ns()
            values_str = msg.split(delim) # Convert into a list of values included in the message
            # if the entire message failed to reach the Raspberry PI
            if (len(values_str)<19): continue
            
            t_diff = int(values_str[0])-t_prev # unit:microsecond
            # conserve only interesting values (Z axis) 
            for i in range(6):
                m[i] = (int( values_str[idx_z[i]])-acc_neutral)/(acc_max/3) # converting valuable string into numbers
            # Handle task here and call q.task_done()
            #modulate = (z_val_i-acc_neutral)/(acc_max/3) # '/' means already floating-point division
            m = aL.define_trajectories(m)
            #instruction = instruction \
            instruction = str(t_diff)+","\
                        + str(m[0]) +"," \
                        + str(m[1]) +"," \
                        + str(m[2]) +"," \
                        + str(m[3]) +"," \
                        + str(m[4]) +"," \
                        + str(m[5]) +"\n"
            t_prev = int(values_str[0])
            nb_msg = nb_msg +1
            consumption_2 = consumption_2 + (time.process_time_ns()-start_time)
            
            start_time = time.process_time_ns()
            socket.send(instruction.encode('utf-8'))
            consumption_3 = consumption_3 + (time.process_time_ns()-start_time)
        nb_while_loop = nb_while_loop+1

        if (time.process_time_ns()-loop_duration)>400000:
            whew = whew+1
            print("Whew." +str(whew)+ "\t nb_while_loop="+str(nb_while_loop) + ", nb_msg="+str(nb_msg))
        loop_duration = time.process_time_ns()

    end = time.process_time_ns()



    consumption_avg = int((end-start)/nb_msg)
    consumption_avg_loop = int((end-start)/nb_while_loop)
    consumption_avg_teensyread = int(consumption_0/nb_while_loop)
    consumption_avg_Teensyprocessing = int(consumption_1/nb_while_loop)
    consumption_avg_instructionprocessing = int(consumption_2/nb_msg)
    consumption_avg_socket = int(consumption_3/nb_msg)
    print("[PY] AVG consumption time per message: \t\t"+ str(consumption_avg) +" ns")
    print("[PY] AVG consumption time per loop: \t\t"+ str(consumption_avg_loop) +" ns")
    print("------------------------")
    print("nb_loop="+str(nb_while_loop) +" and nb_msgs="+str(nb_msg))
    print("------------------------")
    print("[PY] AVG consumption time for teensy communication:\t "+ str(consumption_avg_teensyread) +" ns")
    print("[PY] AVG consumption time for each teensy_read decode: \t  "+ str(consumption_avg_Teensyprocessing) +" ns")
    print("[PY] AVG consumption time for each instruction: \t"+ str(consumption_avg_instructionprocessing) +" ns")
    print("[PY] AVG consumption time for socket communication:\t "+ str(consumption_avg_socket) +" ns")
    print("[PY] ---->:\t "+ str(consumption_avg_teensyread+consumption_avg_Teensyprocessing+consumption_avg_instructionprocessing+consumption_avg_socket) +" ns")
    print("------------------------")
    
    for i in range(10): socket.send(b'0,2048,2048,2048,2048,2048,2048\n')
    socket.send(b'SIG_END_PROGRAM')
    p_act_driver.join()
    print ("[PY]I AD5383 ended.")
    teensy_ser.close() # Close the USB port and pipe
    print ("[PY] read Teensy: exit.")    

    print ("[PY] done.")

    