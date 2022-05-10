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
def read_acceleration(q, is_collecting, is_comm_ready):
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
    while (not is_comm_ready.value): continue

    time.sleep(1)

    print ("[PY]I read Teensy: Start.")
    start = time.process_time_ns()
    cpt_max = 5000
    for cpt in range(cpt_max): # for the sake of poc
        #readedByte = b'0,1,1,1,2,2,2,3,3,3,4,4,4,5,5,5,6,6,6'
        #readedByte = teensy_ser.read(teensy_ser.in_waiting)
        readedByte = teensy_ser.readline()
        readedText = readedByte.decode() # From Bytes to String

        #message = leftover + readedText 

        teensyVal_str = readedText.split(delim) # Convert into a list 

        #print(readedText)

        #print(teensy_ser.in_waiting)
        #
        # if the entire message failed to reach the Raspberry PI
        if (len(teensyVal_str)<19): continue
        # conserve only interesting values
        z_val_i = []
        for i in idx_z:
            z_val_i.append(int(teensyVal_str[i])) # converting valuable string into numbers
        
        msg = [int(teensyVal_str[0]), z_val_i]
        q.put(msg) # push time and z-values to the other thread
        #q.put([0, [1,2,3,4,5,6]])
    print("[PY]I average time for each loop: "+ str(int((time.process_time_ns()-start)/cpt_max)) +"ns")
    
    teensy_ser.close() # Close the USB port and pipe
    is_collecting.value = False # Send working to False to trigger the end of the other thread
    print ("[PY]I read Teensy: done.")
    

''' Description:
'''
def send_instruction_actuators(q, is_collecting, is_comm_ready, ad5383_path):
    #  Socket to talk to server
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")
    
    p_act_driver = Process(target=start_actuators_driver, args=(ad5383_path,))
    p_act_driver.start()

    f = open("/tmp/test", "a")

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
    
    is_comm_ready.value = True

    print ("[PY]II compose actuators: Start.")
    t_prev, _ = q.get() # flush the first values to get the Teensy clock diff
    t_diff = t_prev-0
    instruction = "0,1,2,3,4,5,6\n"
    empt = 0
    cpt = 0
    start = time.process_time_ns()
    while is_collecting.value:
        #q.get(timeout=1)
        #print(".", end = '')
        try:
            #z_values = [1,2,3,4,5,6]
            #t = 0
            t, z_values = q.get(block=True, timeout=0.0004) #seconds (or 1ms)
        except Empty:
            # do nothing: check if is_working is still True
            empt = empt+1
            pass
        else:
            cpt = cpt+1
            t_diff = t-t_prev # unit:microsecond
            t_prev = t
            # Handle task here and call q.task_done()
            modulate = (np.array(z_values)-acc_neutral)/(acc_max/3) # '/' means already floating-point division

            m = aL.define_trajectories(modulate)
            instruction = str(t_diff)+","\
                        + str(m[0]) +"," \
                        + str(m[1]) +"," \
                        + str(m[2]) +"," \
                        + str(m[3]) +"," \
                        + str(m[4]) +"," \
                        + str(m[5]) +"\n"

            #np.set_printoptions(precision=2)
            #print(modulate)
            #print(instruction)
            socket.send(instruction.encode('utf-8')) 
    print("[PY]II average time for each loop: "+ str(int((time.process_time_ns()-start)/cpt)) +"ns")
    
    
    for i in range(10):
        socket.send(b'0,2048,2048,2048,2048,2048,2048\n')

    print ("[PY]II number of <except Empty>: " + str(empt))
    socket.send(b'SIG_END_PROGRAM')
    print ("[PY]II waiting for AD5383 to end.")
    p_act_driver.join()
    print ("[PY]II AD5383 ended.")
    f.close()
    print ("[PY]II compose actuators: done.")


''' Description:
'''
if __name__ == '__main__':
    script_directory = str(pathlib.Path(__file__).parent.resolve())
    ad5383_path = script_directory + "/../../../AD5383_driver/build/bin/"

    init(ad5383_path)

    is_collecting = Value('b', True)
    is_comm_ready = Value('b', False)
    q = Queue()
    # create threads for the sensors and actuators
    p_act = Process(
        target=send_instruction_actuators, 
        args=(q, is_collecting,is_comm_ready, ad5383_path, ))
    p_processing = Process(
        target=read_acceleration, 
        args=(q, is_collecting,is_comm_ready, ))
    
    # start workers
    p_act.start()
    p_processing.start()
    
    # wait for all workers to finish their jobs
    p_processing.join()
    p_act.join()
    