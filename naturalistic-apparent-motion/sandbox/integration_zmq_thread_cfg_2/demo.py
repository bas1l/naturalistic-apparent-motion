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
from colorama import Fore
from datetime import datetime
from multiprocessing.connection import wait
from urllib.parse import parse_qs

import math
import numpy as np
import serial
import shlex
import subprocess
import sys
import time
import pathlib
import zmq
import colorama

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


def initialise_actuators(socket, aL):
    chan = aL.get_dacChannels()
    instruction = "dacChannels:"+ chan[0] +","+ chan[1] +","+ chan[2] +","+ chan[3] +","+ chan[4] +","+ chan[5] +"\n" 
    socket.send(instruction.encode('utf-8'))
    instruction = "frequency:" + aL.get_freqRefresh() + "\n" # in Hz
    socket.send(instruction.encode('utf-8'))


def initialise_accelerometers(teensy_ser, time_to_stop):
    delim = "," # initializing Delimiter
    delim_CR = "\n" # initializing Delimiter

    mean_d = np.zeros(6)
    mean_prev = np.zeros(6)
    var = np.zeros(6)

    in_range_noise = np.zeros(6)
    max_d = np.zeros(6)
    min_d = np.full(6, 1023)
    msgs_str = [""]
    values_str = [""]
    n = 0
    x = 0
    y = 0
    z = 0

    while time.process_time()<time_to_stop:
        readedByte = teensy_ser.read(teensy_ser.in_waiting)
        readedText = readedByte.decode() # From Bytes to String
        messages = msgs_str[-1] + readedText # add a potential previous truncated message
        msgs_str = messages.split(delim_CR) # Convert into a list of messages
        
        for msg in msgs_str[:-1]: # the last part will be added as leftover
            values_str = msg.split(delim) # Convert into a list of values included in the message
            # if the entire message failed to reach the Raspberry PI
            if (len(values_str)<19): continue
            
            # average norm for the 6 accelerometers
            for i in range(6):
                x = int(values_str[i*3+1]) # converting valuable string into numbers
                y = int(values_str[i*3+2]) # converting valuable string into numbers
                z = int(values_str[i*3+3]) # converting valuable string into numbers
                d = [(a)**2 for a in (x, y, z)]
                d = math.sqrt(sum(d))
                
                mean_prev[i] = mean_d[i]

                n = n+1
                mean_d[i] = mean_d[i] + (d-mean_d[i])/n
                var[i] = ((n-1)*var[i]+(d-mean_d[i])*(d-mean_prev[i])) /n

                if max_d[i]<d:
                    max_d[i] = d
                if min_d[i]>d:
                    min_d[i] = d
    
    in_range_noise = np.maximum(abs(max_d-mean_d), abs(min_d-mean_d))/3
    print(mean_d)
    print(in_range_noise)
    
    #return mean, in_range_noise
    return mean_d, var


def interpret(teensy_ser, socket, end_working_sec, aL, z_mean, in_range_noise, rec_agreement):
    delim = "," # initializing Delimiter
    delim_CR = "\n" # initializing Delimiter
    acc_max = 1023

    m = np.zeros(6)
    msgs_str = [""]
    values_str = [""]

    idx_z = [3, 6, 9, 12, 15, 18] # initializing Z-values index
    z_val = 0
    
    instruction = ""
    nb_msg = 0
    nb_while_loop = 0

    # if participant ok with saving accelerations data
    if rec_agreement:
        now = datetime.now()
        fname_base = "recordings/"+ now.strftime("%Y-%m-%d_%H-%M-%S")

        fname_acc = fname_base + "_accelerometers-data.csv"
        rec_acc = open(fname_acc, 'w+')
        rec_acc.write("period(microsec),acc1.x,acc1.y,acc1.z,acc2.x,acc2.y,acc2.z,acc3.x,acc3.y,acc3.z,acc4.x,acc4.y,acc4.z,acc5.x,acc5.y,acc5.z,acc6.x,acc6.y,acc6.z,null,null,null,null,null,null\n")
        
        fname_act = fname_base + "_actuators-data.csv"
        rec_act = open(fname_act, 'w+')
        rec_act.write("period(microsec),act1,act2,act3,act4,act5,act6\n")

    # flush the Serial for realtime demo:
    teensy_ser.flush()

    start = time.process_time_ns()
    while time.process_time()<end_working_sec:

        readedByte = teensy_ser.read(teensy_ser.in_waiting)
        readedText = readedByte.decode() # From Bytes to String
        messages = msgs_str[-1] + readedText # add a potential previous truncated message
        msgs_str = messages.split(delim_CR) # Convert into a list of messages
        
        for msg in msgs_str[:-1]: # the last part will be added as leftover
            values_str = msg.split(delim) # Convert into a list of values included in the message
            # if the entire message failed to reach the Raspberry PI
            if (len(values_str)<19) or values_str[0] == '': 
                continue
            t = int(values_str[0]) # unit:microsecond

            # average norm for the 6 accelerometers
            for i in range(6):
                x = int(values_str[i*3+1]) # converting valuable string into numbers
                y = int(values_str[i*3+2]) # converting valuable string into numbers
                z = int(values_str[i*3+3]) # converting valuable string into numbers
                dist = [(a)**2 for a in (x, y, z)]
                dist = math.sqrt(sum(dist))
                
                # check if it is over the accelerometer noise range
                if dist>in_range_noise[i]:
                    m[i] = 8*dist/acc_max # -1 to +1 value (-100% to 100%)
                else: # ignore
                    m[i] = 0
            m = aL.define_trajectories(m)
            
            instruction = str(t)+","\
                        + str(m[0]) +"," \
                        + str(m[1]) +"," \
                        + str(m[2]) +"," \
                        + str(m[3]) +"," \
                        + str(m[4]) +"," \
                        + str(m[5]) +"\n"
            socket.send(instruction.encode('utf-8'))
            
            if rec_agreement:
                rec_acc.write(msg+"\n")
                rec_act.write(instruction)
            nb_msg = nb_msg +1

        nb_while_loop = nb_while_loop+1
    end = time.process_time_ns()

    if rec_agreement:
        rec_acc.close()

    if nb_msg == 0 :
        consumption_avg = -1
    else:
        consumption_avg = int((end-start)/nb_msg)
    consumption_avg_loop = int((end-start)/nb_while_loop)
    print("Main: AVG consumption time per message: \t"+ str(consumption_avg) +" ns")
    print("Main: AVG consumption time per loop: \t\t "+ str(consumption_avg_loop) +" ns")
    print("------------------------")
    print("Main: nb_loop="+str(nb_while_loop) +" and nb_msgs="+str(nb_msg))
    print("------------------------")
    
    for i in range(10): 
        socket.send(b'0,2048,2048,2048,2048,2048,2048\n')
    socket.send(b'SIG_END_PROGRAM')




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

    # AD5383 driver initialisation
    print ("Main: Start the communication with the actuators.")
    script_directory = str(pathlib.Path(__file__).parent.resolve())
    ad5383_path = script_directory + "/../../../AD5383_driver/build/bin/"
    neutral(ad5383_path)
    p_act_driver = Process(target=start_actuators_driver, args=(ad5383_path,))
    p_act_driver.start()

    time.sleep(1) # let some time to the AD5383 driver to set up the socket; it must be done with a duplex socket
    initialise_actuators(socket, aL)

    # Open the USB port to read accelerometer values from the Teensy
    teensy_ser = serial.Serial()
    teensy_ser.port = '/dev/ttyACM0'
    teensy_ser.baudrate = 6000000
    teensy_ser.open()

    print ("Main: Initialising the accelerometers...", end='')   
    z_mean, in_noise_range = initialise_accelerometers(teensy_ser, time.process_time()+2)
    print ("done.")

    print ("Actuators and Accelerometers Initialised.", end="\n\n")

    # check if participant is ok with recording the data
    print("Is recording accelerometer's data OK? [Y/n] ", end='')
    answer = input()
    if any(answer.lower() == f for f in ['no', 'n', '0']):
        save_agreement = False
        print(Fore.RED + "Recording disabled.")
    else:
        save_agreement = True
        print(Fore.GREEN + "Recording enabled.")
    print(Fore.WHITE)

    input ("---[Press Start to begin the demo]---")
    print ("Main: Start reading the accelerometers...")   
    if len(sys.argv) == 2: 
        end_working_sec = time.process_time()+ int(sys.argv[1]) # now+X seconds
    else: 
        end_working_sec = time.process_time()+ 5 # now+X seconds
    interpret(teensy_ser, socket, end_working_sec, aL, z_mean, in_noise_range, save_agreement)

    p_act_driver.join()
    print ("Main: Close the communication with the actuators.")
    teensy_ser.close() # Close the USB port and pipe
    print ("Main: Close the communication with the Teensy/accelerometers.")    

    print ("Main: ends.")

    