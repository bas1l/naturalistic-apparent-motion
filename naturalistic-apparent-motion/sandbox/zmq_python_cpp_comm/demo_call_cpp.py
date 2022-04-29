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
import zmq


from multiprocessing import Process, Value
# The multiprocessing Queue class is a near clone of Queue.Queue (https://docs.python.org/2/library/multiprocessing.html#multiprocessing.Queue)
from multiprocessing import Queue
# Necessary to handle Empty exceptions (https://stackoverflow.com/questions/6491942/cannot-access-queue-empty-attributeerror-function-object-has-no-attribute)
from queue import Empty

class States:
    is_collecting : Value('b', True)
    is_comm_ready : Value('b', False)


def init(ad5383_path):
    # Ensure the resting state (0V) of the actuators
    subprocess.run([ad5383_path + "neutral"])


def start_actuators_driver(ad5383_path):
    subprocess.run([ad5383_path + "zmq_recv_timer_sandbox"])
    #subprocess.run([ad5383_path + "zerozmq_recv_sandboxing"])


''' Description:
'''
def process_acceleration(q, is_collecting, is_comm_ready):
    # Open the USB port to read accelerometer values from the Teensy
    ser=serial.Serial('/dev/ttyACM0', 9600)
    # initializing Delimiter
    delim = ","
    # initializing Z-values index
    idx = [3, 6, 9, 12, 15, 18]

    while (not is_comm_ready.value):
        continue

    print ("[PY] process_acceleration : Start:")
    cpt = 10000
    while cpt:
        readedByte = ser.readline()
        # From Byte to String
        readedText = readedByte.decode()
        # Convert into a list 
        textList = readedText.split(delim)
        # if the entire message failed to reach the Raspberry PI
        if (len(textList)<19):
            continue
        # conserve only interesting values
        textListShort = []
        for i in idx:
            textListShort.append(textList[i])

        # converting list of string into list of numbers
        intList = list(map(int, textListShort))
        
        # time and z-values
        #q.put([int(textList[0]), intList])
        q.put([cpt, intList])
        cpt = cpt-1

    # Close the USB port and pipe
    ser.close()
    # Send working to False to trigger the end of the other thread
    is_collecting.value = False
    print ("[PY] process_acceleration : done.")
    

''' Description:
'''
def send_instruction_actuators(q, is_collecting, is_comm_ready, ad5383_path):
    f = open("/tmp/test", "a")

    instruction = ""
    acc_max_val = 1023
    #acc_avg_val = [377,450,380,358,497,388]
    acc_avg_val = 377
    acc_min_val = 0

    act_max_val = 4095
    act_action_val = 2500
    act_rest_val = 2048
    
    act_delta_val = act_max_val - act_action_val
    

    #rel_acc =  (x - acc_avg_val)/acc_avg_val # between -1 and 1
    #variation_act = act_delta_val*(x - acc_avg_val)/acc_avg_val

    context = zmq.Context()

    #  Socket to talk to server
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")
    
    p_act_driver = Process(target=start_actuators_driver, args=(ad5383_path,))
    p_act_driver.start()

    is_comm_ready.value = True

    print ("[PY] send_instruction_actuators : Start:")
    while is_collecting.value:
        try:
            t, z_values = q.get(block=True, timeout=0.001) #seconds
        except Empty:
            # do nothing: check if is_working is still True
            pass
        else:
            # Handle task here and call q.task_done()
            instruction = str(t)+","+",".join(str(act_action_val + (act_delta_val*(x-acc_avg_val)/acc_avg_val)) for x in z_values)+"\n"
            #print("[PY] Sending message: " + instruction)
            #socket.send(b'0,4095,4095,4095,4095,4095,4095\n')
            socket.send(instruction.encode('utf-8'))
            #socket.send(b'Hello')
            #f.write(instruction)

    
    for i in range(10):
        socket.send(b'0,2048,2048,2048,2048,2048,2048\n')

    socket.send(b'Goodbye')
    p_act_driver.join()
    f.close()
    print ("[PY] send_instruction_actuators : done.")


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
        target=process_acceleration, 
        args=(q, is_collecting,is_comm_ready, ))
    
    # start workers
    p_act.start()
    time.sleep(2)
    p_processing.start()
    
    # wait for all workers to finish their jobs
    p_processing.join()
    p_act.join()
    