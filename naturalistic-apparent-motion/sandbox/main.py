from multiprocessing import Process
import shlex, subprocess

from modules.accelerometer_processing import AccelerometerProcessing

def call_ls():
    cpt=10
    while cpt:
        subprocess.run(["ls", "-l"])
        cpt=cpt-1
    
def call_neutral():
    cpt=3
    while cpt:
        subprocess.run(["/home/pi/Software/HaptiComm/build/0_executables/neutral/neutral"])
        cpt=cpt-1

if __name__ == '__main__':
    p_ls = Process(target=call_ls)
    p_neutral = Process(target=call_neutral)

    p_ls.start()
    p_neutral.start()
    
    p_ls.join()
    p_neutral.join()