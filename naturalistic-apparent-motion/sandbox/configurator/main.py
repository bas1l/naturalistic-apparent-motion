import sys
sys.path.append('../../modules')

from ActuLine import ActuLine #,Actuator

if __name__ == '__main__':
    al = ActuLine("../../cfg/config.json")
    al.configure()

    for a in al.get_actuators():
        a.print()
        print("\n")