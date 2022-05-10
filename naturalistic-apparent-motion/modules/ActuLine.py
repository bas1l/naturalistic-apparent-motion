import json
import numpy as np

class ActuLine:
    config_file = ""
    nb_actuators = -1
    freqRefresh = -1
    def __init__(self, _config_file="../cfg/config.json", nb_actuators="6", freqRefresh="2000"):
        self.config_file = _config_file
        self.nb_actuators = int(nb_actuators)
        self.freqRefresh = freqRefresh
        self.amplitudes = np.arange(self.nb_actuators)
        self.offsets = np.arange(self.nb_actuators)

        self.actuators = [Actuator() for i in range(self.nb_actuators)]
    
    



    def configure(self):
        with open(self.config_file) as json_data_file:
            data = json.load(json_data_file)
        
        self.freqRefresh = data["actuline_overview"]["freqRefresh"]

        a = 0
        for act_id in data["actuators"]:
            act = data["actuators"][act_id]
            av = act["actionValues"]

            id = act["id"]
            wd = act["windingDirection"]
            dc = act["dacChannel"]
            aneutral = int(av["neutral"])
            amin = int(av["min"])
            amax = int(av["max"])
            aup = int(av["up"])

            self.actuators[a].configure(id, wd, dc, aneutral, amin, amax, aup)
            self.actuators[a].init_traj()

            self.amplitudes[a] = self.actuators[a].get_amplitude()
            self.offsets[a] = self.actuators[a].get_offset()

            a = a+1

        

    def define_trajectories(self, modulations):
        return self.amplitudes*modulations +self.offsets
         
    def get_dacChannels(self):
        chan = []
        for a in self.actuators:
            chan.append(a.get_dacChannel())
        return chan
    def get_actuators(self):
        return self.actuators

    def get_freqRefresh(self):
        return self.freqRefresh



class Actuator:
    # properties
    id = "default"
    windingDirection = "default"
    dacChannel = -1
    action_neutral = -1
    action_min = -1
    action_max = -1
    action_up = -1
    slope = -1
    intercept = -1



    def __init__(self, _id="default", _wd="default", _dc="-1", 
                       _aneutral="-1", _amin="-1", _amax="-1", _aup="-1"):
        self.configure(_id, _wd, _dc, _aneutral, _amin, _amax, _aup)

    def configure(self, _id, _wd, _dc, _aneutral, _amin, _amax, _aup):
        self.set_id(_id)
        self.set_windingDirection(_wd)
        self.set_dacChannel(_dc)
        self.set_action_neutral(_aneutral)
        self.set_action_min(_amin)
        self.set_action_max(_amax)
        self.set_action_up(_aup)

    def init_traj(self):
        # define constant for 
        if self.windingDirection == 'anticlockwise':
            self.amplitude = -1 * (self.action_max - self.action_up)
            self.offset = 2*self.action_neutral - self.action_up # 4095-2700=1300
        else: # clockwise
            self.amplitude = 1 * (self.action_max - self.action_up)
            self.offset = self.action_up


    # modulation = 0 -> 1 (float)
    def define_trajectory(self, modulation):
        return int(self.amplitude*modulation + self.offset)

    #   SETTERS 
    def set_id(self, _id):
        self.id = _id

    def set_windingDirection(self, _windingDirection):
        self.windingDirection = _windingDirection

    def set_dacChannel(self, _dacChannel):
        self.dacChannel = _dacChannel

    def set_action_neutral(self, _action_neutral):
        self.action_neutral = _action_neutral

    def set_action_min(self, _action_min):
        self.action_min = _action_min

    def set_action_max(self, _action_max):
        self.action_max = _action_max

    def set_action_up(self, _action_up):
        self.action_up = _action_up


    #   GETTERS
    def get_id(self):
        return self.id

    def get_windingDirection(self):
        return self.windingDirection 

    def get_dacChannel(self):
        return self.dacChannel

    def get_action_neutral(self):
        return self.action_neutral

    def get_action_min(self):
        return self.action_min

    def get_action_max(self):
        return self.action_max

    def get_action_up(self):
        return self.action_up
        
    def get_amplitude(self):
        return self.amplitude
        
    def get_offset(self):
        return self.offset


    def print(self):
        print("actuator information ---")
        print("id: " + self.get_id())
        print("windingDirection: " + self.get_windingDirection())
        print("dacChannel: " + self.get_dacChannel())
        print("action_neutral: " + str(self.get_action_neutral()))
        print("action_min: " + str(self.get_action_min()))
        print("action_max: " + str(self.get_action_max()))
        print("action_up: " + str(self.get_action_up()))