import json

with open("../../cfg/config.json") as json_data_file:
    data = json.load(json_data_file)
#print(data)

for act_incr in data["actuators"]:
    act = data["actuators"][act_incr]
    print(act["id"])
    print(act["windingDirection"])
    print(act["dacChannel"])

    av = act["actionValues"]
    print(int(av["neutral"]))
    print(av["min"])
    print(av["max"])
    print(av["up"])
    