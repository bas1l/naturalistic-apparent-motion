#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import zmq
import time

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

time.sleep(3)

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending message n%s …" % request)
    socket.send(b"Hello")
