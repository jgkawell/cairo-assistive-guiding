#! /usr/bin/python3
#  Copyright 2015 Mick Walters <Mick Walters> M.L.Walters@herts.ac.uk
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
# Version 0.1 Sept 2015
# Version 1.0 Nov 2015
# Version 1.1 Jan 2016 - Fixed socket bug, contributed by Jamie Hollaway
#

# Python 2 and 3 compatibility
from __future__ import absolute_import, division, print_function
try:
      input=raw_input # Python 3 style input()
except:
      pass

import socket
from time import sleep
import atexit
import sys
import struct

hostname = "localhost"
port = 9001
class GRobot():

    def __init__(self, rname="anon", posx=1, posy=1, colour="red", rshape="None"):
        self.rname=rname
        self.posx=posx
        self.posy=posy
        self.colour=colour
        self.rshape=rshape
        self.new_robot(rname, posx, posy, colour, rshape)

    def _send(self, msg, send_type="string", rec_type="string"):
        # Send message and get respose from Simulator
        try:
            self.tcpSock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcpSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.tcpSock.connect ((hostname, port))

            # if sending a string, encode it
            if send_type == "string":
                msg = msg.encode('utf-8')

            self.tcpSock.send(msg)

            # recieve input by looping to avoid data loss
            buffer_size = 4096
            data = b''
            while True:
                packet = self.tcpSock.recv(buffer_size)
                data += packet
                if len(packet) < buffer_size and len(data) > 0:
                    break

            # NOTE: this only works in python3, python2 requires you to remove the decode statement
            if rec_type == "string":
                rmsg = data.decode("utf-8")
            else:
                rmsg = data

            self.tcpSock.close()

            if rmsg == "":
                rmsg = "Warning: Receieved Data error"

        except Exception as e:
            print(str(e))
            print("Could not send message")
            print("Please make sure Simulator is running")
            exit()

        return rmsg

    def new_robot(self, rname, posx, posy, colour, rshape):
        self._send("new_robot "+str(rname)+" "+str(posx)+" "+str(posy)+" "+colour+" "+rshape)

    def move_forward(self):
        return self._send("move_forward " + self.rname)

    def move_right(self):
        return self._send("turn_right "+ self.rname+" ")

    def move_left(self):
        return self._send("turn_left " + self.rname+" ")

    def look(self):
        msg = self._send("look " + self.rname)
        return eval(msg)

    def get_xy_pos(self, rob_name):
        return eval(self._send("get_xy_pos " + rob_name))

    def get_cur_file(self):
        return self._send("get_cur_file " + self.rname)

    def modify_cell_look(self, x, y, cell_type):
        return self._send("modify_cell_look " + self.rname + " " + str(x) + " " + str(y) + " " + cell_type)

    def get_cur_human_graph(self):
        return self._send("get_cur_human_graph " + self.rname, rec_type="byte")

    def remove_edge(self, key_a, key_b):
        return self._send("remove_edge " + self.rname + " " + str(key_a) + " " + str(key_b))

    def can_human_move(self):
        return eval(self._send("can_human_move " + self.rname))

    def can_robot_move(self):
        return eval(self._send("can_robot_move " + self.rname))

    def set_can_human_move(self, value):
        return self._send("set_can_human_move " + str(value))

    def set_can_robot_move(self, value):
        return self._send("set_can_robot_move " + str(value))



def demo():
    # print() used to show return value from method/function calls
    fred=GRobot("fred", 1, 1)
    bill=GRobot("bill", 1, 1, "green")
    print("Fred forward", fred.move_forward())
    print("Bill forward",bill.move_forward())
    print("Fred right", fred.move_right())
    print("Bill right", bill.move_right())
    count = 12
    while count > 0:
        print("Fred looks at:", fred.look())
        print("Fred forward",fred.move_forward())
        print("Bill looks at:", bill.look())
        print ("Bill forward",bill.move_forward())
        count -= 1
    print("Fred looks forward at", fred.look()[2])
    print("Bill looks forward at", bill.look()[2])

def demo2():
    arthur=GRobot("arthur", 1, 4, "blue")
    ted=GRobot("ted", 4, 4, "yellow")
    print("Arthur forward", arthur.move_forward())
    print("Ted forward",ted.move_forward())
    print("Arthur right", arthur.move_right())
    print("Ted right", ted.move_right())
    count = 12
    while count > 0:
        print("Arthur looks at: ", arthur.look())
        print("Arthur forward",arthur.move_forward())
        print("Ted looks at:", ted.look())
        print ("Ted forward",ted.move_forward())
        count -= 1
    print("Arthur looks at:", arthur.look())
    print("ted looks at:", ted.look())

if __name__ == "__main__":
    demo()
    print("Finished")
