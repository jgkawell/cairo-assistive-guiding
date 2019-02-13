#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  RoboGridWorld.py
#
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
# Version 1.1 Jan 2016
#   Fixed (Mac) socket, default map load dir. Contributed by Jamie Hollaway
#   Fixed Bug in Look() routine when robot heading east. MLW
# Version 2  March 2016
#   Fixed serious bug in Look()routine. had to change World.map file format/size
#       Will load V1 maps, but if saved from V2, will load in V1 program
#       Reccomend update to V2!

# Python 2 and 3 compatibility
from __future__ import absolute_import, division, print_function
# try:
#     input = raw_input  # Python 3 style input()
# except:
#     pass

try:
    # Python 3 tkinter
    import tkinter as tk
    import tkinter.filedialog as fd
    import tkinter.messagebox as mb

except:
    # Else Python 2 Tkinter
    import Tkinter as tk
    import tkFileDialog as fd
    import tkMessageBox as mb

# Standard imports
from threading import Thread
from time import sleep
import path_planning
import turtle as rbt
import pickle
import socket
import atexit
import sys
import struct
import copy


class GridWorldSim(tk.Tk):
    # Just one big class!
    def __init__(self, master=None):
        self.frmht = 622
        self.frmwt = 622
        self.gridspace = 20
        self.robots = {}  # Mutiple named robots?
        self.shp = []  # Robot shapes list
        self.robotStates = {}  # Internal states of robots
        self.trails = False  # Trails off to start with
        tk.Tk.__init__(self, master)
        tk.Tk.title(self, "Robot Grid World")

        # Options for visuals
        self.fogWorld = False # True=begin with greyed out world, False=show full world

        # drawing canvas in frame to include turtle graphics
        self.frame = tk.Frame(master, bg="black", borderwidth=3)
        self.frame.pack()
        self.canvas = tk.Canvas(self.frame,  height=self.frmht, width=self.frmwt, bg="white")
        self.canvas.pack()

        # Buttons under canvas area
        self.newButton = tk.Button(master, text="New World", command=lambda: self.newWorld())
        self.newButton.pack(side=tk.LEFT)
        self.loadButton = tk.Button(master, text="Load World", command=lambda: self.loadWorld())
        self.loadButton.pack(side=tk.LEFT)
        self.saveButton = tk.Button(master, text="Save World", command=lambda: self.saveWorld())
        self.saveButton.pack(side=tk.LEFT)
        self.trailButton = tk.Button(master, text="Toggle Trails", command=lambda: self.toggleTrails())
        self.trailButton.pack(side=tk.RIGHT)
        self.speedSlider = tk.Scale(master, from_=1, to=10, orient=tk.HORIZONTAL, command=self.simSpeed)
        self.speedSlider.set(5)
        self.speedSlider.pack(side=tk.RIGHT)
        self.speedLabel = tk.Label(text="Speed")
        self.speedLabel.pack(side=tk.RIGHT)

        # Add dummy turtle as hidden to set up drawing area
        # changes canvas coords! (0,0) now in middle
        self.robot1 = rbt.RawTurtle(self.canvas)
        self.robot1.hideturtle()

        # Handler for mouse clicks
        self.screen = self.robot1.getscreen()
        self.screen.onclick(self.clickGrid, btn=1)  # Mouse left button

        # Initialize default world
        self.defaultWorld = "./Maps/MazeExtra.map"
        self.cur_file = self.defaultWorld # Save the current file for other agents to pull from
        self.defaultWorldSize = 31
        self.explored = [[False] * (self.defaultWorldSize) for i in range(self.defaultWorldSize)]  # unexplored map
        self.openWorld(self.defaultWorld)
        self.human_graph = path_planning.Graph()
        self.real_graph = path_planning.Graph()
        self.real_graph.setup_graph(self.world, self.world_size)
        self.removed_edges = {}

        # Start server for robot programs to connect
        self.tcpTrd = Thread(target=self.tcpServer)
        self.tcpTrd.daemon = True
        self.tcpTrd.start()

        # Start timer for simulation speed
        self.timerTrd = Thread(target=self.simtimer)
        self.timerTrd.daemon = True
        self.timerTrd.start()

    def simtimer(self):
        while True:
            self.wait = True
            sleep(0.3-self.delay/50)
            self.wait = False
            sleep(0.05)
            # Stops window freezing when not in focus
            self.update()
            self.update_idletasks()

    def simSpeed(self, event):
        self.delay = self.speedSlider.get()

    def toggleTrails(self):
        # Work in progress!
        for rob_name in self.robots:
            if self.trails == True:
                print("OFF")
                self.robots[rob_name].penup()
                self.robots[rob_name].clear()
            else:
                print("ON")
                self.robots[rob_name].pendown()
        if self.trails == True:
            self.trails = False
        else:
            self.trails = True

    def drawWorld(self):
        # Draws the grid and labels
        # XYaxis lines, labels

        # Vertical lines
        self.canvas.delete("all")
        count = 0
        for i in range(-self.frmht//2, self.frmht//2-1, self.gridspace):
            self.canvas.create_line(
                i, self.frmht//2, i, -self.frmwt//2, dash=(2, 4))
            self.canvas.create_text(
                i+10, self.frmht//2-10, text=str(count), font=("courier", 6), fill="red")
            count += 1

        # Horizontal lines
        count = self.frmht//self.gridspace
        for i in range(-self.frmwt//2, self.frmwt//2-1, self.gridspace):
            self.canvas.create_line(-self.frmwt//2, i,
                                    self.frmht//2, i, dash=(2, 4))
            self.canvas.create_text(-self.frmwt//2+10, i+12,
                                    text=str(int(count-1)), font=("courier", 6), fill="red")
            count -= 1

        # Draw filled grids squares
        for i in range(0, self.world_size):
            for j in range(0, self.world_size):
                if self.fogWorld:
                    self.fillGrid(i, j, "Fog")
                else:
                    self.fillGrid(i, j, self.world[i][j])

    def clickGrid(self, mousex, mousey):
        x = self.maptoX(mousex)
        y = self.maptoY(mousey)
        cell_type = self.world[x][y]

        # cycle through different cell values
        if cell_type == None:
            # Make Fog
            self.fillGrid(x, y, "Fog")
            self.world[x][y] = "Fog"
        elif cell_type == "Fog":
            # Make wall
            self.fillGrid(x, y, "Wall")
            self.world[x][y] = "Wall"
        elif cell_type == "Wall":
            # Make hazard
            self.fillGrid(x, y, "Hazard")
            self.world[x][y] = "Hazard"
        elif cell_type == "Hazard":
            # Make reward
            self.fillGrid(x, y, "Reward")
            self.world[x][y] = "Reward"
        elif cell_type == "Reward":
            # Make door
            self.fillGrid(x, y, "Door")
            self.world[x][y] = "Door"
        else:
            # Make wall (etc.?)
            self.fillGrid(x, y, None)
            self.world[x][y] = None

    # change the look of a cell WITHOUT changing its contents
    def modifyCellLook(self, x, y, cell_type):
        self.fillGrid(x, y, cell_type)
        return "OK"

    # change the look of a cell AND change its contents
    def modifyCell(self, x, y, cell_type):
        self.fillGrid(x, y, cell_type)
        self.world[x][y] = cell_type

    # update the copy of the human graph from serialized object
    def updateHumanGraph(self, serializedGraph):
        sys.modules['path_planning'] = path_planning
        self.human_graph = pickle.loads(serializedGraph)
        print("Updated human graph")
        return "OK"

    def removeEdge(self, key_a, key_b):
        self.removeSingleEdge(key_a, key_b)
        self.removeSingleEdge(key_b, key_a)
        return "OK"

    def removeSingleEdge(self, from_key, to_key):
        # pull out neighbors of from vertex
        neighbors_from = self.real_graph.get_vertex(from_key).get_neighbors()

        # remove neighbor with matching key
        for key in neighbors_from.keys():
            if key == to_key:
                del neighbors_from[key]
                break

        print("Removed edges between keys: ", (from_key, to_key))


    def fillGrid(self, x, y, cell_type):
        if cell_type == None:
            self.canvas.create_line(self.xtoWorld(x)-11, self.ytoWorld(y),
                                    self.xtoWorld(x)+8, self.ytoWorld(y),
                                    fill="white", width=19)
        if cell_type == "Fog":
            self.canvas.create_line(self.xtoWorld(x)-11, self.ytoWorld(y),
                                    self.xtoWorld(x)+8, self.ytoWorld(y),
                                    fill="grey", width=19)
        elif cell_type == "Wall":
            self.canvas.create_line(self.xtoWorld(x)-11, self.ytoWorld(y),
                                    self.xtoWorld(x)+8, self.ytoWorld(y),
                                    fill="blue", width=19)
        elif cell_type == "Hazard":
            self.canvas.create_line(self.xtoWorld(x)-11, self.ytoWorld(y),
                                    self.xtoWorld(x)+8, self.ytoWorld(y),
                                    fill="red", width=19)
        elif cell_type == "Reward":
            self.canvas.create_line(self.xtoWorld(x)-11, self.ytoWorld(y),
                                    self.xtoWorld(x)+8, self.ytoWorld(y),
                                    fill="green", width=19)
        elif cell_type == "Door":
            self.canvas.create_line(self.xtoWorld(x)-11, self.ytoWorld(y),
                                    self.xtoWorld(x)+8, self.ytoWorld(y),
                                    fill="purple", width=19)

    def clearGrid(self, x, y):
        tagstr = str(x)+"u"+str(y)
        self.canvas.delete(tagstr)

    def xtoWorld(self, x=0):
        return int(-self.frmwt//2 + 12 + x * self.gridspace)

    def ytoWorld(self, y=0):
        return int(self.frmwt//2 - 12 - y * self.gridspace)

    def maptoX(self, mapx=0):
        return int((mapx + self.frmwt//2) // self.gridspace)

    def maptoY(self, mapy=0):
        return int(self.world_size - 1 - (mapy - self.frmht//2) // -self.gridspace)

    def newWorld(self):
        self.world = [[None] * (self.defaultWorldSize) for i in range(self.defaultWorldSize)]  # World map
        self.world_size = self.defaultWorldSize
        self.drawWorld()

    def saveWorld(self):
        filename = fd.asksaveasfilename(filetypes=[("World Files", "*.map")], initialdir=".")
        if filename != None:
            # remove robots from world!
            for rob_name in list(self.robots.keys()):
                xpos, ypos = self.getXYpos(rob_name)
                self.world[xpos][ypos] = None

            # Then save!
            if filename[-4:] != ".map":
                filename += ".map"

            # Protocol 2 for python 2 compatilbility
            pickle.dump(self.world, open(filename, 'wb'), 2)

    def loadWorld(self):
        filename = fd.askopenfilename(filetypes=[("World Files", "*.map")], initialdir="./Worlds/")

        if filename != "":
            if filename[-4:] != ".map":
                filename += ".map"

            self.cur_file = filename
            self.openWorld(filename)

    def openWorld(self, filename):
        self.world = pickle.load(open(filename, 'rb'))
        self.world_size = len(self.world)

        # reset unexplored map
        exploredValue = True
        if self.fogWorld:
            exploredValue = False

        self.explored = [[exploredValue] * (self.world_size) for i in range(self.world_size)]
        self.drawWorld()

    def newRobot(self, rob_name="None",  posx=1, posy=1, colour="red", rshape="None"):
        if rob_name == "None":
            # create/use Anonymous robot. Can only do one!
            rob_name = "anon"

        if not rob_name in self.robots:
            self.robots[rob_name] = rbt.RawTurtle(self.canvas)
        else:
            # Remove "old" robot from World
            self.world[self.maptoX(self.robots[rob_name].xcor())][self.maptoY(self.robots[rob_name].ycor())] = None

        # Robot shape/colour
        if rshape == "None":  # Can provide own shape def
            # Otherwise use standard robot shape
            self.shp.append(rbt.Shape("compound"))
            # print(self.shp, len(self.shp)-1)# debug
            poly1 = ((0, 0), (10, -5), (0, 10), (-10, -5))
            self.shp[len(self.shp)-1].addcomponent(poly1, colour, "black")
            poly2 = ((0, 0), (10, -5), (-10, -5))
            self.shp[len(self.shp)-1].addcomponent(poly2, "black", colour)
            self.screen.register_shape(
                rob_name+"shape", self.shp[len(self.shp)-1])
        else:
            # Can use standard shape  “arrow”, “turtle”, “circle”,
            # “square”, “triangle”, “classic”
            self.robots[rob_name].shape(rshape)

        # Initalise robot
        self.robotStates[rob_name] = 0
        self.robots[rob_name].hideturtle()
        self.robots[rob_name].pencolor(colour)
        self.robots[rob_name].clear()
        self.robots[rob_name].penup()
        self.robots[rob_name].shape(rob_name+"shape")
        self.robots[rob_name].speed(0)
        self.robots[rob_name].goto(self.xtoWorld(posx)-2, self.ytoWorld(self.world_size-posy-1)+1)
        self.robots[rob_name].setheading(90)
        self.robots[rob_name].showturtle()

        if self.trails == True:
            self.robots[rob_name].clear()
            self.robots[rob_name].pendown()
        else:
            self.robots[rob_name].penup()
            self.robots[rob_name].clear()

        self.robots[rob_name].speed(2)
        self.world[posx][posy] = rob_name

        return "OK"

    def getXYpos(self, rob_name):
        posx = self.maptoX(self.robots[rob_name].xcor())
        posy = self.maptoY(self.robots[rob_name].ycor())
        return (posx, posy)

    def moveForward(self, rob_name):
        #  check to see if forward is clear
        if self.look(rob_name)[2][0] != "Wall":  # Clear to move
            # move to next grid square
            self.robots[rob_name].forward(20)
            posx = self.maptoX(self.robots[rob_name].xcor())
            posy = self.maptoY(self.robots[rob_name].ycor())
            print((posx, posy))

            return "OK"
        else:
            # If not clear (None), then don't move
            print("Cannot move forward")
            return "OK"

    def turnLeft(self, rob_name):
        if rob_name in self.robots:
            self.robots[rob_name].left(90)
            self.look(rob_name)
            return "OK"

        return "Robot name not found"

    def turnRight(self, rob_name):
        if rob_name in self.robots:
            self.robots[rob_name].right(90)
            self.look(rob_name)
            return "OK"

        return "Robot name not found"

    def look(self, rob_name):
        if rob_name in self.robots:
            posx = self.maptoX(self.robots[rob_name].xcor())
            posy = self.maptoY(self.robots[rob_name].ycor())
            heading = int(self.robots[rob_name].heading())

            if heading == 0 and posx < self.world_size:  # East
                val = [self.getValue(posx, posy+1), self.getValue(posx+1, posy+1),
                        self.getValue(posx+1, posy), self.getValue(posx+1, posy-1), self.getValue(posx, posy-1)]
            elif heading == 90 and posy < self.world_size:  # North
                val = [self.getValue(posx-1, posy), self.getValue(posx-1, posy+1),
                        self.getValue(posx, posy+1), self.getValue(posx+1, posy+1), self.getValue(posx+1, posy)]
            elif heading == 180 and posx >= 0:  # West
                val = [self.getValue(posx, posy-1), self.getValue(posx-1, posy-1),
                        self.getValue(posx-1, posy), self.getValue(posx-1, posy+1), self.getValue(posx, posy+1)]
            elif heading == 270 and posy >= 0:  # South
                val = [self.getValue(posx+1, posy), self.getValue(posx+1, posy-1),
                        self.getValue(posx, posy-1), self.getValue(posx-1, posy-1), self.getValue(posx-1, posy)]
            else:
                # Facing edge of world
                val == [("Wall", 0, 0), ("Wall", 0, 0), ("Wall", 0, 0), ("Wall", 0, 0), ("Wall", 0, 0)]


            for block in val:
                px, py = block[1], block[2]
                if self.explored[px][py] == False:
                    self.explored[px][py] = True
                    self.fillGrid(px, py, self.world[px][py])

            if(len(self.removed_edges) > 0):
                for key_a, key_b in self.removed_edges.items():
                        val.append((int(key_a), int(key_b)))
                self.removed_edges.clear()

            return val

        return "Robot name not found"

    def getValue(self, x, y):
        if x < 0 or x >= self.world_size:
            return ("Wall", x, y)
        elif y < 0 or y >= self.world_size:
            return ("Wall", x, y)
        else:
            return (self.world[x][y], x, y)

    def tcpServer(self):
        """
        TCIP server, opens a TC IP socket and passes the message on to be executed and
        waits for input from the TCIP socket and pases it on to dispatch() for
        evaluation. If "q" input, ends connection, "Q" input ends server.
        """
        # variables
        tcpSock = None
        tcpOk = 0
        try:
            # Create IP socket and wait for customers
            tcpSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except:
            print("Error creating socket")
        print("Please wait: Binding address to socket")

        # Bug fix for Mac - C ontributed by Jamie Hollaway
        tcpSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        msgtext = tk.Label(
            self.frame, text="Please Wait: Setting up Connection", bg="red")
        msgtext.pack(side=tk.TOP)
        while tcpOk == 0:
            try:
                tcpSock.bind(("localhost", 9001))
                tcpSock.listen(3)
                tcpOk = 1
            except:
                sleep(1.0)  # Keep trying!
        print("Socket ready now")
        msgtext.destroy()

        # make sure socket closes at eop
        atexit.register(tcpSock.close)
        atexit.register(tcpSock.shutdown, 1)
        while tcpOk == 1:
            # when customer calls, service requests
            cli_sock, cli_ipAdd = tcpSock.accept()
            try:
                # python 3
                thrd = Thread(target=self.dispatch, args=(cli_sock,))
                thrd.daemon = True
                thrd.start()
            except:
                # raise # debug
                print("Warning TCP/IP Error")  # Just keep on with next request

        # Clean up if this point ever reached
        tcpSock.shutdown(1)
        tcpSock.close()
        print("Server closed")

    def dispatch(self, cli_sock):
        # message variables
        msg = "$"
        rmsg = None

        # recieve input by looping to avoid data loss
        buffer_size = 4096
        data = b''
        while True:
            packet = cli_sock.recv(buffer_size)
            data += packet
            if len(packet) < buffer_size:
                break

        message = data

        # attempt to parse string
        try:
            msg = message.decode("utf-8")
        except:
            print("Not string")

        if msg != "Q":
            # split string msg into parts
            msg = msg.split()

            # Do robot commands
            try:
                if msg[0] == "N":
                    # New or init robot
                    rmsg = self.newRobot(msg[1], int(msg[2]), int(msg[3]), msg[4], msg[5])
                elif msg[0] == "F":
                    # msg[1] is robot name
                    rmsg = self.moveForward(msg[1])
                elif msg[0] == "R":
                    rmsg = self.turnRight(msg[1])
                elif msg[0] == "L":
                    rmsg = self.turnLeft(msg[1])
                elif msg[0] == "S":
                    rmsg = str(self.look(msg[1]))
                elif msg[0] == "P":
                    rmsg = self.getXYpos(msg[1])
                elif msg[0] == "G":
                    rmsg = self.cur_file
                elif msg[0] == "M":
                    rmsg = self.modifyCellLook(x=int(msg[2]), y=int(msg[3]), cell_type=msg[4])
                elif msg[0] == "A":
                    rmsg = pickle.dumps(copy.deepcopy(self.human_graph), protocol=2)
                elif msg[0] == "E":
                    key_a = msg[2]
                    key_b = msg[3]
                    rmsg = self.removeEdge(key_a, key_b)
                    self.removed_edges[key_a] = key_b
                else:
                    rmsg = self.updateHumanGraph(message)
            except Exception as e:
                # raise #debug. If error just carry on
                rmsg = "Server Error"
                print("EXCEPTION: " + str(e))
                print(msg)

            if rmsg == None:
                rmsg == "None"

            # Wait here for step timer
            while self.wait == True:
                sleep(0.01)

            if type(rmsg)==str:
                cli_sock.send(str(rmsg).encode('utf-8'))
            else:
                cli_sock.send(rmsg)

        cli_sock.close()
        return


if __name__ == '__main__':
    GRSApp = GridWorldSim()
    GRSApp.mainloop()
