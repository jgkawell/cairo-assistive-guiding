# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass


# Set up curses for key input (from https://www.codehaven.co.uk/using-arrow-keys-with-inputs-python/)
# import curses
# screen = curses.initscr()
# curses.noecho()
# curses.cbreak()
# screen.keypad(True)

# Setup imports
from grobot import GRobot
from path_planning import *
import pickle


# Initialise globals
robot = GRobot("simpleRC", colour="yellow")
defaultWorld = "./../Maps/MazeExtra.map" # this must be the same as that used in RobotGridWorld.pyw (and any other agents operating together)
mapsize = 31

# Import map for data
world = pickle.load(open(defaultWorld, 'rb'))

# Erase hazards from memory
# TODO: We will need to modify this to remove random rewards as well
for i in range(0, mapsize):
    for j in range(0, mapsize):
        if world[i][j] == "Hazard":
            world[i][j] = None

#path plan with a*
G = Graph(mapsize, world)
start = G.get_vertex(31*2 + 2)
goal = G.get_vertex(31*16 + 2) #goal pos for MazeExtra.map
t = a_star(G, start, goal)
print(reversed(t))

heading = 90 #0=forward, 90 = right, 180 = down, 270 = left
for coord in reversed(t):
    (i, j) = coord
    (x, y) = (i-1, j-1)
    direction = (x - robot.posx, y - robot.posy)
    print((x,y), (robot.posx, robot.posy))
    if direction == (1, 0): #right
        if heading == 0:
            robot.forward()
        else:
            for i in range(int(heading / 90)): robot.right()
            robot.forward()
            heading = 0
        robot.posx += 1

    elif direction == (0, 1): #up
        if heading == 90:
            robot.forward()
        elif heading > 90:
            for i in range(int((heading - 90)/90)): robot.right()
            robot.forward()
            heading = 90
        else: #facing right
            robot.left()
            robot.forward()
            heading = 90
        robot.posy += 1

    elif direction == (-1, 0): #left
        if heading == 180:
            robot.forward()
        elif heading < 180:
            for i in range(int((180-heading)/90)): robot.left()
            robot.forward()
            heading = 180
        else: # facing down = 270
            robot.right()
            robot.forward()
            heading = 180
        robot.posx -= 1

    elif direction == (0, -1): #down
        if heading == 270:
            robot.forward()
        else:
            for i in range(int((270-heading)/90)): robot.left()
            robot.forward()
            heading = 270
        robot.posy -= 1

    else:
        pass
