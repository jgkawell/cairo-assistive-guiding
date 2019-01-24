# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass


# Set up curses for key input (from https://www.codehaven.co.uk/using-arrow-keys-with-inputs-python/)
import curses
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

# Setup imports
from grobot import *
import pickle

# Initialise globals
robot = GRobot("simpleRC", colour="yellow")
defaultWorld = "./../Maps/MazeExtra.map" # this must be the same as that used in RobotGridWorld.pyw (and any other agents operating together)
mapsize = 30

# Import map for data
world = pickle.load(open(defaultWorld, 'rb'))

try:
    while True:
        char = screen.getch()
        if char == 'q' or char == 'Q':
            break
        elif char == curses.KEY_RIGHT:
            result = robot.right()
        elif char == curses.KEY_LEFT:
            result = robot.left()
        elif char == curses.KEY_UP:
            result = robot.forward()

        result = robot.look()

        # pull out values from world based on result coords
        cells = []
        for cell in result:
            x = cell[1]
            y = cell[2]
            cells.append((world[x][y], x, y))
            
        # print out values from look and from world (note they are the same)
        screen.addstr(0, 0, str(result))
        screen.addstr(1, 0, str(cells))
finally:
    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()
    exit()
