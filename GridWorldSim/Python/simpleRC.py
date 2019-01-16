# simpleRC.py
# Simple Remote Control Program for simulated gRobot.
# M L Walters, Version 1.0 Feb 2016

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

# You can use the demo map file "Maze.map", or create your own by
# mouse clicking on the map area to raise/lower walls.
from grobot import *

# Initialise a new GRobot

robot = NewRobot("robot", colour="yellow")
# You can call your robot any name you want!
# Normal Python variable rules apply (no spaces, chars aA-zZ, etc.)
# Colours can be: “red”, “green”, “blue”, “orange” etc.

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
finally:
    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()
    exit()
