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
from grobot import GRobot
import pickle

class DemoAgent():

    def __init__(self):
        # Initialise globals
        self.robot = GRobot("demoAgent", colour="yellow")
       
        # get file name from simulator
        file_name = self.robot.getFile()

        # import world
        new_world = pickle.load(open(file_name, 'rb'))
                
        # take out the buffer walls if old map
        if len(new_world) == 33:
            self.world_size = len(new_world) - 2
            self.world = [[None] * (self.world_size) for i in range(self.world_size)]  # World map
            for i in range(self.world_size):
                for j in range(self.world_size):
                    self.world[i][j] = new_world[i+1][j+1]
        else:
            self.world_size = len(new_world)
            self.world = [[None] * (self.world_size) for i in range(self.world_size)]  # World map
            for i in range(self.world_size):
                for j in range(self.world_size):
                    self.world[i][j] = new_world[i][j]

        # Erase hazards from memory
        # TODO: We will need to modify this to remove random rewards as well
        for i in range(0, self.world_size):
            for j in range(0, self.world_size):
                if self.world[i][j] == "Hazard":
                    self.world[i][j] = None

    def run(self):   

        try:
            while True:
                char = screen.getch()
                if char == 'q' or char == 'Q':
                    break
                elif char == curses.KEY_RIGHT:
                    result = self.robot.right()
                elif char == curses.KEY_LEFT:
                    result = self.robot.left()
                elif char == curses.KEY_UP:
                    result = self.robot.forward()

                result = self.robot.look()

                # pull out values from world based on result coords
                cells = []
                for cell in result:
                    x = cell[1]
                    y = cell[2]
                    cells.append(self.cellVal(x, y))
                    
                # print out values from look and from world (note they are the same)
                screen.erase()
                # print what is actually there
                screen.addstr(0, 0, str(result))
                # print what the agent thinks should be there
                screen.addstr(1, 0, str(cells))
        finally:
            # shut down cleanly
            curses.nocbreak()
            screen.keypad(0)
            curses.echo()
            curses.endwin()
            exit()

    def cellVal(self, x, y):
        if x < 0 or x >= self.world_size:
            return ("Wall", x, y)
        elif y < 0 or y >= self.world_size:
            return ("Wall", x, y)
        else:
            cell_type = self.world[x][y]
            if cell_type != None:
                cell_type = cell_type.encode("utf-8")
            return (cell_type, x, y)


if __name__ == "__main__":
    Agent = DemoAgent()
    Agent.run()
