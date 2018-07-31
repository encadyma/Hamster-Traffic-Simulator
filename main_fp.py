'''
This is the basic structure of the final project
Proj name: traffic simulation
Team: Tom, JP and Kevin
Basic components:
	motion control
	UI display
	JP's separate joystick smart driving assistant //not in this program
	graph with edges class
	event watcher thread
'''

from graph_with_edge_cost import *
import time
import threading
from comm_ble import RobotComm
import Tkinter as tk
from GUI_fp import *

#class Graph ==> graph with edge cost
#class Node ==> the data structure used to represent the nodes in the graph
#class GUI ==> the class for displaying the data from event watcher
#				also in charge of controlling the whole program

'''
class GUI():
	def __init__(self, rootWin, robot_handle):
	briefly list methods of the class here for reference

class Graph:
    def __init__(self):
        self.nodes = {}
        self.startNode = None
        self.goalNode = None
        self.queue = Queue.PriorityQueue()
'''

class FSM(object):
    def __init__(self, q_handle, robot_control):

class Motion_control():
	def __init__(self, robotList):


def main():
	robot_num = 1 # max number of robots to control
	gMaxRobotNum = robot_num 
    comm = RobotComm(gMaxRobotNum)
    comm.start()

    #The comm thread is actively searching for connections
    print 'Bluetooth starts'
    robotList = comm.robotList

    #print robotList
    robot_handle = Motion_control(robotList)
    #print type(robotList)

    m = tk.Tk() #root
    gui = GUI(m, robot_handle)

    m.mainloop()

    comm.stop()
    comm.join()





if __name__ == "__main__":
	main()
