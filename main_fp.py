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
#from GUI_fp import *
import Queue as que

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

#robot_go = False
#robot_quit = False

class Event(object):
    def __init__(self, event_type, event_data):
      self.type = event_type #string
      self.data = event_data #list of number or character depending on type

class FSM(object):
    def __init__(self, q_handle, robot_control):
        self.go = False
        self.quit = False

        self.startState = None
        self.states = []
        self.endStates = []

        self.q = q_handle
        self.robot_control = robot_control

    def addStartState(self, startState):
        self.startState = startState

    def add_state(self, state, event, callback, next_state):
        self.states.append([state, event, callback, next_state]) # append to list
        return
    
    def addEndState(self, addedEndState):
        self.endStates.append(addedEndState)

    def run(self):
        i = 0

        currentState = self.startState
        while not self.robot_control.quit:
            time.sleep(0.03)
            #print "Inside the FSM"
            if currentState in self.endStates:
                break
            if not self.q.empty():
                event = self.q.get()
                for c in self.states:
                    if c[0] == currentState and c[1] == event.type:
                        c[2]()
                        currentState = c[3]
                        print "state changes from " + c[0] + "to" + currentState
                        break

        self.robot_control.quit = True
        self.robot_control.go = False               

        return

class Motion_control():
	def __init__(self, robotList):
        self.robotList = robotList
        self.q = que.Queue()

        ##########The robot's control variables#########
        self.go = False
        self.quit = False

        ##########Create the thread for event_watcher#######
        t_robot_watcher = threading.Thread(name='watcher thread', target=self.robot_event_watcher, argss(self.q))
        t_robot_watcher.daemon = True
        t_robot_watcher.start()

        #########Create the thread for FSM engine######
        self.sm = FSM(self.q, self)
        self.setMachine()

        sm_thread = threading.Thread(name = "FSM", target = self.sm.run)
        sm_thread.setDaemon(True)
        sm_thread.start()

    def setMachine(self):
        self.sm.add_state("TurningLeft", "obsLeft", self.turning_left, "TurningLeft")

    def event_watcher(self):
        q = self.q

        


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
