'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          Robot Escape
   By:            Qin Chen
   Last Updated:  6/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
# This program shows how threads can be created using Thread class and your
# own functions. Another way of creating threads is subclass Thread and override
# run().
# 

import sys
#sys.path.append('../')
import time  # sleep
import threading
import Tkinter as tk
import Queue
from comm_ble import RobotComm

#logging.basicConfig(level=logging.DEBUG,format='(%(threadName)-10s) %(message)s',)

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



class BehaviorThreads(object):
    Threshold_border = 20   # if floor sensor reading falls equal or below this value, border is detected
    Threshold_obstacle = 50   # if prox sensor reading is equal or higher than this, obstacle is detected
    
    def __init__(self, robot_list):
    	self.robot_list = robot_list
        self.go = False
        self.quit = False
        # events queues for communication between threads
        self.alert_q = Queue.Queue()
        self.motion_q = Queue.Queue()
        self.t_robot_watcher = None     # thread handles
        self.t_motion_handler = None
        
        # start a watcher thread
        #The t_robot_watcher is an instance variable?
        t_robot_watcher = threading.Thread(name='watcher thread', target=self.robot_event_watcher, args=(self.alert_q, self.motion_q))
        t_robot_watcher.daemon = True
        t_robot_watcher.start()
        #why do the following step?
        self.t_robot_watcher = t_robot_watcher

        self.sm = FSM(self.motion_q, self)
        self.setMachine()

        sm_thread = threading.Thread(name = "FSM", target = self.sm.run)
        sm_thread.setDaemon(True)
        sm_thread.start()

        return

    ###################################
    # This function is called when border is detected
    ###################################
    #def get_out (self, robot):
    #    pass
    def setMachine(self):
        self.sm.add_state("TurningLeft", "obsLeft", self.turning_left, "TurningLeft")
        self.sm.add_state("TurningLeft", "free", self.moving_forward, "GoingForward")
        self.sm.add_state("TurningRight", "obsRight", self.turning_right, "TurningRight")
        self.sm.add_state("TurningRight", "free", self.moving_forward, "GoingForward")
        self.sm.add_state("GoingForward", "obsLeft", self.turning_right, "TurningRight")
        self.sm.add_state("GoingForward", "obsRight", self.turning_left, "TurningLeft")
        self.sm.add_state("GoingForward", "free", self.moving_forward, "GoingForward")
        self.sm.add_state("TurningLeft", "border", self.stop_move, "AtBorder")
        self.sm.add_state("GoingForward", "border", self.stop_move, "AtBorder")
        self.sm.add_state("TurningRight", "border", self.stop_move, "AtBorder")
        self.sm.add_state("AtBorder", "free", self.stop_move, "AtBorder")

        self.sm.addStartState("GoingForward")
        self.sm.addEndState("AtBorder")

    # This function monitors the sensors
    def robot_event_watcher(self, q1, q2):
        count = 0

        #logging.debug('starting...')
        while not self.quit: #the GUI's stopProg turns the self.quit into True, so this daemon thread ends
            for robot in self.robot_list:
                if self.go and robot:
                    #print('inside event watcher loop')
                    prox_l = robot.get_proximity(0)
                    prox_r = robot.get_proximity(1)
                    line_l = robot.get_floor(0)
                    line_r = robot.get_floor(1)

                    if (prox_l > BehaviorThreads.Threshold_obstacle or prox_r > BehaviorThreads.Threshold_obstacle):
                        alert_event = Event("alert", [prox_l,prox_r])
                        q1.put(alert_event)
                        count += 1
	                    #update movement every 5 ticks, to avoid being too sensitive?
                        if (count % 5 == 0):
                            if prox_l > prox_r:
                                obs_event = Event("obsLeft", [prox_l, prox_r])
                                print "obs_left"
                                q2.put(obs_event)
                            else:
                                obs_event = Event("obsRight", [prox_l, prox_r])
                                print "obs_right"
                                q2.put(obs_event)
                    else:
                        if (count > 0):
                        	# free event is created when robot goes from obstacle to no obstacle
                            #logging.debug("free of obstacle")
                            free_event = Event("free",[])
                            q2.put(free_event)  # put event in motion queue
                            q1.put(free_event)  # put event in alert queue
                            count = 0
                    if (line_l < BehaviorThreads.Threshold_border or line_r < BehaviorThreads.Threshold_border):
                        border_event = Event("border", [line_l, line_r])
                        #print 
                        print "border detected"
                        q1.put(border_event)
                        q2.put(border_event)
                    
                else:
                    #print 'waiting ...'
                    pass

            time.sleep(0.06)	# delay to give alert thread more processing time. Otherwise, it doesn't seem to have a chance to serve 'free' event
        return

    ##############################################################
    # Implement your motion handler. You need to get event using the passed-in queue handle and
    # decide what Hamster should do. Hamster needs to avoid obstacle while escaping. Hamster
    # stops moving after getting out of the border and remember to flush the motion queue after getting out.
    #############################################################
    def turning_left(self):
        print "turing left"
        if self.robot_list and self.go:
            print "TURNING LEFT"
            for robot in self.robot_list:
                robot.set_wheel(0, -20)
                print "TURNING RIGHT"
                robot.set_wheel(1, 30)
    def turning_right(self):
        print "turning right"
        if self.robot_list and self.go:
            print "TURNING RIGHT"
            for robot in self.robot_list:
                robot.set_wheel(0, 30)
                print "TURNING RIGHT"
                robot.set_wheel(1, -20)
    def moving_forward(self):
        if self.robot_list and self.go:
            print "MOVING FORWARD"
            for robot in self.robot_list:
                robot.set_wheel(0, 30)
                print "MOVING FORWARD"
                robot.set_wheel(1, 30)
    def stop_move(self):
        if self.robot_list and self.go:
            for robot in self.robot_list:
                robot.set_wheel(0, 0)
                robot.set_wheel(1, 0)
                                       


class GUI(object):
    def __init__(self, root, threads_handle):  #self.t_handle = threads_handle
        self.root = root
        self.t_handle = threads_handle
        self.event_q = threads_handle.alert_q
        self.t_alert_handler = None
        self.canvas = None
        self.prox_l_id = None
        self.prox_r_id = None
        self.initUI()

    ##########################################################
    # 1. Create a canvas widget and three canvas items: a square, and two lines 
    # representing prox sensor readings.
    # 2. Create two button widgets, for start and exit.
    # 3. Create a thread for alert handler, which is responsible for displaying prox sensors.
    ##########################################################
    def initUI(self):
        self.root.geometry("400x600")
        self.root.title("Hamster_escape")

        #self.t_alert_handler = 
        self.cc = tk.Canvas(self.root, bg = "green", width = 400, height = 545)
        self.cc.pack(side = "top")

        hamsterIcon = self.cc.create_rectangle(130, 260, 270, 400, fill = "blue", width = 3.6)

        self.lineLeft = self.cc.create_line(170, 260, 170, 260, width = 3, fill = "red")
        self.lineRight = self.cc.create_line(230, 260, 230, 260, width = 3, fill = "red")

        b1 = tk.Button(self.root, text = "Start")
        b1.bind("<Button-1>", self.startRobot)
        b1.pack(side = "bottom")

        bb = tk.Button(self.root, text = "Exit")
        bb.bind("<Button-1>", self.stopProg)
        bb.pack(side = "bottom")
        #print "hahahahahaha"
        
        #create alert handler which is in charge of displaying
        #threading.Thread(name = alert_thread, target = robot_alert_handler, args = )
        self.robot_alert_handler(self.event_q)
 

    ###################################################
    # Handles prox sensor display and warning(sound).
    # Query event queue(using passed-in queue handle).
    # If there is an "alert" event, display red beams.
    # Erase the beams when "free" event is in queue.
    # This runs in the main GUI thread. Remember to schedule
    # a callback of itself after 50 milliseconds.
    ###################################################
    def robot_alert_handler(self, q):
        if not q.empty():#process an event every time
            current_event = q.get()
            #print(q.qsize(), 'robot alert handler q size')

            # for robot in self.t_handle.robot_list:
                # robot.set_buzzer(0)

            #initialize every time, in case it's free
            self.cc.coords(self.lineLeft, 170, 260, 170, 260)
            self.cc.coords(self.lineRight, 230, 260, 230, 260)

            if current_event.type == "border":
                pass
                #print('currentEvent border')
  
            elif current_event.type == "alert":
                lp = current_event.data[0]
                rp = current_event.data[1]
                self.cc.coords(self.lineLeft, 170, 260, 170, 260-lp*2.5)
                self.cc.coords(self.lineRight, 230, 260, 230, 260-rp*2.5)
                for robot in self.t_handle.robot_list:
                    robot.set_musical_note(40)

            elif current_event.type == "obstacle":
                lp = current_event.data[0]
                rp = current_event.data[1]
                self.cc.coords(self.lineLeft, 170, 260, 170, 260-lp*2.5)
                self.cc.coords(self.lineRight, 230, 260, 230, 260-rp*2.5)

            elif current_event.type == "free":
                for robot in self.t_handle.robot_list:
                    robot.set_musical_note(0)

        self.cc.after(40, self.robot_alert_handler, (q))
     
    def startRobot(self, event=None):
        self.t_handle.go = True
        for robot in self.t_handle.robot_list:
            robot.set_wheel(1, 30)
            robot.set_wheel(0, 30)
        return

    def stopProg(self, event=None):
        self.t_handle.quit = True
        
        for robot in self.t_handle.robot_list:
            robot.reset()
        '''
        self.t_handle.t_motion_handler.join()
        self.t_handle.t_robot_watcher.join()
        self.t_alert_handler.join()
        '''
        self.root.quit()    # close GUI window
        return   

def main():
    max_robot_num = 1   # max number of robots to control
    comm = RobotComm(max_robot_num)
    comm.start()
    print 'Bluetooth starts'
    robotList = comm.robotList

    root = tk.Tk()
    #print "hahahah"
    t_handle = BehaviorThreads(robotList)
    #print "haha"
    gui = GUI(root, t_handle)

    root.mainloop()

    comm.stop()
    comm.join()

if __name__== "__main__":
  sys.exit(main())
  