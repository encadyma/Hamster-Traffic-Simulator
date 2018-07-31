'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          bfs_engine.py
   By:            Qin Chen
   Last Updated:  6/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import Queue as que
import sys
# import starter_grid_graph_display as graph_display
import starter_grid_graph
import time
import Tkinter as tk
from comm_ble import RobotComm
from threading import Thread

class BFS(object):
    def __init__(self, graph):
        #graph is a two-dimensional list ==> graph[ [name, [connected nodes]] ]
        self.graph = graph
        return

    ######################################################
    # this function returns the shortest path for given start and goal nodes
    ######################################################
    def bfs_shortest_path(self, startName, goalName):
        visited = []
        visited_order = []

        q = que.Queue()
        q.put([startName, []])

        while not q.empty():
            #print "q is not empty"
            #path = None
            [currentNodeName, path] = q.get()

            print "\n"
            print currentNodeName
            print path
            for node in self.graph.nodes:
                the_path = list(path)
                #print node[0]
                #print currentNodeName
                if node[0] == currentNodeName: 
                    if currentNodeName not in visited:
                        visited_order.append(currentNodeName)
                        #print "visited_order:  ", visited_order
                        visited.append(currentNodeName)
                        the_path.append(currentNodeName)
                        if currentNodeName == goalName:
                            #return visited_order
                            return the_path
                        #otherwise, just push other connected nodes
                        #into the queue
                        for connected_node_name in node[1]:
                            if connected_node_name not in visited:
                                q.put([connected_node_name, the_path])
                                print "expanding: "+ connected_node_name
                                print the_path
        
        
    ######################################################
    # this function returns all paths for given start and goal nodes
    ######################################################
    def bfs_paths(self, start, goal):
        pass
                
    #########################################################
    # This function returns the shortest paths for given list of paths
    #########################################################
    def shortest(self, paths):
        pass

    #########################################################
    # THis function traverses the graph from given start node
    # return order of nodes visited
    #########################################################
    def bfs(self, startName):
        visited = []
        visited_order = []

        q = que.Queue()
        q.put(startName)

        while not q.empty():
            #print "q is not empty"
            currentNodeName = q.get()

            for node in self.graph.nodes:
                #print node[0]
                #print currentNodeName
                if node[0] == currentNodeName: 
                    if currentNodeName not in visited:
                        visited_order.append(currentNodeName)
                        print "visited_order:  ", visited_order
                        visited.append(currentNodeName)
                        for connected_node_name in node[1]:
                            if connected_node_name not in visited:
                                q.put(connected_node_name)
        
        return visited_order

# Robot Programming
# breadth first search
# by Dr. Qin Chen
# May, 2016

##############
# This class supports display of a grid graph. The node location on canvas
# is included as a data field of the graph, graph.node_display_locations.
##############

StartFlag = False
class GridGraphDisplay(object):
    def __init__(self, frame, graph_object):
        self.node_dist = 60
        self.node_size = 44
        self.frame = frame
        self.canvas = None
        self.graph_object = graph_object
        self.width = graph_object.grid_columns * self.node_dist + 80
        self.height = graph_object.grid_rows * self.node_dist + 80

        #two lists for storing the labels and the ovals for displaying the graph
        self.labels = {}
        self.ovals = {}
        self.path_num = []
        self.visited_nodes = set()
        #a list of coordinates
        self.nodes_location = graph_object.node_display_locations
        #[(x1,y1), (x2, y2), (x3, y3)]

        self.start_node = graph_object.startNode
        self.goal_node = graph_object.goalNode

        self.frame = frame
        www = str(self.width+2)
        hhh = str(self.height+60)
        print www
        print hhh

        self.frame.geometry(www+"x"+hhh)
        b1 = tk.Button(self.frame, text = "start")
        b1.bind("<Button-1>", self.startProg)
        b2 = tk.Button(self.frame, text = "exit")
        b2.bind("<Button-1>", self.endProg)
        b1.pack(side = "bottom")
        b2.pack(side = "bottom")
        print "prepare to print graph"
        self.display_graph()
        return

    def startProg(self, event):
        global StartFlag
        StartFlag = True
        print StartFlag

    def endProg(self, event):
        global StartFlag
        StartFlag = False
        self.frame.quit()
        print "program ended"

    # draws nodes and edges in a graph
    def display_graph(self):        
        self.canvas = tk.Canvas(self.frame, height = self.height, width = self.width, bg = "yellow")
        self.canvas.place(x = 0, y = 0)

        #create ovals
        
        print self.nodes_location

        for (yy, xx) in self.nodes_location:
            #print "anything"
            # x and y are the location of the center of the oval
            x = xx*self.node_dist + self.node_size
            y = self.height - yy*self.node_dist -self.node_size

            theOval = self.canvas.create_oval(x - self.node_size/2, y - self.node_size/2, x + self.node_size/2, y + self.node_size/2, outline = "black", fill = "green")
            #print "self.ovals" +str(xx)+str(yy)+"= lbl"
            self.ovals[str(xx)+str(yy)] = theOval
        #display labels
        #print "anything........"

        #in the self.nodes_location, the row info is before column info
        #that is the yy info is before xx info
        for (yy, xx) in self.nodes_location:
            #print "anything"
            # x and y are the location of the center of the oval
            xxxx = xx*self.node_dist + self.node_size
            yyyy = self.height - yy*self.node_dist -self.node_size

            lbl = tk.Label(text = str(yy)+"-"+str(xx), fg = "black", bg = "yellow")
            lbl.place(x = xxxx-13, y = yyyy-11)
            #print "self.labels" +str(xx)+str(yy)+"= lbl"
            self.labels[str(xx)+str(yy)] = lbl

        ###uses draw-edge() to draw lines
        
        #print self.nodes_location
        for (xx, yy) in self.nodes_location:
            #print "\n"
            if yy - 1 >= 0:
                self.draw_edge(xx, yy, xx, yy - 1, "black")
                '''
                print "connecting:" + str(xx) + str(yy)+ " and " + str(xx) + str(yy-1)
                print "1"
                print "\n"
                '''
            if yy + 1 < self.graph_object.grid_columns:
                self.draw_edge(xx, yy, xx, yy + 1, "black")

            if xx - 1 >= 0:
                self.draw_edge(xx - 1, yy, xx, yy, "black")

            if xx + 1 < self.graph_object.grid_rows:
                self.draw_edge(xx + 1, yy, xx, yy,"black")

            self.visited_nodes.add((xx, yy))

    # path is a list of nodes ordered from start to goal node
        return 

    def highlight_path(self, path):
        for node in path:
            self.path_num.append((int(node[0]), int(node[2])))

        #print self.labels
        #print self.ovals
        
        for (yy, xx) in self.path_num:
            self.labels[str(xx)+str(yy)].config(fg = "red", bg = "yellow")
            self.canvas.itemconfig(self.ovals[str(xx)+str(yy)], fill = "red")
        
    # draws a node in given color. The node location info is in passed-in node object

    def draw_node(self, node, n_color):
        pass

    # draws an line segment, between two given nodes, in given color
    #ALSO!!! Check if one of the objects is an obstacle
    # draw_edge(self, (x1, y1), (x2, y2), e_color)
    def draw_edge(self, y1, x1, y2, x2, e_color):
        #print self.graph_object.obs_list

        tmp = [y1, x1]
        tmp2 = [y2, x2]
        for node in self.graph_object.obs_list:
            if tmp == node:
                return
            if tmp2 == node:
                return

        for node in self.visited_nodes:
            if tmp == node:
                return
            if tmp2 == node:
                return

        #print type(self.visited_nodes)
        '''
        if tmp in self.visited_nodes:
            return
        if tmp2 in self.visited_nodes:
            return 
        #list object unhashable?
        '''

        #print "connecting:" + str(y1) + str(x1)+ " and " + str(y2) + str(x2)

        #switch to the coordinates for the canvas drawing
        x1 = x1*self.node_dist + self.node_size
        y1 = self.height - y1*self.node_dist -self.node_size
        x2 = x2*self.node_dist + self.node_size
        y2 = self.height - y2*self.node_dist -self.node_size

        self.canvas.create_line(x1, y1, x2, y2, fill = e_color, width = 3)

class movingTheRobot(object):
    def __init__(self, robotList, order_que):
        print order_que
        self.robotList = robotList
        #print self.robotList
        self.order_que = order_que
        #self.main_movement()
        return

    def main_movement(self):
        global StartFlag
        while not StartFlag:
            #print "waiting for start order"
            time.sleep(0.01)
            pass

        #print "hahahah"
        currentState = 0 # currentState = 0 or 90 or 180 or 270, that is the steering
        #set the initial state here
        for i in range(len(self.order_que) - 1):
            if StartFlag == False:
                return

            yy = int(self.order_que[i+1][0])
            xx = int(self.order_que[i+1][2])
            y = int(self.order_que[i][0])
            x = int(self.order_que[i][2])

            print yy, xx
            print y, x

            ##### change the initial facing of the robot here###
            if yy == y + 1:
                if currentState == 0:
                    self.move_forward()
                    currentState = 0
                elif currentState == 90:
                    self.move_left()
                    currentState = 0
                elif currentState == 270:
                    self.move_right()
                    currentState = 0
            if yy == y - 1:
                if currentState == 0:
                    self.move_backward()
                    currentState = 0
                elif currentState == 90:
                    self.move_left()
                    currentState = 0
                elif currentState == 270:
                    self.move_right()
                    currentState = 0
            if xx == x + 1:
                if currentState == 0:
                    self.move_right()
                    currentState = 90
                elif currentState == 90:
                    self.move_forward()
                    currentState = 90
                elif currentState == 270:
                    self.move_backward()
                    currentState = 270
            if xx ==  x - 1:
                if currentState == 0:
                    self.move_left()
                    currentState = 270
                elif currentState == 90:
                    self.move_backward()
                    currentState = 90
                elif currentState == 270:
                    self.move_forward()
                    currentState = 270


            i += 1

        #when everything is done, stop the program
        StartFlag = False 

    def move_forward(self):
        #print "trying to move forward"
        global StartFlag
        if self.robotList:
            for robot in self.robotList:
                intersection = False
                robot.set_wheel(0, 50)
                robot.set_wheel(1, 50)
                time.sleep(0.2)
                #print "still trying to move forward"
                while not intersection and StartFlag:
                    #print "moving forward"
                    if self.get_floor(True) < 10 and self.get_floor(False) < 10:
                        intersection = True
                        print "There is an intersection"
                    else:
                        print "moving forward"
                        if self.get_floor(True) < 10:
                            robot.set_wheel(0,10)
                            robot.set_wheel(1,30)
                        elif self.get_floor(False) < 10:
                            robot.set_wheel(1,10)
                            robot.set_wheel(0,30)
                        else:
                            robot.set_wheel(0,50)
                            robot.set_wheel(1,50)
                    time.sleep(0.01)
                robot.set_wheel(0, 0)
                robot.set_wheel(1, 0)
                #when out of the while-loop, meaning that the intersection is detected,
                #we should pause for a little time in the main_movement and keep the robot moving for some time,
                #to prevent it from constantly detecting the intersection
        else:
            print "waiting for robot"

    def move_backward(self):
        #print "trying to move backward"
        if self.robotList:
            for robot in self.robotList:
                intersection = False
                robot.set_wheel(0, -40)
                robot.set_wheel(1, -40)
                time.sleep(0.2)
                #print "still trying to move backward"
                while not intersection and StartFlag:
                    #print "moving backward"
                    if self.get_floor(True) < 10 and self.get_floor(False) < 10:
                        intersection = True
                        print "There is an intersection"
                    else:
                        print "moving backward"
                        if self.get_floor(True) < 10:
                            robot.set_wheel(0,-10)
                            robot.set_wheel(1,-35)
                        elif self.get_floor(False) < 10:
                            robot.set_wheel(1,-10)
                            robot.set_wheel(0,-35)
                        else:
                            robot.set_wheel(0,-40)
                            robot.set_wheel(1,-40)
                    time.sleep(0.01)

                robot.set_wheel(0, 0)
                robot.set_wheel(1, 0)
        else:
            print "waiting for robot"

    def move_left(self):
        #print "trying to move left"
        global StartFlag
        if self.robotList and StartFlag:
            for robot in self.robotList:
                print "turning left"
                robot.set_wheel(0, -5)
                robot.set_wheel(1, 40)
                time.sleep(1.2)
                self.move_forward()
                print "and then..."
                print "moving forward"
        else:
            print "waiting for robot"

    def move_right(self):
        #print "trying to move right"
        global StartFlag
        if self.robotList and StartFlag:
            for robot in self.robotList:
                print "moving right"
                robot.set_wheel(1, -5)
                robot.set_wheel(0, 40)
                time.sleep(1.2)
                self.move_forward()
                print "and then..."
                print "moving forward"
        else:
            print "waiting for robot"

    '''
    def get_prox(self, left):
        #print "the get_prox is running"
        if self.robotList:
            for robot in self.robotList: # Single return values
                if left:
                    return int(robot.get_proximity(0)) # The function just ends here 
                else:
                    return int(robot.get_proximity(1)) # Or here
    '''
    def get_floor(self, left):
        if self.robotList:
            for robot in self.robotList:
                if left:
                    return robot.get_floor(0) # The function just ends here 
                else:
                    return robot.get_floor(1) # Or here

    def stop_move(self):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0, 0)
                robot.set_wheel(1, 0)
            global current_wheel_speed
            current_wheel_speed[0] = 0
            current_wheel_speed[1] = 0

    def reset_robot(self, ): # use Hamster API reset()
        if self.robotList:
            for robot in self.robotList:
                robot.reset()
def main():
    '''
    graph = [
         ['A', ['B', 'C']],
         ['B', ['A', 'E', 'D']],
         ['C', ['A', 'F', 'G']],
         ['D', ['B', 'H']],
         ['E', ['B','I', 'J']],
         ['F', ['C','K']],
         ['G', ['C']],
         ['H', ['D']],
         ['I', ['E']],
         ['J', ['E']],
         ['K', ['F']]
         ]

    '''
    graph = starter_grid_graph.GridGraph()
    graph.set_grid_rows(5)
    graph.set_grid_cols(4)

    # origin of grid is (0, 0) lower left corner
    # graph.obs_list = ([1,1],)    # in case of one obs. COMMA
    graph.obs_list = ([1,1], [3,0], [2,2], [3,3])
    
    #graph.set_start("0-0")
    #graph.set_goal("2-1")
    
    graph.make_grid()
    graph.connect_nodes()
    graph.compute_node_locations()

    bfs = BFS(graph)

    shortest_path = bfs.bfs_shortest_path("0-0", "4-2")

    #####For the use of UI display######
    rootWin = tk.Tk()
    gridDisplay = GridGraphDisplay(rootWin, graph)
    #print "debugging!!"
    #print bfs.bfs_shortest_path("0-0", "2-1")
    gridDisplay.highlight_path(shortest_path)
    #print "hhahahaha"
    
    print "hahahahh"
    ######Starting to control the robot######
    gMaxRobotNum = 1 # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    robotList = comm.robotList

    #robot control runs in a separate thread and waits for the order to start
    myRobot = movingTheRobot(robotList, shortest_path)
    t_robot = Thread(name = "t_robot", target = myRobot.main_movement)
    t_robot.setDaemon(True)
    t_robot.start()

    #batterylbl = tk.Label(rootWin, text = "")

    rootWin.mainloop()
    return

if __name__ == "__main__":
    sys.exit(main())