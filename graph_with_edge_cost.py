import Queue

class Node:
    def __init__(self):
      self.name = ''
      self.data =[]
      self.f_cost = 0
      self.h_cost = 0
      self.back_pointer = False
      self.closed = False
      self.edges = []

class Graph:
    def __init__(self):
        self.nodes = {}
        self.startNode = None
        self.goalNode = None
        self.queue = Queue.PriorityQueue()

    def add_node(self, name, data):
        a_node = Node()
        a_node.name = name
        a_node.data = data
        self.nodes[name] = a_node
        return a_node

    def set_start(self, name):
        self.startNode = name
        self.nodes[name].f_cost = 0
        self.queue.put((0, self.nodes[name]))

    def set_goal(self, name):
        self.goalNode = name

    def add_edge(self, node1, node2, g_cost):
        self.nodes[node1].edges.append([node2, g_cost])
        self.nodes[node2].edges.append([node1, g_cost])

    def Dijkstra(self):
      print "Dijkstra's Greedy Search"
      print "Start: ", self.startNode
      print "Goal: ", self.goalNode
      #print "Queue: ", self.queue

      while not self.queue.empty():
        current_node = self.queue.get()[1]
        #print "current node: ", current_node.name, current_node.f_cost
        for an_edge in current_node.edges:
          a_node_name = an_edge[0]
          if not self.nodes[a_node_name].closed:
            #print "expand next node: ", a_node_name
            f_cost = current_node.f_cost + an_edge[1]
            if not self.nodes[a_node_name].f_cost or self.nodes[a_node_name].f_cost > f_cost:
                self.nodes[a_node_name].f_cost = f_cost
                self.nodes[a_node_name].back_pointer = current_node
                self.queue.put((self.nodes[a_node_name].f_cost, self.nodes[a_node_name]))
                #print "put queue: ", a_node_name, f_cost
                if a_node_name == self.goalNode:
                  #print "found path with cost: ", self.nodes[a_node_name].f_cost
                  #print "path node: ", a_node_name
                  path_node = self.nodes[a_node_name]
                  while path_node.back_pointer != False:
                    path_node = path_node.back_pointer
                    #print "path node: ", path_node.name
        current_node.closed = True
