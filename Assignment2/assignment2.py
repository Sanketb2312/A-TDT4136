from numpy.core.fromnumeric import trace
from Map import Map_Obj
import numpy as np

#klasse for å lage graf. Denne lager grafen for kartet. Alle nodene som det er mulig å gå på blir laget 
# i denne klassen. self.edges er det som kalles for children i pseudokoden, og elementene lagres som tupler
# på formen ((Node), weight), altså noden og vekten i noden. 
class Graph:
    def __init__(self):
        self.map = Map_Obj()
        self.nodes = [[None for x in range(len(self.map.str_map[0]))] for y in range(len(self.map.str_map))]
        self.edges = []
        self.goal = Node(0,0)
        self.start = Node(0,0)

#går gjennom alle nodene, og sjekker hvilke barn de har, altså nodene som er over, under, til venstre
# og til høyre. Her lages da edge
    def path_nodes(self):
        for y in range(len(self.map.str_map)):
            for x in range(len(self.map.str_map[y])):
                if self.map.str_map[y][x] != ' # ':
                    node = Node(x,y)
                    if self.map.str_map[y][x] == ' S ':
                        self.start = node
                    if self.map.str_map[y][x] == ' G ':
                        self.goal = node
                    self.nodes[y][x]=node
                    if x > 0 and self.nodes[y][x-1]!= None:
                        node.edges.append((self.nodes[y][x-1], self.map.int_map[y][x-1]))
                        self.nodes[y][x-1].edges.append((node,self.map.int_map[y][x]))
                    if y > 0 and self.nodes[y-1][x]!= None:
                        node.edges.append((self.nodes[y-1][x], self.map.int_map[y-1][x]))
                        self.nodes[y-1][x].edges.append((node, self.map.int_map[y][x]))

#node klasse

class Node:
    def __init__(self, x, y):
        self.parent = None
        self.children = []
        self.x = x
        self.y = y
        self.edges=[]
        self.map = Map_Obj()
        
    def f(self, goal_node):
        return self.g()+self.h(goal_node)

    def g(self):
        if self.parent == None:
            return 0
        return self.parent.g()+self.map.int_map[self.y][self.x]

    def h(self, goal_node):
        return self.manhattan_distance(goal_node)

    def manhattan_distance(self, goal_node):
        return abs(goal_node.x-self.x)+abs(goal_node.y-self.y)
    
class BFS:
    def __init__(self):
        self.open = []
        self.closed = []
        self.shortest_path = []
        self.count = 0
    
    #bruker denne for push i search algoritmen, slik at alt settes inn i stigende rekkefølge
    def push_in_ascending_order(self, node, goal_node):
        for x in range(len(self.open)):
            if node.f(goal_node) < self.open[x].f(goal_node):
                self.open.insert(x, node)
                break
        self.open.append(node)
    
    def search(self, start, goal): 
        self.open.append(start)
        while len(self.open) != 0:
            self.count+=1
            print(self.count)
            if  len(self.open)==0:
                return False
            x = self.open.pop(0)
            self.closed.append(x)
            if x == goal:
                return True
            for node in x.edges:
                node[0].edges.append((x,node[1]))
                if node[0] not in self.open and node[0] not in self.closed:
                    self.attach_and_eval(node[0], x)
                    self.push_in_ascending_order(node[0], goal)
                elif x.g()+node[1] < node[0].g():
                    self.attach_and_eval(node[0],x)
                    if node[0] in self.closed:
                        self.propagate_path_improvements(node[0])
       


    def attach_and_eval(self, C,P):
        C.parent = P


    def propagate_path_improvements(self, P):
        if type(P) != Node:
            P = P[0]
        for C in P.edges:
            if P.g()+C[1] <= C[0].g():
                C[0].parent = P
                self.propagate_path_improvements(C)
    
    #bruker denne til å traversere bakover fra goal. Finner da goal sin parent, også den nye noden sin parent
    #osv til parent er None,legger alle nodene inn i listen, som da er den korteste veien fra Start til Goal
    def trace_back_parent(self,node):
        if node.parent == None:
            return
        self.shortest_path.append(node)
        return self.trace_back_parent(node.parent)


graph = Graph()
graph.path_nodes()
bfs = BFS()
bfs.search(graph.start, graph.goal)
bfs.trace_back_parent(graph.goal.parent)
map = Map_Obj()
map.show_map(bfs.shortest_path)
        
            
        
        





