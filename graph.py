# OSPF Using Dijkstra's Algorithm
# Name: Aditya Gupta

import sys
from sys import argv
from queue import *

# Implementing a priority Queue implementation using binary heap.
class priorityQueue(object):
    def __init__(self):
        self.verticesList = [] 
    
    # function to find the left element.
    def left(self, i):
        return (2*i)+1

    # function to find the right element.
    def right(self, i):
        return (2*i)+2

    # to find the parent node.
    def parent(self, i):
        if i % 2 == 0 and i != 0:
            return int((i/2)-1)
        elif i % 2 != 0:
            return int(i/2)
        else:
            return 0

    # Function Implements minHeapify. 
    # A is the list to be heapified,
    # i is the index i
    # n is the length of list A.
    def minHeapify(self, A, i, n):
        l = self.left(i)
        r = self.right(i)
        if l <= n and A[l][0] < A[i][0]:
            smallest = l
        else:
            smallest = i
        if r <= n and A[r][0] < A[smallest][0]:
            smallest = r
        if smallest != i:
            A[i], A[smallest] = A[smallest], A[i]
            self.minHeapify(A, smallest, n)

    # function to build min heap.
    def buildMinHeap(self, A, n):
        for i in range(int(n/2)-1, -1, -1):
            self.minHeapify(A, i, n-1)

    # function to insert an element
    # to the priority queue.
    def insert(self, A, element):
        A.insert(len(A), element)
        self.buildMinHeap(A, len(A))

    # function to decrease priority key of an
    # element in the heap.
    def decreasePriority(self, A, i, key):
        A[i] = key
        while i > 0 and A[self.parent(i)][0] > A[i][0]:
            A[i], A[self.parent(i)] = A[self.parent(i)], A[i]
            i = self.parent(i)

    # function to find the minimum value from the
    # priority queue.
    def extractMin(self, A):
        minimum = A[0]
        A[0] = A[len(A)-1]
        del A[len(A)-1]
        self.minHeapify(A, 0, (len(A) - 1))
        return minimum[1]

# Vertex class is used to store information of all the vertices.
class Vertex(object):
    def __init__(self, vertex_name, status):
        self.name = vertex_name     # vertex name
        self.status = status        # status of the vertex
        self.parent = None          # parent of the vertex
        self.cost = float('inf')    # distance to the self
        self.adj = []               # list of adjacent vertices

    # function to reset the information. 
    def reset(self):                
        self.dist = float('inf')
        self.prev = None
        #adjacent vertices

# Edge class is used to store information of all the edges.
class Edge(object):
    def __init__(self, source_vertex, destination_vertex, cost, status):
        self.source_vertex = source_vertex              # source vertex of the edge
        self.destination_vertex = destination_vertex    # destination vertex of the edge
        self.status = status                            # status of the edge
        self.cost = float(cost)                         # cost/weight of the edge
        
# Graph class to implement the graph.        
class Graph(object):
    def __init__(self):
        self.vertices = {}          # list of the vertices.
        self.edges = {}             # list of the edges.
        self.adjlist = {}           # maintaining an adjacency list of all vertices.

    # funciton to add a new vertex to the graph.
    def addvertex(self, name, vertex):
        self.vertices[name] = vertex

    # function to make the vertex status as down.    
    def vertexdown(self, vertex):
        self.vertices[vertex].status = False

    # function to make the vertex status as down.
    def vertexup(self, vertex):
        self.vertices[vertex].status = True
    
    # function to add an edge to the graph.  
    def addedge(self, v1_to_v2, edge):        
        if v1_to_v2[0] not in self.vertices:          # if the source vertex is not present already.
            self.addvertex(v1_to_v2[0], Vertex(v1_to_v2[0], True))  
        
        if v1_to_v2[1] not in self.vertices:          # if the destination vertex is not present already.
            self.addvertex(v1_to_v2[1], Vertex(v1_to_v2[1], True))
        
        if (v1_to_v2[0], v1_to_v2[1]) in self.edges:  # updating the edge weight if vertices are already present.
            self.edges[(v1_to_v2[0],v1_to_v2[1  ])].cost = float(edge.cost)
        else:
            self.addAdjVertex(v1_to_v2[0], v1_to_v2[1])
            self.addAdjVertex(v1_to_v2[1], None)
            self.edges[v1_to_v2] = edge
    
    # funciton to add the vertex in the adjacency list.
    def addAdjVertex(self, v1, v2):
        if v2 == None:
            self.adjlist.setdefault(v1,[])
        else:
            self.adjlist.setdefault(v1,[]).append(v2)    
    
    # function to make the edge status as down.
    def edgedown(self, v1, v2):
        self.edges[(v1, v2)].status = False

    # function to make the edge status as up.
    def edgeup(self, v1, v2):
        self.edges[v1, v2].status = True

    # funcition to delete an edge from vertex v1 to vertex v2.
    def deleteEdge(self, v1, v2):
        del self.edges[(v1,v2)]
        self.adjlist[v1].remove(v2)

    # print the graph as output.
    def printGraph(self):
        for v in sorted(self.vertices.keys()):
            if self.vertices[v].status == False:
                print(self.vertices[v].name, "DOWN")
            else:
                print(self.vertices[v].name) 

            for adj_vertex in sorted(self.adjlist[v]):
                if (self.edges[(v,adj_vertex)].status == False): 
                    print(" ", adj_vertex,self.edges[(v,adj_vertex)].cost, "DOWN")  
                else:
                    print(" ", adj_vertex,self.edges[(v,adj_vertex)].cost)  
    
    # print all the reachable vertices from each vertex.
    def printReachable(self):
        for vertex in (sorted(self.vertices.keys())):
            if self.vertices[vertex].status == True:
                self.reachable(vertex)

    # finding all the reachable vertices implementing BFS.
    # This has a running time complexity of O(V+E) for each 
    # Vertex or edge that is up.
    # for worst case the time complexity would be O(V*(V + E)) 
    def reachable(self, vertex):
        discovered_vertices = {}
        reachable_vertices = {}
        for ver in self.vertices.keys():
            discovered_vertices[ver] = "white" 
        discovered_vertices[vertex] = "gray"        
        queue = Queue()
        queue.put(vertex)
        while not queue.empty():
            get = queue.get()
            for v in sorted(self.adjlist[get]):
                if discovered_vertices[v] == "white" and self.vertices[v].status == True and self.edges[(get,v)].status == True:
                    discovered_vertices[v] == "gray"
                    queue.put(v)
                    reachable_vertices[v] = v
            discovered_vertices[get] = "black"
        print(vertex)
        for vert in sorted(reachable_vertices):
            print(" ", vert)
        
    # funciton to reset the vertices.
    def clearAll(self):
        for vertex in self.vertices.values():
            vertex.reset()

    # calculating the shortest path from source to destination 
    # implementing dijsktra's algorithm and priority queue using Binary Min Heap.
    # this has a running time complexity of O((|V|+|E|)lnV)
    def path(self, source, destination):
        #self.clearAll()
        pq = priorityQueue()
        for vertex in self.vertices.keys():
            self.vertices[vertex].parent = None
            self.vertices[vertex].cost = float('inf')
        self.vertices[source].cost = 0.0
        dist = []
        for w in self.vertices:
            dist.insert(len(dist), (self.vertices[w].cost, self.vertices[w]))
        pq.buildMinHeap(dist, len(dist))  # build a binary min heap for the priority queue
        s = []
        while dist:
            v = pq.extractMin(dist)  # extract the minimum distance element from the priority queue
            if v.status == False:  # if the vertex is not active, skip the vertex
                continue
            else:
                s.insert(len(s), v)  # marked the vertex as visited
                for element in self.adjlist[v.name]:  # for each element in the adjacency list of vertex v,
                    if self.vertices[element].status == True and self.edges[(v.name, element)].status == True:
                        prevDistance = self.vertices[element].cost
                        if self.vertices[element].cost > (self.vertices[v.name].cost + self.edges[(v.name,element)].cost) :
                            self.vertices[element].cost = self.vertices[v.name].cost + self.edges[(v.name,element)].cost
                            self.vertices[element].parent = v
                            index = dist.index((prevDistance, self.vertices[element]))
                            pq.decreasePriority(dist, index, (self.vertices[element].cost, self.vertices[element]))
        node = self.vertices[destination]
        while node.parent is not None:   #Display the vertices in the correct order
            dist.append(node.name)
            node = node.parent
        dist.append(node.name)
        dist.reverse()
        print(" ".join([str(vert) for vert in dist]),"%.2f" % self.vertices[destination].cost)


# taking the input file from command line
# defining the maximum table size
# opening the input file
# reading the input file and storing the file data into data variable
def main():
    input_file = argv[1]
    f = open(input_file,'r')
    graph = Graph()
    for line in f:
        node = line.split()
        if len(node) != 3:
            print("Ill formatted Line ", end="")
            print(node)
            exit()
        else:
            v1 = Vertex(line.split()[0], True)  # vertex 1
            v2 = Vertex(line.split()[1], True)  # vertex 2
            graph.addvertex(v1.name,v1)         # add vertex 1
            graph.addvertex(v2.name,v2)         # add vertex 2
            edge1 = Edge(line.split()[0], line.split()[1], line.split()[2], True)   # make edge 1    
            edge2 = Edge(line.split()[1], line.split()[0], line.split()[2], True)   # make edge 2
            graph.addedge((v1.name, v2.name), edge1)    # add edge 1
            graph.addedge((v2.name, v1.name), edge2)    # add edge 1
    
    f.close()   # close the input graph file

    # reading the queries.
    while True:
        line = sys.stdin.readline()
        if line.strip():
            input_query = line.split()
            if len(input_query) == 4:
                if input_query[0] == "addedge":
                    e1 = Edge(input_query[1], input_query[2], input_query[3], True)
                    graph.addedge((input_query[1],input_query[2]), e1)
                else:
                    print("Invalid Command, Please try again.")

            elif len(input_query) == 3:
                if input_query[0] == "deleteedge":
                    graph.deleteEdge(input_query[1], input_query[2])

                elif input_query[0] == "edgedown":
                    graph.edgedown(input_query[1], input_query[2])

                elif input_query[0] == "edgeup":
                    graph.edgeup(input_query[1], input_query[2])

                elif input_query[0] == "path":
                    if input_query[1] not in graph.vertices.keys():
                        print("Source Vertex Not found.")
                    
                    elif input_query[2] not in graph.vertices.keys():
                        print("Destination Vertex Not found.")
                    
                    else:
                        graph.path(input_query[1],input_query[2])

                else:
                    print("Invalid Command, Please try again.")

            elif len(input_query) == 2:
                if input_query[0] == "vertexdown":
                    graph.vertexdown(input_query[1])

                elif input_query[0] == "vertexup":
                    graph.vertexup(input_query[1])

                else:
                    print("Invalid Command, Please try again")
                
            elif len(input_query) == 1:
                if input_query[0] == "reachable":
                    graph.printReachable()

                elif input_query[0] == "print":
                    graph.printGraph()
                
                elif input_query[0].lower() == "quit":
                    break

                else:
                    print("Wrong Command. Please try again")
                
            else:
                print("Wrong Command. Please try again")

# calling the main function
if __name__ == '__main__':
    main()
