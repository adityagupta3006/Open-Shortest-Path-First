Student Name:	Aditya Gupta
UNCC ID:	800966229
Programming language: Python 3.5.2
Compiler:	MSC v.1900 32 bit (Intel)
Algorithm:	Dijkstra’s Algorithm
File: 		graph.py

This program requires user input as a python file name, 
an input text file name, a query file as a standard input file name 
and a standard output file name through command line.

Program Description:
Open Shortest Path First implementation using Dijkstra's Algorithm 
Also, implementing the priority queue using min-heap to have an
Efficient running time.

Classes:
The Vertex class holds the information of vertex such as self cost, 
vertex name, parent information, vertex status and maintains an 
adjacency list. 
The Edge Class holds the edge in the graph including the source, 
destination, cost and status of the edge.
The Graph Class builds a graph by implementing an adjacency list which includes the
graph information.
Running time of the shortest path algorithm is O((|V|+|E|)lnV)
The PriorityQueue Class holds the information of the min-heap data structure.

Computing reachable vertices using the function (reachable):
The concept of Breadth first search (BFS) algorithm is incorporated to 
Find out the reachable vertices from each vertex in the graph.
This also prints the reachable vertices (in up status) from every vertex 
Whose status is up.
The running time complexity of this is O(V+E) for each 
Vertex or edge that is up.          
However, in the worst case all Vertices and Edges will be up 
Hence, running time complexity for the overall graph will be
O(V*(V + E)) since the loop runs for each vertex in the graph.

Implementation of the Data Structure:
The priority queue is implemented in the form of a min-heap for 
good efficiency. 
The min-heap is used to store the distances of the vertices from
the source vertex and help to extract the closest vertex very fast.

Changes to the Graph
•	addedge tailvertex headvertex transmit time: add a new edge or update a previous edge.
•	deleteedge tailvertex headvertex: delete an existing edge going from tail vertex to headvertex.
•	edgedown tailvertex headvertex: make the edge status as down.
•	edgeup tailvertex headvertex: make the edge status as up.
•	vertexdown vertex: make the vertex as down.
•	vertexup vertex: make the vertex status as up.

Print the graph
•	print

Reachable vertices:
•	reachable: prints all the reachable vertices.

Finding the Shortest Path
•	path from_vertex to_vertex
      Exit the program:
•	quit


How to run the file:
1. Open the command window.
2. Set the current directory to the location where the file is present.
3. Make sure that the file with graph details (network) and the query file are in the current working directory.
4. Please write quit at the end of file which consists of all the queries.
5. At the command prompt enter:
	python graph.py network.txt < queries.txt > output.txt

The output file will be stored as output.txt

The program works well with the example provided on canvas, 
for other files the efficiency depends on the size of query file and 
the graph (network) file repeating data values.
