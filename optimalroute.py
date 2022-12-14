from cmath import inf
import heapq
"""
Name: Syed Zubin Hafiz
Version: 1.3

"""
"""Question 1"""
class Vertex:
    def __init__(self, name):
        self.name = name
        self.edges = []
        self.distance = float('inf')
        self.last_vertice = None
        self.visited = False 
        self.waiting_time = 0
class Edge:
    def __init__(self,u,v,w):
        self.u = u
        self.v = v 
        self.w = w
        
# A class to represent a graph object
class Graph:
    # Constructor to construct a graph
    def __init__(self, edges, vertices):
 
        # A list of lists to represent an adjacency list
        self.vertices = [None] * vertices
 
        # Allocate memory for the adjacency list
        for i in range(vertices):
            self.vertices[i] = Vertex(i)
 
        # Add edges to the directed graph
        for (source, destination, weight) in edges:
            self.vertices[source].edges.append(Edge(self.vertices[source],self.vertices[destination],weight))

class RoadGraph:
    def __init__(self,roads,cafes):
        # Find the largest vertex name within roads
        maxIndex = 0
        for i in range(len(roads)):
            maxIndex = max(roads[i][0],roads[i][1],maxIndex)
        # Define base graph
        self.g = Graph(roads,maxIndex+1)
        # Construct the reversed version of roads
        inverse_roads = []
        for i in range(len(roads)):
            u = roads[i][0]
            v = roads[i][1]
            w = roads[i][2]
            inverse_roads.append((v,u,w))
        # Define inverse graph
        self.gInverse = Graph(inverse_roads,maxIndex+1)
        # Store cafes
        self.cafeList = []
        for (index,weight) in cafes:
            self.cafeList.append(self.g.vertices[index])
            self.g.vertices[index].waiting_time = weight
            
            
    def dijkstra(self,source,graph):
        # Inspired from the following solution : https://leetcode.com/problems/network-delay-time/discuss/329376/efficient-oe-log-v-python-dijkstra-min-heap-with-explanation
        
        # Reset all distances and last vertices to infinity and None
        for i in range(len(graph.vertices)):
            graph.vertices[i].distance = float('inf')
            graph.vertices[i].last_vertice = None  
        # Set source to zero
        graph.vertices[source].distance = 0
        # Instantiate the heap
        minimum_distance = [(0,source)]
        # Loop until heap is empty
        while minimum_distance:
            # Get distance and vertex to check
            current_distance,current = heapq.heappop(minimum_distance)
            # Skip if already processed
            if graph.vertices[current].visited ==True:
                continue
            # Go through neighbors
            for neighbor in graph.vertices[current].edges:
                # Skip neighbours you have visited
                if neighbor.v.visited == True:
                    continue
                # Calculate distance to neighbour's vertex if this edge is used
                this_distance = current_distance+neighbor.w
                # Check if using this edge is better
                if this_distance < neighbor.v.distance:
                    # Update accordingly
                    neighbor.v.distance = this_distance
                    neighbor.v.last_vertice = graph.vertices[current]
                    heapq.heappush(minimum_distance,(this_distance,neighbor.v.name))
                    
                    
    def routing(self,start,end):
        """Finds the shortest path from starting location to end location, considering the route to a cafe and the corresponding
        waiting time.
        Time Complexity: O(|E| log |V |) 
        Space Complexity:O(|V | + |E|)"""
        self.dijkstra(start,self.g)    
        self.dijkstra(end,self.gInverse)
        minCafe = None
        minDist = float('inf')
        #Loop through all the cafes to find out which one provides the fastest route between start and end 
        for cafe in self.cafeList:
            dist = self.g.vertices[cafe.name].distance + self.gInverse.vertices[cafe.name].distance + cafe.waiting_time
            if(dist<minDist):
                minCafe = cafe
                minDist = dist 
        # Starting at cafe, backtrack to start
        startPath = []
        u = minCafe 
        while(self.g.vertices[u.name].last_vertice!=None):
            u = self.g.vertices[u.name].last_vertice
            startPath.append(u.name)
        # Starting at cafe, backtrack to end
        endPath = []
        u = minCafe 
        while(self.gInverse.vertices[u.name].last_vertice!=None):
            u = self.gInverse.vertices[u.name].last_vertice
            endPath.append(u.name)
        startPath.reverse()
        # Concatenate everything to get the path
        path = startPath + [minCafe.name] + endPath
        # Return
        return path
 
#--------------------------------------------------------------------------#     
"""TEST CASES""" 
if __name__ == '__main__':
# Question 1
    roads = [(0,1,4),(1,2,2),(2,3,3),(3,4,1),(1,5,2),
        (5,6,5),(6,3,2),(6,4,3),(1,7,4),(7,8,2),
        (8,7,2),(7,3,2),(8,0,11),(4,3,1),(4,8,10)]
    cafes = [(5,10),(6,1),(7,5),(0,3),(8,4)]
    rg = RoadGraph(roads,cafes)
    print(rg.routing(1,7))
    rg = RoadGraph(roads,cafes)
    print(rg.routing(1,3))
    rg = RoadGraph(roads,cafes)
    print(rg.routing(1,4))
    rg = RoadGraph(roads,cafes)
    print(rg.routing(3,4))
    print(rg)
