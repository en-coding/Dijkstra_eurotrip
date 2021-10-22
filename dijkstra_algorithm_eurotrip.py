import sys

# this code make the implementation of the algorithm more succinct
# it has nothing to do with the Dijkstra algorithm
class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)
        
    def construct_graph(self, nodes, init_graph):
        """
        This method makes sure that the graph is symmetrical. In other words,
        if there's a path from node A to B with a value V, there needs to be a
        path from node B to node A with a value V.
        """
        graph = {}
        for node in nodes:
            graph[node] = {}

        graph.update(init_graph)

        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value
        
        return graph
    
    def get_nodes(self):
        "Returns the nodes of the graph."
        return self.nodes

    def get_outgoing_edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections

    def value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]


# Here start the main part
# graph is an instance of the Graph class
# start_node is the node from which we start the calculations 
def dijkstra_algorithm(graph, start_node):
    
    # Initializr the list of unvisited nodes
    unvisited_nodes = list(graph.get_nodes())
    
    """
    Create two dicts:
    
    1- shortest_path store the best-known cost of visiting each city in the graph 
    starting from the start_node. In the beginning the cost starts at infinity,
    and values are updated later
    
    2- previous_node store the trajectory of the current best known path for each
    node. 
    """
    shortest_path = {}
    previous_nodes = {}
    # We'll use max_value to initialize the "infinity" value of the unvisited nodes
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    # However, we initialize the starting node's value with 0
    shortest_path[start_node] = 0

    # Dijkstra executes until it visits all the nodes in a graph, so we use while-loop
    while unvisited_nodes:
        
        # this block instructs the algorithm to find the node with the lowest value
        current_min_node = None
        for node in unvisited_nodes: #iterate over the nodes
            # if the new path to the neighbor is better than current best, 
            # the algorithm makes the adjustments
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
        
        # this code block retrieves the current node's neighbors and updates the 
        # distances
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # we also update teh best path to the current node
                previous_nodes[neighbor] = current_min_node
        
        unvisited_nodes.remove(current_min_node)
        
    # return the two dictionaries
    return previous_nodes, shortest_path


# print out the results
"""
This function will take the two dictionaries as well as the names of the begenning 
and target nodes. It will use the two dictionaries to find the best path and 
calculate the path's score
"""
def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node

    while node != start_node:
        path.append(node)
        node = previous_nodes[node]

    # Add the start node manually
    path.append(start_node)

    print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ".join(reversed(path)))



# The algorithm in action!
nodes = ["Reykjavik", "Oslo", "Moscow", "London", "Rome", "Berlin", "Belgrade", "Athens"]

init_graph = {}
for node in nodes:
    init_graph[node] = {}

init_graph["Reykjavik"]["Oslo"] = 5
init_graph["Reykjavik"]["London"] = 4
init_graph["Oslo"]["Berlin"] = 1
init_graph["Oslo"]["Moscow"] = 3
init_graph["Moscow"]["Belgrade"] = 5
init_graph["Moscow"]["Athens"] = 4
init_graph["Athens"]["Belgrade"] = 1
init_graph["Rome"]["Berlin"] = 2
init_graph["Rome"]["Athens"] = 2

# We now use this values to create an object of the Graph class
graph = Graph(nodes, init_graph)

# With the graph construct we can then send it to the Dijkstra algorithm
previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node="Reykjavik")

# print out the results
print_result(previous_nodes, shortest_path, start_node="Reykjavik", target_node="Belgrade")