class RouteManager:
    nodesList = []
    edgesList = []
    adjacencyList = dict()
    routesList = []

    def __init__(self, nodes_list, edges_list):
        self.nodesList = nodes_list
        self.edgesList = edges_list
        self.GenerateAdjacencyList()
        nodes_route_list = list(self.GenerateAllRoutes('0/0','3/3',self.adjacencyList))
        self.routesList = self.FormatRoutes(nodes_route_list)

    def GenerateAdjacencyList(self):
        for node in self.nodesList:
            if node not in self.adjacencyList:
                self.adjacencyList[node] = []
            for edge in self.edgesList:
                origin_node, dest_node = self.SplitEdge(edge)
                if origin_node == node:
                    self.adjacencyList[node].append(dest_node)

    # Splite edge 0/0to0/1 in -> origin = 0/0, dest = 0/1
    def SplitEdge(self, edge):
        edge_nodes = edge.split("to")
        origin = edge_nodes[0]
        dest = edge_nodes[1]
        return origin, dest

    # It runs a BFS Algorithm
    def GenerateAllRoutes(self, start_node, dest_node, network_adjacency_list):
        queue = [(start_node, [start_node])]
        while queue:
            (vertex, path) = queue.pop(0)
            for next in set(network_adjacency_list[vertex]) - set(path):
                if next == dest_node:
                    yield path + [next]
                else:
                    queue.append((next, path + [next]))

    #Format the routes to the following format -> 0/0to0/1
    def FormatRoutes(self, route_nodes):
        routesList = []
        route = ""
        for nodesList in route_nodes:
            i = 0
            while i < len(nodesList) - 1:
                route += nodesList[i]
                route += "to"
                route += nodesList[i+1]
                i += 1
                if i < len(nodesList) - 1:
                    route += " "
            routesList.append(route)
            route = ""
        return routesList
