import random

class RouteManager:
    nodesList = []
    edgesList = []
    adjacencyList = dict()
    routesList = []

    def Initialize(self, nodes_list, edges_list, numberOfNodes):
        self.nodesList = nodes_list
        self.edgesList = edges_list
        self.GenerateAdjacencyList()
        nodes_route_list = list(self.GenerateAllRoutes('0/0',str(numberOfNodes - 1) + "/" + str(numberOfNodes - 1),self.adjacencyList))
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

    def GenerateRouteFileWithRouteDistribution(self, numberOfRoutes, numberOfCars):
        with open("src/speed_control.rou.xml", "w") as routes:
            print("""<?xml version="1.0" encoding="UTF-8"?>\n<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
    <routeDistribution id="routedist1">""", file=routes)
            i = 0
            while i < numberOfRoutes:
                route = random.choice(self.routesList)
                color = str(round(random.random(),2)) + "," + str(round(random.random(),2)) + "," + str(round(random.random(),2))
                prob = str(round(random.random(),1))
                print('\t\t\t<route id="route' + str(i) + '" color="' + color + '" edges="' + route + '" probability="'+ prob +'"/>', file=routes)
                i += 1
            print('    </routeDistribution>\n\t<vType id="normal car" vClass="passenger" maxSpeed="40" speedFactor="0.9" speedDev="0.2" sigma="0.5"/>\n\t<flow id="normal" type="normal car"' + str(numberOfCars) + '"route="routedist1"/>\n</routes>', file=routes)

    def GenerateRouteFileWithTaz(self, numberOfCars, n_nodes):
        with open("src/speed_control.rou.xml", "w") as routes:
            print("""<?xml version="1.0" encoding="UTF-8"?>\n<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">""", file=routes)
            print('\t<vType id="normal car" vClass="passenger" maxSpeed="40" speedFactor="0.9" speedDev="0.2" sigma="0.5"/>\n\t<flow id="normal" type="normal car" begin="0" number="' + str(numberOfCars) + '" fromTaz="origin_n' + str(n_nodes) + '" toTaz="destination_n' + str(n_nodes) + '"/>\n</routes>', file=routes)
