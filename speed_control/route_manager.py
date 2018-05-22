import random
import sumolib
import traci

class RouteManager:
    nodesList = []
    edgesList = []
    adjacencyList = dict()
    routesList = []
    usedRoutes = []
    activeRoutes = []

    def Initialize(self, nodes_list, edges_list, numberOfNodes, onlyForward):
        self.nodesList = nodes_list
        self.edgesList = edges_list
        self.GenerateAdjacencyList(onlyForward)
        nodes_route_list = list(self.GenerateAllRoutes('0/0',str(numberOfNodes - 1) + "/" + str(numberOfNodes - 1),self.adjacencyList))
        self.routesList = self.FormatRoutes(nodes_route_list)

    def GenerateAdjacencyList(self, onlyForward):
        for node in self.nodesList:
            if node not in self.adjacencyList:
                self.adjacencyList[node] = []
            for edge in self.edgesList:
                origin_node, dest_node = self.SplitEdge(edge)
                if origin_node == node:
                    if onlyForward:
                        if dest_node.split("/")[0] > origin_node.split("/")[0] or \
                           dest_node.split("/")[1] > origin_node.split("/")[1]:
                           self.adjacencyList[node].append(dest_node)
                    else:
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
                print(str(self.routesList.index(route)))
                color = str(round(random.random(),2)) + "," + str(round(random.random(),2)) + "," + str(round(random.random(),2))
                prob = str(round(random.random(),1))
                print('\t\t\t<route id="route' + str(i) + '" color="' + color + '" edges="' + route + '" probability="'+ prob +'"/>', file=routes)
                i += 1
            print('    </routeDistribution>\n\t<vType id="normal car" vClass="passenger" maxSpeed="40" speedFactor="0.9" speedDev="0.2" sigma="0.5"/>\n\t<flow id="normal" type="normal car" number="' + str(numberOfCars) + '" route="routedist1"/>\n</routes>', file=routes)

    def GenerateRouteFileWithTaz(self, numberOfCars, n_nodes):
        with open("src/speed_control.rou.xml", "w") as routes:
            print("""<?xml version="1.0" encoding="UTF-8"?>\n<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">""", file=routes)
            print('\t<vType id="normal car" vClass="passenger" maxSpeed="40" speedFactor="0.9" speedDev="0.2" sigma="0.5"/>\n' +
                  '\t<flow id="normal1" color="1,1,0" type="normal car" begin="0" number="' + str((int)(numberOfCars/3)) + '" fromTaz="origin1_n' + str(n_nodes) + '" toTaz="destination1_n' + str(n_nodes) + '"/>\n'
                  '\t<flow id="normal2" color="1,0,1" type="normal car" begin="0" number="' + str((int)(numberOfCars/3)) + '" fromTaz="origin2_n' + str(n_nodes) + '" toTaz="destination2_n' + str(n_nodes) + '"/>\n'
                  '\t<flow id="normal3" color="0,1,1" type="normal car" begin="0" number="' + str((int)(numberOfCars/3)) + '" fromTaz="origin3_n' + str(n_nodes) + '" toTaz="destination3_n' + str(n_nodes) + '"/>\n</routes>', file=routes)

    def GetEdgesList(self, n_nodes):
        edges_list = []
        net = sumolib.net.readNet("src/no_rl/n" + str(n_nodes) + "_cross.net.xml")
        edges_list_xml = net.getEdges()
        print(edges_list_xml)
        for edge_xml in edges_list_xml:
            edges_list.append(edge_xml.getID())
        return edges_list

    def GetNodesList(self, n_nodes):
        nodes_list = []
        net = sumolib.net.readNet("src/no_rl/n" + str(n_nodes) + "_cross.net.xml")
        nodes_list_xml = net.getNodes()
        print(nodes_list_xml)
        for node_xml in nodes_list_xml:
            nodes_list.append(node_xml.getID())
        return nodes_list

    def AddRoutesToTraci(self, numberOfRoutes):
        i = 0
        while i < numberOfRoutes:
            routeString = random.choice(self.routesList)
            route = routeString.split(" ")
            routeIndex = self.routesList.index(routeString)
            traci.route.add(str(routeIndex), route)
            i += 1
            self.usedRoutes.append(str(routeIndex))
            self.activeRoutes.append(str(routeIndex))

    def AddAllRoutesToTraci(self):
        for routeString in self.routesList:
            route = routeString.split(" ")
            routeIdx = self.routesList.index(routeString)
            traci.route.add(str(routeIdx), route)

    def SelectRoutes(self, numberOfRoutes):
        i = 0
        while i < numberOfRoutes:
            routeIdx = self.routesList.index(random.choice(self.routesList))
            self.usedRoutes.append(str(routeIdx))
            self.activeRoutes.append(str(routeIdx))
            if routeIdx not in self.activeRoutes:
                i += 1

    def UpdateRoutes(self, allLanesList):
        i = 0
        lanesSpeedZero = []
        while i < len(allLanesList):
            if traci.lane.getMaxSpeed(allLanesList[i]) <= 0.00:
                lanesSpeedZero.append(allLanesList[i])
            i += 1

        if lanesSpeedZero:
            for laneZero in lanesSpeedZero:
                for routeIdx in self.activeRoutes:
                    routeIndex = int(routeIdx)
                    routeString = self.routesList[routeIndex]
                    route = routeString.split(" ")
                    if laneZero.split("_")[0] in route:
                        while True:
                            newRouteString = random.choice(self.routesList)
                            newRouteId = self.routesList.index(newRouteString)
                            # newRoute = newRouteString.split(" ")
                            if str(newRouteId) is not self.usedRoutes:
                                break
                        # traci.route.add(str(newRouteId), newRoute)
                        self.usedRoutes.append(str(newRouteId))
                        self.activeRoutes.remove(str(routeIndex))
                        self.activeRoutes.append(str(newRouteId))





