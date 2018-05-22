import traci
import random
import route_manager as route

class VehicleManager:

    vehID = 0
    routes = route.RouteManager()

    def __init__(self):

        return

    def AddVehicleToTraci(self, numberOfCars):
        i = 0
        while i < numberOfCars:
            traci.vehicle.addFull(str(self.vehID), random.choice(self.routes.activeRoutes), departSpeed='random')
            traci.vehicle.setColor(str(self.vehID), (str(random.randint(0,255)),str(random.randint(0,255)),str(random.randint(0,255))))
            self.vehID += 1
            i += 1