class State:

    id = None
    WE_roads = None
    EW_roads = None
    NS_roads = None
    SN_roads = None

    def __init__(self, NETWORK_SIZE, ID):
        self.id = ID
        self.WE_roads = [0] * NETWORK_SIZE * (NETWORK_SIZE - 1)
        self.EW_roads = [0] * NETWORK_SIZE * (NETWORK_SIZE - 1)
        self.NS_roads = [0] * NETWORK_SIZE * (NETWORK_SIZE - 1)
        self.SN_roads = [0] * NETWORK_SIZE * (NETWORK_SIZE - 1)
        return
