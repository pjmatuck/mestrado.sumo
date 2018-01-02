class Action:

    id = None
    closed_roads = []
    opened_roads = []
    tls_status = []

    def __init__(self,ID, CLOSEDROADS, OPENEDROADS):
        self.id = ID
        self.closed_roads = CLOSEDROADS
        self.opened_roads = OPENEDROADS
        # self.tls_status = TLSSTATUS
        return