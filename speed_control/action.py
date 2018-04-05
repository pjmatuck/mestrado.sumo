class Action:

    def GenerateActionsFromLanes(self, lanesList):
        actions = []
        action_rise = []
        action_decrease = []

        for lane in lanesList:
            way, x1, y1 = self.CheckLaneStatus(lane)
            if way == "WE":
                for lane_ in lanesList:
                    a1,b1,a2,b2 = self.DecompoundLane(lane_)
                    if b1 == b2 == y1 and a2 > a1:
                        action_rise.append(1)
                        action_decrease.append(-1)
                    else:
                        action_rise.append(0)
                        action_decrease.append(0)
            elif way == "EW":
                for lane_ in lanesList:
                    a1,b1,a2,b2 = self.DecompoundLane(lane_)
                    if b1 == b2 == y1 and a1 > a2:
                        action_rise.append(1)
                        action_decrease.append(-1)
                    else:
                        action_rise.append(0)
                        action_decrease.append(0)
            elif way == "SN":
                for lane_ in lanesList:
                    a1,b1,a2,b2 = self.DecompoundLane(lane_)
                    if a1 == a2 == x1 and b2 > b1:
                        action_rise.append(1)
                        action_decrease.append(-1)
                    else:
                        action_rise.append(0)
                        action_decrease.append(0)
            elif way == "NS":
                for lane_ in lanesList:
                    a1,b1,a2,b2 = self.DecompoundLane(lane_)
                    if a1 == a2 == x1 and b1 > b2:
                        action_rise.append(1)
                        action_decrease.append(-1)
                    else:
                        action_rise.append(0)
                        action_decrease.append(0)
            if action_rise not in actions:
                actions.append(action_rise)
            if action_decrease not in actions:
                actions.append(action_decrease)
            action_rise = []
            action_decrease = []

        # Insert action = [0,0,0....0]
        for i in lanesList:
            action_rise.append(0)
        actions.append(action_rise)

        return actions

    def CheckLaneStatus(self, lane):
        # x1/y1tox2/y2_0

        x1, y1, x2, y2 = self.DecompoundLane(lane)

        if x2 > x1:
            way = "WE"
        if x1 > x2:
            way = "EW"
        if y2 > y1:
            way = "SN"
        if y1 > y2:
            way = "NS"

        return way, x1, y1

    def DecompoundLane(self, lane):
        x1 = int(lane[0])
        y1 = int(lane[2])
        x2 = int(lane[5])
        y2 = int(lane[7])
        return x1, y1, x2, y2


