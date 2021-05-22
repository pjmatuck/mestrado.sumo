import numpy as np
import random

EPSILON = 0.1   # police
ALPHA = 0.45     # learning rate
GAMMA = 0.9    # discount factor

class QLearningTable:
    def __init__(self, learning_rate=ALPHA, reward_discount=GAMMA, policy=EPSILON):
        self.lr = ALPHA
        self.reward_discount=GAMMA
        self.policy = EPSILON
        self.q_table = []

    def add_q_table_state(self):
        self.q_table.append([])

    def add_action_to_state(self, stateId, actionId):
        if self.q_table[stateId]:
            for action in self.q_table[stateId]:
                if action[0] == actionId:
                    print("Ação " + str(actionId) + " ja existente no estado " + str(stateId))
                    return

        self.q_table[stateId].append([actionId, 0])
        return

    def get_action_value(self, stateId, actionId):
        for action in self.q_table[stateId]:
            if action[0] == actionId:
                return action[1]

    def set_action_value(self, stateId, actionId, actionValue):
        for action in self.q_table[stateId]:
            if action[0] == actionId:
                action[1] = actionValue
                return

    def get_max_action_value(self, stateId):
        actionMaxValue = -100000
        for action in self.q_table[stateId]:
            if actionMaxValue < action[1]:
                actionMaxValue = action[1]
        return actionMaxValue

    def get_max_action_id(self, stateId):
        actionMaxValue = -100000
        actionMaxId = None
        if self.check_state_exist(stateId):
            if len(self.q_table[stateId]) > 0:
                for action in self.q_table[stateId]:
                    if actionMaxValue < action[1]:
                        actionMaxValue = action[1]
                        actionMaxId = action[0]
                return actionMaxId
            else:
                return None
        else:
            return None

    def add_q_table_item(self, stateId, actionId):
        while stateId > len(self.q_table) - 1:
            self.add_q_table_state()

        #qt_item = qtItem.QTableItem(actionId, 0.0)
        self.add_action_to_state(stateId, actionId)

    def check_state_exist(self, stateId):
        if stateId < len(self.q_table) - 1:
            return True
        return False

    # Q-Learning
    # Q(s,a) <- (1 - alpha) * Q(s,a) + alpha * (R + gamma * maxQ(s_,a))
    def Qlearn(self, s, a, r, s_):
        if self.check_state_exist(s_) and self.q_table[s_] is not []:
            # Calcula o valor da ação para o próximo estado dado o estado anterior
            learned_value = r + self.reward_discount * self.get_max_action_value(s_)
        else:
            learned_value = r
        try:
            action_value = (1 - self.lr) * self.get_action_value(s, a) + self.lr * (learned_value)
            self.set_action_value(s,a,action_value)
        except:
            print("Error")
            print(s)
            print(a)
            print(s_)
            print(self.get_action_value(s,a))

