import numpy as np
import pandas as pd
import q_table_element as qtItem
import random

EPSILON = 0.7   # police
ALPHA = 0.1     # learning rate
GAMMA = 0.9    # discount factor

class QLearningTable:
    def __init__(self, learning_rate=ALPHA, reward_discount=GAMMA, policy=EPSILON):
        # self.actions = ACTIONS
        self.lr = ALPHA
        self.reward_discount=GAMMA
        self.policy = EPSILON
        # self.q_table = pd.DataFrame()
        self.q_table = [[]]

    # Verifica se o estado existe na Q-Table, caso não, adiciona.
    # def check_q_table_state(self, stateId, actionsByStateList):
    #     if stateId not in self.q_table.index:
    #         self.q_table = self.q_table.append(
    #             pd.Series(
    #                 actionsByStateList,
    #                 index = self.q_table.columns,
    #                 name=stateId
    #             )
    #         )
    # def check_q_table_state(self, stateId, actionsByStateList):
    #     if stateId > len(self.q_table) - 1:
    #         self.q_table.append(actionsByStateList)
    #     else:
    #         self.q_table[stateId] = actionsByStateList

    def add_q_table_state(self):
        self.q_table.append([])

    def add_action_to_state(self, stateId, actionId):
        # if actionId > len(self.q_table[stateId]) - 1:
        #     i = 0
        #     while i <= actionId:
        #         self.q_table[stateId].append(0)
        #         i += 1
        if self.q_table[stateId] is not []:
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

    def get_max_action_value(self, stateId, actionId):
        actionMaxValue = -1
        for action in self.q_table[stateId]:
            if actionMaxValue < action[1]:
                actionMaxValue = action[1]
        return actionMaxValue

    def get_max_action_by_state(self, stateId):
        actionMaxValue = -1
        actionMaxId = None
        if self.check_state_exist(stateId):
            if len(self.q_table[stateId]) > 0:
                for action in self.q_table[stateId]:
                    if actionMaxValue < action[1]:
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

    def choose_best_action(self, stateId):
        best_action_value = -1
        # for item in self.q_table[stateId]:
        #     if best_action_value < qt_item.value:
        #         best_action_value = qt_item.value
        #         qt_item_best_action = qt_item
        # return qt_item_best_action
        return max(self.q_table[stateId])

    def choose_random_action(self, state):
        rand = random.randint(0, len(self.q_table[state.id]))
        return self.q_table[state.id][rand].actionId

    # Escolhe uma ação da Q-Table ou aleatória, de acordo com a política.
    def choose_action(self, state):
        if self.check_state_exist(state.id):
            if np.random.uniform() < self.policy:
                best_action_id = self.choose_best_action(state.id).actionId
            else:
                best_action_id = self.choose_random_action(state.id).actionId
            return best_action_id
        else:
            print("Ocorreu um erro: O estado não se encontra na Q-Table")
            return None

    def get_q_table_item(self, state, action):
        for qt_item in self.q_table[state.id]:
            if qt_item.actionId == action.id:
                return qt_item
        print("Ocorreu um erro: Não existe esta ação para o referente estado na Q-Table")
        return None

    def update_q_table_item(self, state, action, q_target, q_predict):
        for qt_item in self.q_table[state.id]:
            if qt_item.actionId == action.id:
                qt_item.value += self.lr * (q_target - q_predict)

    # Q-Learning
    # Q(s,a) <- (1 - alpha) * Q(s,a) + alpha * (R + gamma * maxQ(s_,a))
    # SARSA
    # Q(s,a) <- Q(s,a) + alpha*[R + gamma*Q(s_,a_) - Q(s,a)]
    # alpha = learning rate, gamma = discount factor
    def SARSAlearn(self, s, a, r, s_):
        # Assume o valor da ação referente ao 's' e 'a' na Q-Table
        q_predict = self.get_q_table_item(s, a).value
        if self.check_state_exist(s_):
            # Calcula o valor da ação para o próximo estado dado o estado anterior
            q_target = r + self.reward_discount * self.choose_best_action(s_).value
            # q_target = r + self.gamma * self.q_table.ix[s_, :].max()  # next state is not terminal
            # q_target = r  # next state is terminal
        else:
            q_target = r
        # self.q_table.ix[s, a] += self.lr * (q_target - q_predict)  # update
        #Update Q-Table Item
        for qt_item in self.q_table[s.id]:
            if qt_item.actionId == a.id:
                qt_item.value += self.lr * (q_target - q_predict)

    # Q-Learning
    # Q(s,a) <- (1 - alpha) * Q(s,a) + alpha * (R + gamma * maxQ(s_,a))
    def Qlearn(self, s, a, r, s_):
        # Assume o valor da ação referente ao 's' e 'a' na Q-Table
        # q_predict = self.get_q_table_item(s, a).value
        # q_predict = self.q_table[s][a]
        q_predict = self.get_action_value(s, a)
        if self.check_state_exist(s_) and self.q_table[s_] is not []:
            # Calcula o valor da ação para o próximo estado dado o estado anterior
            q_target = r + self.reward_discount * self.get_max_action_value(s_, a)
        else:
            q_target = r

        # for qt_item in self.q_table[s.id]:
        #     if qt_item.actionId == a.id:
        #         qt_item.value = (1 - self.lr)* qt_item.value +\
        #                         self.lr * (q_target - q_predict)
        # self.q_table[s][a] = (1 - self.lr) * self.q_table[s][a] + self.lr * (q_target - q_predict)
        action_value =  (1 - self.lr) * self.get_action_value(s,a) + self.lr * (q_target - q_predict)
        self.set_action_value(s,a,action_value)
