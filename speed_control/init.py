
from __future__ import absolute_import
from __future__ import print_function
from xml.dom import minidom
from action import Action

import os
import sys
import random
import traci
import time
import threading
import reinforcement_learning as rl
import file_manager as fm
# import chart as chart

# Ordem da matriz que representa a malha viária
N_NODES = 3
EPISODES = 50
HORIZON = 250
HORIZON_SIZE = 300
ITERATION = 0
# ITERATION_STEP = 400
OCCUPANCY_RESOLUTION = 5
N_ACTIONS = 3
REWARD = 0
# EDGES_SHOULD_NOT_CHANGE = ['0/0to1/0', '1/0to1/1', '1/1to2/1', '2/1to2/2',
#                           '2/2to3/2', '3/2to3/3', '3/2to3/3']

arrived_vehicles = 0
last_arrived_vehicles = 0
max_number_arrived_veh = 0
states_list = []
actions_list = []
transition_function = []
arrived_vehicles_data = []
lanes_speed = [5.55, 22.22] #In m/s
q_table = rl.QLearningTable()
# data_chart = chart.Chart()

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation"
        " (it should contain folders 'bin', 'tools' and 'docs')")

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


# Realiza o parse do output XML (tripinfo) e calcula o Tempo de Viagem Médio
def get_mean_travel_time(horizon):
    travelTimeSum = 0
    # travelTimeAverage = 0
    xmldoc = minidom.parse("output/tripinfo_" + str(horizon) + ".xml")
    tripInfoList = xmldoc.getElementsByTagName('tripinfo')
    for trip in tripInfoList:
        travelTimeSum += float(trip.attributes['duration'].value)
    travelTimeAverage = travelTimeSum / len(tripInfoList)
    print("Travel Time Average: " + str(round(travelTimeAverage, 2)) + "s")
    return travelTimeAverage

def check_state_open_close_lane(edge, way, state, open_or_close):
    vector_index = convert_edge_to_vector(edge)

    if open_or_close:
        if way == "WE":
            state.WE_roads[vector_index] = 1
        elif way == "EW":
            state.EW_roads[vector_index] = 1
        elif way == "SN":
            state.SN_roads[vector_index] = 1
        elif way == "NS":
            state.NS_roads[vector_index] = 1
    else:
        if way == "WE":
            state.WE_roads[vector_index] = 0
        elif way == "EW":
            state.EW_roads[vector_index] = 0
        elif way == "SN":
            state.SN_roads[vector_index] = 0
        elif way == "NS":
            state.NS_roads[vector_index] = 0

    return state

def check_state_swipe_lane_status(edge, way, state):
    vector_index = convert_edge_to_vector(edge)

    if way == "WE":
        if state.WE_roads[vector_index] == 0:
            state.WE_roads[vector_index] = 1
        else:
            state.WE_roads[vector_index] = 0
    elif way == "EW":
        if state.EW_roads[vector_index] == 0:
            state.EW_roads[vector_index] = 1
        else:
            state.EW_roads[vector_index] = 0
    elif way == "SN":
        if state.SN_roads[vector_index] == 0:
            state.SN_roads[vector_index] = 1
        else:
            state.SN_roads[vector_index] = 0
    elif way == "NS":
        if state.NS_roads[vector_index] == 0:
            state.NS_roads[vector_index] = 1
        else:
            state.NS_roads[vector_index] = 0

    return state

def choose_action(state):
    #Determina randomicamente se abre ou fecha uma via
    n_lanes_to_close = random.randint(1,N_ACTIONS)
    n_lanes_to_open = N_ACTIONS - n_lanes_to_close

    i = 0
    lanes_to_close = []
    while i < n_lanes_to_close:
        generated_edge, way = generate_random_edge()
        if generated_edge not in lanes_to_close: #Garante que não gere duas arestas iguais
            lanes_to_close.append(generated_edge)
        else:
            i -= 1
        check_state_open_close_lane(generated_edge, way, state, 1)
        i += 1

    i = 0
    lanes_to_open = []
    while i < n_lanes_to_open:
        generated_edge, way = generate_random_edge()
        # Garante que não gere duas arestas iguais
        # Ou determina a abertura de uma via que será fechada na mesma ação
        if generated_edge not in lanes_to_open or lanes_to_close:
            lanes_to_open.append(generated_edge)
        else:
            i -= 1
        check_state_open_close_lane(generated_edge, way, state, 0)
        i += 1

    action = Action(len(actions_list), lanes_to_close, lanes_to_open)
    check_action_result = check_action_exists(action)
    if check_action_result is not None:
        action.id = check_action_result
    else:
        actions_list.append(action)
    return action

# Sistema de geração randomica de arestas no formato "a/btoc/d"
def generate_random_edge():
    while True:
        #Set the type and direction to choose the edge
        movement_type = random.randint(0, 1)  # 0 - Horizontal, 1 - Vertical
        movement_direction = random.randint(0, 1)  # 0 - Forward, 1 - Backward

        #start from Origin NODE component X
        node_origin_x = random.randint(0, N_NODES - 1)

        #All the inconsistent situations
        if node_origin_x == 0:
            node_origin_y = random.randint(0, N_NODES - 1)
            movement_direction = 0
            if node_origin_y == N_NODES - 1:
                if movement_type == 0:
                    movement_direction = 0
                else:
                    movement_direction = 1

        elif node_origin_x == N_NODES - 1:
            node_origin_y = random.randint(0, N_NODES - 2)
            if node_origin_y == N_NODES - 2 or movement_type == 0:
                movement_direction = 1
        else:
            node_origin_y = random.randint(0, N_NODES - 1)
            if node_origin_y == N_NODES - 1:
                if movement_type == 1 or node_origin_x == 2:
                    movement_direction = 1

        if node_origin_y == 0 and movement_type == 1:
            movement_direction = 0

        #Destiny NODE
        if movement_type == 0 and movement_direction == 0:
            node_destiny_x = node_origin_x + 1
            node_destiny_y = node_origin_y
            way = "WE"
        elif movement_type == 0 and movement_direction == 1:
            node_destiny_x = node_origin_x - 1
            node_destiny_y = node_origin_y
            way = "EW"
        elif movement_type == 1 and movement_direction == 0:
            node_destiny_x = node_origin_x
            node_destiny_y = node_origin_y + 1
            way = "SN"
        elif movement_type == 1 and movement_direction == 1:
            node_destiny_x = node_origin_x
            node_destiny_y = node_origin_y - 1
            way = "NS"

        edge_name = str(node_origin_x) + "/" + str(node_origin_y) + "to" \
                    + str(node_destiny_x) + "/" + str(node_destiny_y)

        if not validate_edge(edge_name):
            break

    return edge_name, way

# Verifica se o edge pode ser bloqueado ou não
def validate_edge(generated_edge):
    not_valid = 0
    for edge in EDGES_SHOULD_NOT_CHANGE:
        if edge == generated_edge:
            not_valid = 1
            return not_valid
    return not_valid

# Habilita ou desabilita faixa da pista de acordo com a via (edge) e vetor do estado
# isLaneEnable -> 0: Faixa habilitada para uso / 1: Faixa desabilitada para uso
def enable_disable_lane(edge, isLaneEnabled):
    if isLaneEnabled:
        traci.lane.setDisallowed(edge + "_0", ['passenger'])
    else:
        traci.lane.setAllowed(edge + "_0", ['passenger'])

# Configura a malha conforme o estado
def network_setup(state):
    i = 0
    while i < len(state.NS_roads):
        enable_disable_lane(convert_vector_to_edge("NS", i), state.NS_roads[i])
        i += 1

    i = 0
    while i < len(state.SN_roads):
        enable_disable_lane(convert_vector_to_edge("SN", i), state.SN_roads[i])
        i += 1

    i = 0
    while i < len(state.EW_roads):
        enable_disable_lane(convert_vector_to_edge("EW", i), state.EW_roads[i])
        i += 1

    i = 0
    while i < len(state.WE_roads):
        enable_disable_lane(convert_vector_to_edge("WE", i), state.WE_roads[i])
        i += 1

# SISTEMA DE CONVERSÃO DE ARESTA('a/btoc/d') PARA INDICE DO VETOR
def convert_edge_to_vector(edge):
    # a/btoc/d
    a = int(edge[0])
    b = int(edge[2])
    c = int(edge[5])
    d = int(edge[7])

    if c > a:
        way = "WE"
        vector_position = a + b*(N_NODES - 1)
    if a > c:
        way = "EW"
        vector_position = c + d*(N_NODES - 1)
    if d > b:
        way = "SN"
        vector_position = b + a*(N_NODES - 1)
    if b > d:
        way = "NS"
        vector_position = d + c*(N_NODES - 1)

    return vector_position

def check_lane_way(lane):
    # a/btoc/d_0
    a = int(lane[0])
    b = int(lane[2])
    c = int(lane[5])
    d = int(lane[7])

    if c > a:
        way = "WE"
    if a > c:
        way = "EW"
    if d > b:
        way = "SN"
    if b > d:
        way = "NS"

    return way

#Sistema de conversão de vetor para faixa da pista
def convert_vector_to_edge(way, lane_in_vector):
    if way == "WE":
        a = lane_in_vector % (N_NODES - 1)
        b = int(lane_in_vector / (N_NODES - 1))
        c = a + 1
        d = b
    elif way == "EW":
        c = lane_in_vector % (N_NODES - 1)
        d = int(lane_in_vector / (N_NODES - 1))
        a = c + 1
        b = d
    elif way == "SN":
        a = int(lane_in_vector / (N_NODES - 1))
        b = lane_in_vector % (N_NODES - 1)
        c = a
        d = b + 1
    elif way == "NS":
        c = int(lane_in_vector / (N_NODES - 1))
        d = lane_in_vector % (N_NODES - 1)
        a = c
        b = d + 1

    return str(a)+"/"+str(b)+"to"+str(c)+"/"+str(d)

# Verifica se o estado ja existe na lista de estados
def check_state_exists(state):
    for s in states_list:
        if state.WE_roads == s.WE_roads:
            if state.NS_roads == s.NS_roads:
                if state.EW_roads == s.EW_roads:
                    if state.SN_roads == s.SN_roads:
                        return s.id
    return None

# Verifica se as ações ja existem na lista de ações
def check_action_exists(action):
    for a in actions_list:
        if action.closed_roads == a.closed_roads:
            if action.opened_roads == a.opened_roads:
                # if action.tls_status == a.tls_status:
                return a.id
    return None

def get_actions_by_state(stateId, transitionFunction, actionList):
    actionsByState = []
    for tuple in transition_function:
        transition_stateId = tuple[0]
        if transition_stateId == stateId:
            for action in actions_list:
                if tuple[1] == action.id:
                    actionsByState.append(action)
    return actionsByState

def get_q_table_elements_list_by_state(stateId, transitionFunction):
    q_table_elements_list = []
    for tuple in transition_function:
        transition_stateId = tuple[0]
        if transition_stateId == stateId:
            q_table_elements_list.append(tuple[1])
    return q_table_elements_list

def getLanesOccupancy(lanesList):
    lanesOcuppancy = []
    for lane in lanesList:
        lanesOcuppancy.append(traci.lane.getLastStepOccupancy(lane))

        #FOR TESTS
        if traci.lane.getLastStepOccupancy(lane) > 0.95:
            print(lane + str(traci.lane.getLastStepOccupancy(lane)))
        elif traci.lane.getLastStepOccupancy(lane) > 0.75:
            print(lane + str(traci.lane.getLastStepOccupancy(lane)))
        elif traci.lane.getLastStepOccupancy(lane) > 0.5:
            print(lane + str(traci.lane.getLastStepOccupancy(lane)))
    return lanesOcuppancy

def getLanesMaxSpeed(lanesList):
    lanesMaxSpeed = []
    for lane in lanesList:
        # lanesMaxSpeed.append(traci.lane.getMaxSpeed(lane))
        lanesMaxSpeed.append(lanes_speed[1])
        # traci.lane.setMaxSpeed(lane, lanes_speed[1])
    return lanesMaxSpeed

def generate_random_state(lanesList):
    lanesMaxSpeed = []
    for lane in lanesList:
        # lanesMaxSpeed.append(traci.lane.getMaxSpeed(lane))
        speed = lanes_speed[random.randint(0, (len(lanes_speed) - 1))]
        lanesMaxSpeed.append(speed)
        # traci.lane.setMaxSpeed(lane, speed)
    return lanesMaxSpeed

def update_network_lanes_maxspeed(lanesList, maxSpeedLanesList):
    i = 0
    while i < len(lanesList):
        traci.lane.setMaxSpeed(lanesList[i], maxSpeedLanesList[i])
        i += 1
    return

def changeLaneMaxSpeedRandomly(lane):
    changeSpeedMod = random.randint(0,1)
    if changeSpeedMod is 0:
        lane = str(round(float(lane) - 2,2))
    else:
        lane = str(round(float(lane) + 2,2))
    return lane

def updateLanesMaxSpeed(lanesMaxSpeedList, actionsLanesMaxSpeed):
    i = 0
    updatedLanesMaxSpeed = []
    while i < len(lanesMaxSpeedList):
        # newSpeed = round(lanesMaxSpeedList[i] + actionsLanesMaxSpeed[i],2)
        # if newSpeed < 5.5:
        #     newSpeed = 5.5
        # if newSpeed > 21.9:
        #     newSpeed = 21.9
        action = actionsLanesMaxSpeed[i]
        speed_index = lanes_speed.index(lanesMaxSpeedList[i])
        if action == 1 and speed_index < (len(lanes_speed) - 1):
            newSpeed = lanes_speed[speed_index + 1]
        elif action == -1 and speed_index > 0:
            newSpeed = lanes_speed[speed_index - 1]
        else:
            newSpeed = lanesMaxSpeedList[i]
        updatedLanesMaxSpeed.append(newSpeed)
        i += 1
    return updatedLanesMaxSpeed

def getLanesMaxSpeedActions(lanesMaxSpeedListSize):
    lanesMaxSpeedActions = []
    i = 0
    while i < lanesMaxSpeedListSize:
        changeSpeedMod = random.randint(0, 1)
        if changeSpeedMod is 0:
            speed = -2
        else:
            speed = 2
        lanesMaxSpeedActions.append(speed)
        i += 1
    return lanesMaxSpeedActions

def getLanesActionsWithOccupancy(lanesMeanOccupancy):
    i = 0
    lanesMaxSpeedActions = []
    while i < len(lanesMeanOccupancy):
        if lanesMeanOccupancy[i] > 0.5:
            lanesMaxSpeedActions.append(-1)
        elif lanesMeanOccupancy[i] >= 0 and lanesMeanOccupancy[i] <= 0.2:
            lanesMaxSpeedActions.append(1)
        else:
            lanesMaxSpeedActions.append(0)
        i += 1
    return lanesMaxSpeedActions

def get_random_lanes_actions():
    i = 0
    lanesActions = []
    number_of_lanes = (4*(N_NODES**2 - N_NODES))/2
    while i < number_of_lanes:
        lanesActions.append(random.randint(-1,1))
        i += 1
    return lanesActions

def check_element_to_list(element, elementList):
    if element not in elementList:
        elementList.append(element)
    return

def sum_occupancy(occ_list1, occ_list2):
    total_occupancy = []
    if occ_list1 == []:
        return occ_list2
    else:
        i = 0
        while i < len(occ_list1):
            total_occupancy.append(occ_list1[i] + occ_list2[i])
            i += 1
    return total_occupancy

def reward_by_arrived_veh(last_arrived_veh, arrived_veh):
    if arrived_vehicles > last_arrived_vehicles * 1.5:
        reward = 1
    elif arrived_vehicles > last_arrived_vehicles * 1.25:
        reward = 0.75
    elif arrived_vehicles > last_arrived_vehicles * 1.1:
        reward = 0.5
    elif arrived_vehicles >= last_arrived_vehicles:
        reward = 0.25
    else:
        reward = -1
    return reward

def reward_by_max_arrived_veh(arrived_veh, max_arrived_veh):
    if arrived_veh > max_arrived_veh * 0.9:
        reward = 1
    elif arrived_veh > max_arrived_veh * 0.75:
        reward = 0.75
    elif arrived_veh > max_arrived_veh * 0.6:
        reward = 0.5
    else:
        reward = -1
    return reward

# Laço principal de execução da simulação
def run(episode):
    print(time.ctime())
    global ITERATION
    global state, state_, arrived_vehicles, last_arrived_vehicles, max_number_arrived_veh

    # Busca todas as faixas (lanes) da malha
    allLanesList = traci.lane.getIDList()[:(4*(N_NODES**2 - N_NODES))]

    # Define quais faixas deverão ser alteradas.
    # Neste caso: apenas aquelas sentido Sul -> Norte e Oeste -> Leste
    lanesToChange = []
    for lane in allLanesList:
        way = check_lane_way(lane)
        if way == "SN" or way == "WE":
            lanesToChange.append(lane)

    """execute the TraCI control loop"""
    step = 0
    arrived_vehicles = 0
    last_arrived_vehicles = 0
    reward = 0
    ITERATION = 0
    setupFirstState = True
    lanesMeanOccupancy = []
    occupancy_res_iterator = 0

    while step < HORIZON * HORIZON_SIZE:

        # Realiza as configurações do primeiro estado
        if (setupFirstState is True):
            if episode is 0:
                state = getLanesMaxSpeed(lanesToChange)
                arrived_vehicles_data.append((step + (HORIZON * HORIZON_SIZE * episode), 0))
            # else:
                state = generate_random_state(lanesToChange)

            # state = generate_random_state(lanesToChange)

            update_network_lanes_maxspeed(lanesToChange, state)

            # Check if state exists and add to list
            check_element_to_list(state, states_list)

            setupFirstState = False
            print("\nITERATION: " + str(ITERATION) + " | STEP: " + str(step) + " | EPISODE: " + str(episode))

            # chart.chart_data = arrived_vehicles_data

        # Coleta os dados de ocupação das vias de acordo com a resolução configurada
        if step % (HORIZON_SIZE/OCCUPANCY_RESOLUTION) == 0 and step is not 0:
            lanesMeanOccupancy = sum_occupancy(lanesMeanOccupancy, getLanesOccupancy(lanesToChange))

        # Executa os passos a partir do segundo estado (s_)
        if step % HORIZON_SIZE == 0 and step is not 0:
            ITERATION += 1
            # getLanesOccupancy(allLanesList)
            # actualLanesMaxSpeed = updateLanesMaxSpeed(actualLanesMaxSpeed)
            # action = getLanesMaxSpeedActions(len(lanesToChange))

            lanesMeanOccupancy = [x / OCCUPANCY_RESOLUTION for x in lanesMeanOccupancy]

            # Avalia a política e define a ação a ser tomada para alteração de estado
            if random.random() < q_table.policy:
                actionId = q_table.get_max_action_by_state(states_list.index(state))
                if actionId is not None:
                    action = actions_list[actionId]
                else:
                    action = getLanesActionsWithOccupancy(lanesMeanOccupancy)
            else:
                action = get_random_lanes_actions()

            # action = get_random_lanes_actions()

            lanesMeanOccupancy[:] = []

            # Check if action exists and add to list
            check_element_to_list(action, actions_list)

            # Adiciona estado anterior e ação a Q-Table
            q_table.add_q_table_item(states_list.index(state), actions_list.index(action))

            # Define o estado resultado a partir do estado anterior e a ação tomada
            state_ = updateLanesMaxSpeed(state, action)

            update_network_lanes_maxspeed(lanesToChange, state_)

            # Check if state exists and add to list
            check_element_to_list(state_, states_list)

            # Define a função de transição
            transition_function.append(
                [states_list.index(state), actions_list.index(action), states_list.index(state_)])

            # Definição da recompensa baseada no nº de carros no destino
            # if last_arrived_vehicles < arrived_vehicles:
            #     reward = 1
            # else:
            #     reward = -1
            if max_number_arrived_veh < arrived_vehicles:
                max_number_arrived_veh = arrived_vehicles

            # reward = reward_by_arrived_veh(last_arrived_vehicles, arrived_vehicles)
            reward = reward_by_max_arrived_veh(arrived_vehicles, max_number_arrived_veh)

            last_arrived_vehicles = arrived_vehicles
            arrived_vehicles_data.append((step + (HORIZON * HORIZON_SIZE * episode), arrived_vehicles))
            # chart.chart_data = arrived_vehicles_data
            arrived_vehicles = 0

            #Aciona a função de aprendizado da Q-Table
            q_table.Qlearn(states_list.index(state), actions_list.index(action),
                           reward, states_list.index(state_))

            print("\nITERATION: " + str(ITERATION) + " | STEP: " + str(step) + " | EPISODE: " + str(episode))

            state = state_

        # Calcula a quantidade de carros que chegou na origem
        arrived_vehicles += traci.simulation.getArrivedNumber()

        traci.simulationStep()

        # Contador do número de passos da simulação
        step += 1

    traci.close()
    # getLanesOccupancy(allLanesList)
    # getLanesMaxSpeed(allLanesList)


# Define o formato (GUI/Console) de execução da simulação
def get_options():
     options = input("Do you like to run in GUI(0) or Console(1) form? \n")
     return options

def save_simulation_state():
    fm.save_object(states_list, "sl")
    fm.save_object(actions_list, "al")
    fm.save_object(transition_function, "tf")
    fm.save_object(q_table.q_table, "qt")
    return

def load_simulation_state(args_list):
    args_list[0] = fm.load_object("sl")
    args_list[1] = fm.load_object("al")
    args_list[2] = fm.load_object("tf")
    args_list[3] = fm.load_object("qt")
    return

def clean_lists(args_list):
    args_list[0] = []
    args_list[1] = []
    args_list[2] = []
    args_list[3] = []

def generate_output(args_list):
    fm.generate_output_file(args_list[0], "states_list")
    fm.generate_output_file(args_list[1], "actions_list")
    fm.generate_output_file(args_list[2], "transition_function")
    fm.generate_output_file(args_list[3], "q_table")
    fm.generate_output_file(args_list[4], "arrived_veh_data")

# Ponto de entrada do script
if __name__ == "__main__":

    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options == '0':
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo')

    # Laço de execução dos HORIZONTES
    for h in range(0, EPISODES):
        if h > 0:
            clean_lists([states_list,actions_list,transition_function,q_table.q_table])
            load_simulation_state([states_list,actions_list,transition_function,q_table.q_table])
        traci.start([sumoBinary, "-c", "data/speed_control.sumocfg", "-S", "-Q"])
        print("\nEPISODE: " + str(h))
        run(h)
        save_simulation_state()
        # if h == 3:
        #     print("Hello moto crois!")
        # get_mean_travel_time(h)

    generate_output([states_list,actions_list,transition_function,q_table.q_table,arrived_vehicles_data])
    # data_chart.set_chart_data(arrived_vehicles_data)
    # data_chart.draw()

    # wait = input("PRESS ENTER TO CONTINUE.")
