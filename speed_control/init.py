
from __future__ import absolute_import
from __future__ import print_function
from xml.dom import minidom

import os
import sys
import random
import traci
import time
import reinforcement_learning as rl
import file_manager as fm
import route_manager as rm
import action as act

## Parametros de configuração da simulação
N_NODES = 5
LANES_NUMBER = 4*(N_NODES**2 - N_NODES)
EPISODES = 100
HORIZON = 100
HORIZON_SIZE = 300
ITERATION = 0
OCCUPANCY_RESOLUTION = 5
REWARD = 0
EXPLORATION_RATE = 30 # in %
LANES_SHOULD_NOT_CHANGE = ["0/0to0/1_0","0/0to1/0_0"]
ROUTE_MODE = "Taz"

arrived_vehicles = 0
total_arrived_veh = 0
sum_arrived_veh = []
collisions = 0
sum_collisions = []
avg_lane_speed = 0
sum_avg_lane_speed = []

departed_vehicles = 0
last_arrived_vehicles = 0
max_number_arrived_veh = 0
states_list = []
actions_list = []
transition_function = []
arrived_vehicles_data = []
lanes_speed = [0.00, 8.33, 16.66] #In m/s
q_table = rl.QLearningTable()
routeManager = rm.RouteManager()
actions = act.Action()
preset_action_list = []

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

# def check_state_open_close_lane(edge, way, state, open_or_close):
#     vector_index = convert_edge_to_vector(edge)
#
#     if open_or_close:
#         if way == "WE":
#             state.WE_roads[vector_index] = 1
#         elif way == "EW":
#             state.EW_roads[vector_index] = 1
#         elif way == "SN":
#             state.SN_roads[vector_index] = 1
#         elif way == "NS":
#             state.NS_roads[vector_index] = 1
#     else:
#         if way == "WE":
#             state.WE_roads[vector_index] = 0
#         elif way == "EW":
#             state.EW_roads[vector_index] = 0
#         elif way == "SN":
#             state.SN_roads[vector_index] = 0
#         elif way == "NS":
#             state.NS_roads[vector_index] = 0
#
#     return state
#
# def check_state_swipe_lane_status(edge, way, state):
#     vector_index = convert_edge_to_vector(edge)
#
#     if way == "WE":
#         if state.WE_roads[vector_index] == 0:
#             state.WE_roads[vector_index] = 1
#         else:
#             state.WE_roads[vector_index] = 0
#     elif way == "EW":
#         if state.EW_roads[vector_index] == 0:
#             state.EW_roads[vector_index] = 1
#         else:
#             state.EW_roads[vector_index] = 0
#     elif way == "SN":
#         if state.SN_roads[vector_index] == 0:
#             state.SN_roads[vector_index] = 1
#         else:
#             state.SN_roads[vector_index] = 0
#     elif way == "NS":
#         if state.NS_roads[vector_index] == 0:
#             state.NS_roads[vector_index] = 1
#         else:
#             state.NS_roads[vector_index] = 0
#
#     return state

# Sistema de geração randomica de arestas no formato "a/btoc/d"
# def generate_random_edge():
#     while True:
#         #Set the type and direction to choose the edge
#         movement_type = random.randint(0, 1)  # 0 - Horizontal, 1 - Vertical
#         movement_direction = random.randint(0, 1)  # 0 - Forward, 1 - Backward
#
#         #start from Origin NODE component X
#         node_origin_x = random.randint(0, N_NODES - 1)
#
#         #All the inconsistent situations
#         if node_origin_x == 0:
#             node_origin_y = random.randint(0, N_NODES - 1)
#             movement_direction = 0
#             if node_origin_y == N_NODES - 1:
#                 if movement_type == 0:
#                     movement_direction = 0
#                 else:
#                     movement_direction = 1
#
#         elif node_origin_x == N_NODES - 1:
#             node_origin_y = random.randint(0, N_NODES - 2)
#             if node_origin_y == N_NODES - 2 or movement_type == 0:
#                 movement_direction = 1
#         else:
#             node_origin_y = random.randint(0, N_NODES - 1)
#             if node_origin_y == N_NODES - 1:
#                 if movement_type == 1 or node_origin_x == 2:
#                     movement_direction = 1
#
#         if node_origin_y == 0 and movement_type == 1:
#             movement_direction = 0
#
#         #Destiny NODE
#         if movement_type == 0 and movement_direction == 0:
#             node_destiny_x = node_origin_x + 1
#             node_destiny_y = node_origin_y
#             way = "WE"
#         elif movement_type == 0 and movement_direction == 1:
#             node_destiny_x = node_origin_x - 1
#             node_destiny_y = node_origin_y
#             way = "EW"
#         elif movement_type == 1 and movement_direction == 0:
#             node_destiny_x = node_origin_x
#             node_destiny_y = node_origin_y + 1
#             way = "SN"
#         elif movement_type == 1 and movement_direction == 1:
#             node_destiny_x = node_origin_x
#             node_destiny_y = node_origin_y - 1
#             way = "NS"
#
#         edge_name = str(node_origin_x) + "/" + str(node_origin_y) + "to" \
#                     + str(node_destiny_x) + "/" + str(node_destiny_y)
#
#         if not validate_edge(edge_name):
#             break
#
#     return edge_name, way

# Verifica se o edge pode ser bloqueado ou não
# def validate_edge(generated_edge):
#     not_valid = 0
#     for edge in EDGES_SHOULD_NOT_CHANGE:
#         if edge == generated_edge:
#             not_valid = 1
#             return not_valid
#     return not_valid

# Habilita ou desabilita faixa da pista de acordo com a via (edge) e vetor do estado
# isLaneEnable -> 0: Faixa habilitada para uso / 1: Faixa desabilitada para uso
def enable_disable_lane(edge, isLaneEnabled):
    if isLaneEnabled:
        traci.lane.setDisallowed(edge + "_0", ['passenger'])
    else:
        traci.lane.setAllowed(edge + "_0", ['passenger'])

# Configura a malha conforme o estado
# def network_setup(state):
#     i = 0
#     while i < len(state.NS_roads):
#         enable_disable_lane(convert_vector_to_edge("NS", i), state.NS_roads[i])
#         i += 1
#
#     i = 0
#     while i < len(state.SN_roads):
#         enable_disable_lane(convert_vector_to_edge("SN", i), state.SN_roads[i])
#         i += 1
#
#     i = 0
#     while i < len(state.EW_roads):
#         enable_disable_lane(convert_vector_to_edge("EW", i), state.EW_roads[i])
#         i += 1
#
#     i = 0
#     while i < len(state.WE_roads):
#         enable_disable_lane(convert_vector_to_edge("WE", i), state.WE_roads[i])
#         i += 1

# SISTEMA DE CONVERSÃO DE ARESTA('a/btoc/d') PARA INDICE DO VETOR
# def convert_edge_to_vector(edge):
#     # a/btoc/d
#     a = int(edge[0])
#     b = int(edge[2])
#     c = int(edge[5])
#     d = int(edge[7])
#
#     if c > a:
#         way = "WE"
#         vector_position = a + b*(N_NODES - 1)
#     if a > c:
#         way = "EW"
#         vector_position = c + d*(N_NODES - 1)
#     if d > b:
#         way = "SN"
#         vector_position = b + a*(N_NODES - 1)
#     if b > d:
#         way = "NS"
#         vector_position = d + c*(N_NODES - 1)
#
#     return vector_position

# def check_lane_way(lane):
#     # a/btoc/d_0
#     a = int(lane[0])
#     b = int(lane[2])
#     c = int(lane[5])
#     d = int(lane[7])
#
#     if c > a:
#         way = "WE"
#     if a > c:
#         way = "EW"
#     if d > b:
#         way = "SN"
#     if b > d:
#         way = "NS"
#
#     return way

#Sistema de conversão de vetor para faixa da pista
# def convert_vector_to_edge(way, lane_in_vector):
#     if way == "WE":
#         a = lane_in_vector % (N_NODES - 1)
#         b = int(lane_in_vector / (N_NODES - 1))
#         c = a + 1
#         d = b
#     elif way == "EW":
#         c = lane_in_vector % (N_NODES - 1)
#         d = int(lane_in_vector / (N_NODES - 1))
#         a = c + 1
#         b = d
#     elif way == "SN":
#         a = int(lane_in_vector / (N_NODES - 1))
#         b = lane_in_vector % (N_NODES - 1)
#         c = a
#         d = b + 1
#     elif way == "NS":
#         c = int(lane_in_vector / (N_NODES - 1))
#         d = lane_in_vector % (N_NODES - 1)
#         a = c
#         b = d + 1
#
#     return str(a)+"/"+str(b)+"to"+str(c)+"/"+str(d)

# Verifica se o estado ja existe na lista de estados
# def check_state_exists(state):
#     for s in states_list:
#         if state.WE_roads == s.WE_roads:
#             if state.NS_roads == s.NS_roads:
#                 if state.EW_roads == s.EW_roads:
#                     if state.SN_roads == s.SN_roads:
#                         return s.id
#     return None

# Verifica se as ações ja existem na lista de ações
# def check_action_exists(action):
#     for a in actions_list:
#         if action.closed_roads == a.closed_roads:
#             if action.opened_roads == a.opened_roads:
#                 # if action.tls_status == a.tls_status:
#                 return a.id
#     return None

# def get_actions_by_state(stateId, transitionFunction, actionList):
#     actionsByState = []
#     for tuple in transition_function:
#         transition_stateId = tuple[0]
#         if transition_stateId == stateId:
#             for action in actions_list:
#                 if tuple[1] == action.id:
#                     actionsByState.append(action)
#     return actionsByState

# def get_q_table_elements_list_by_state(stateId, transitionFunction):
#     q_table_elements_list = []
#     for tuple in transition_function:
#         transition_stateId = tuple[0]
#         if transition_stateId == stateId:
#             q_table_elements_list.append(tuple[1])
#     return q_table_elements_list

def getLanesOccupancy(lanesList):
    lanesOcuppancy = []
    for lane in lanesList:
        lanesOcuppancy.append(traci.lane.getLastStepOccupancy(lane))
    return lanesOcuppancy

def getLanesMaxSpeed(lanesList):
    lanesMaxSpeed = []
    for lane in lanesList:
        lanesMaxSpeed.append(lanes_speed[int(len(lanes_speed)/2)])
    return lanesMaxSpeed

# def generate_random_state(lanesList):
#     lanesMaxSpeed = []
#     for lane in lanesList:
#         # lanesMaxSpeed.append(traci.lane.getMaxSpeed(lane))
#         speed = lanes_speed[random.randint(0, (len(lanes_speed) - 1))]
#         lanesMaxSpeed.append(speed)
#         # traci.lane.setMaxSpeed(lane, speed)
#     return lanesMaxSpeed

# def get_random_state(state):
#     action = random.choice(preset_action_list)
#     return updateLanesMaxSpeed(state, action)

def update_network_lanes_maxspeed(lanesList, maxSpeedLanesList):
    i = 0
    while i < len(lanesList):
        if lanesList[i] not in LANES_SHOULD_NOT_CHANGE:
            if maxSpeedLanesList[i] <= 0.00:
                traci.lane.setDisallowed(lanesList[i], "passenger")
            else:
                traci.lane.setAllowed(lanesList[i], "passenger")
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

# def getLanesMaxSpeedActions(lanesMaxSpeedListSize):
#     lanesMaxSpeedActions = []
#     i = 0
#     while i < lanesMaxSpeedListSize:
#         changeSpeedMod = random.randint(0, 1)
#         if changeSpeedMod is 0:
#             speed = -2
#         else:
#             speed = 2
#         lanesMaxSpeedActions.append(speed)
#         i += 1
#     return lanesMaxSpeedActions

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

# def get_random_lanes_actions():
#     i = 0
#     lanesActions = []
#     number_of_lanes = (4*(N_NODES**2 - N_NODES))/2
#     while i < number_of_lanes:
#         lanesActions.append(random.randint(-1,1))
#         i += 1
#     return lanesActions

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

def reward_by_arrived_veh(arrived_veh):
    # if arrived_veh == 0:
    #     reward = - 100
    # else:
    reward = arrived_veh * 2
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

def reward_by_delta_arrived_departed_veh(arrived_veh, departed_veh):
    if departed_vehicles - arrived_vehicles <= 0:
        reward = 1
    else:
        reward = 0
    return reward
    return arrived_vehicles - departed_vehicles

def get_random_action():
    return random.choice(preset_action_list)

def get_edge_travel_time():
    edges_list = traci.edge.getIDList()[:(4*(N_NODES**2 - N_NODES))]
    for ed in edges_list:
        print(traci.edge.getTraveltime(ed))

def get_avg_lanes_speed(lanesList):
    avg_speed = 0
    for lane in lanesList:
        avg_speed += float(traci.lane.getMaxSpeed(lane))
    avg_speed = avg_speed/len(lanesList)
    return avg_speed

def choose_action(action):
    # Avalia a política e define a ação a ser tomada para alteração de estado
    if random.random() < rl.EPSILON:
        actionId = q_table.get_max_action_id(states_list.index(state))
        if actionId is not None:
            action = preset_action_list[actionId]
        else:
            # action = getLanesActionsWithOccupancy(lanesMeanOccupancy)
            action = get_random_action()
    else:
        # action = get_random_lanes_actions()
        action = get_random_action()
    return action

def config():
    if ROUTE_MODE is "Taz":
        routeManager.GenerateRouteFileWithTaz(30000,N_NODES)
    elif ROUTE_MODE is "Route":
        routeManager.GenerateRouteFileWithRouteDistribution(30000)

# Laço principal de execução da simulação
def run(episode):
    print(time.ctime())
    global ITERATION
    global state, state_, preset_action_list
    global total_arrived_veh, collisions, avg_lane_speed


    # Busca todas as faixas (lanes) da malha
    allLanesList = traci.lane.getIDList()[:LANES_NUMBER]
    allEdgesList = traci.edge.getIDList()[:LANES_NUMBER]
    allNodesList = traci.junction.getIDList()[:N_NODES**2]
    allTLSList = traci.trafficlight.getIDList()

    # preset_action_list = actions.GenerateActionsByLanes(allLanesList)
    preset_action_list = actions.GenerateActionsByLanes(allLanesList)

    routeManager.Initialize(allNodesList, allEdgesList, N_NODES)

    # Define quais faixas deverão ser alteradas.
    # Neste caso: apenas aquelas sentido Sul -> Norte e Oeste -> Leste
    lanesToChange = allLanesList
    # for lane in allLanesList:
    #     way = check_lane_way(lane)
    #     if way == "SN" or way == "WE":
    #         lanesToChange.append(lane)
    #     else:
    #         traci.lane.setMaxSpeed(lane, 0.00)

    """execute the TraCI control loop"""
    step = 0
    departed_vehicles = 0
    arrived_vehicles = 0
    colliding_vehicles = 0
    last_arrived_vehicles = 0
    reward = 0
    ITERATION = 0
    setupFirstState = True
    lanesMeanOccupancy = []
    occupancy_res_iterator = 0
    action = []

    while step < HORIZON * HORIZON_SIZE:

        #1. Realiza as configurações do primeiro estado
        if (setupFirstState is True):

            state = getLanesMaxSpeed(lanesToChange)
            # arrived_vehicles_data.append((step + (HORIZON * HORIZON_SIZE * episode), 0))
            # state = get_random_state(state)
            # state = generate_random_state(lanesToChange)
            # update_network_lanes_maxspeed(lanesToChange, state)
            # Check if state exists and add to list
            ## Adiciona estado na lista de estados
            check_element_to_list(state, states_list)

            setupFirstState = False


        print("\nITERATION: " + str(ITERATION) + " | STEP: " + str(step) + " | EPISODE: " + str(episode))

        #2. Define a ação a ser executada
        action = choose_action(action)

        ## Adiciona estado anterior e ação a Q-Table
        q_table.add_q_table_item(states_list.index(state), preset_action_list.index(action))

        #3. Define o próximo estado a partir da ação no passo 2
        state_ = updateLanesMaxSpeed(state, action)

        ## Atualiza malha
        update_network_lanes_maxspeed(lanesToChange, state_)

        ## Adiciona estado na lista de estados
        check_element_to_list(state_, states_list)

        ## Define a função de transição T(s,a,s_)
        transition_function.append(
            [states_list.index(state), preset_action_list.index(action), states_list.index(state_)])

        arrived_vehicles = 0
        departed_vehicles = 0

        #4. Executa simulação por HORIZON_SIZE passos
        while(True):
            traci.simulationStep()
            step += 1

            arrived_vehicles += traci.simulation.getArrivedNumber()
            total_arrived_veh += traci.simulation.getArrivedNumber()
            departed_vehicles += traci.simulation.getDepartedNumber()
            colliding_vehicles += traci.simulation.getCollidingVehiclesNumber()
            collisions += traci.simulation.getCollidingVehiclesNumber()

            if step % HORIZON_SIZE == 0:
                break

        #5. Define recompensa baseada no nº de carros no destino do estado: state
        reward = reward_by_arrived_veh(arrived_vehicles)
        # if colliding_vehicles > 0:
        #     reward += -100 * (colliding_vehicles/2)
        # colliding_vehicles = 0

        avg_lane_speed += get_avg_lanes_speed(allLanesList)

        arrived_vehicles_data.append((step + (HORIZON * HORIZON_SIZE * episode), arrived_vehicles))

        #6. Atualiza tabela Q
        q_table.Qlearn(states_list.index(state), preset_action_list.index(action),
                       reward, states_list.index(state_))

        #7. Define próximo estado como estado atual para repetir o loop
        state = state_

        ITERATION += 1

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
    fm.generate_output_file(args_list[5], "sum_arrived_veh")
    fm.generate_output_file(args_list[6], "collisions")
    fm.generate_output_file(args_list[7], "avg_speed")

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
        config()
        if h is 0:
            initial_epsilon = rl.EPSILON
        if h > 0:
            clean_lists([states_list,actions_list,transition_function,q_table.q_table])
            load_simulation_state([states_list,actions_list,transition_function,q_table.q_table])
        traci.start([sumoBinary, "-c", "src/speed_control.sumocfg", "-S", "-Q"])
        print("\nEPISODE: " + str(h))
        run(h)
        save_simulation_state()
        if rl.EPSILON < 0.98:
            rl.EPSILON += (1 - initial_epsilon)/(EPISODES/(100/EXPLORATION_RATE))
        sum_arrived_veh.append(total_arrived_veh)
        sum_collisions.append(collisions)
        sum_avg_lane_speed.append(avg_lane_speed/HORIZON)
        total_arrived_veh = 0
        collisions = 0
        avg_lane_speed = 0


    generate_output([states_list,actions_list,transition_function,
                     q_table.q_table,arrived_vehicles_data, sum_arrived_veh,
                     sum_collisions, sum_avg_lane_speed])

    # wait = input("PRESS ENTER TO CONTINUE.")
