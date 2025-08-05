#!/usr/bin/env python
import copy
import heapq
import numpy as np
import time
import random

class VacuumCleaner:

    def __init__(self, size: int):
	# CONFIGURATION
        size = size - 1
        self.number_robots = 2 # Number of robots in the grid
        self.init_pos = [[1,size,0], [0,size,3]] # MODIFICABLE. Init position of robots [[rob1_red_x, rob1_red_y, rob1_red_rot], [rob2_green_x, rob2_green_y, rob2_green_rot]] where rot South:0, East:1, North:2, West:3	
        self.size = [size, size] # Size of the grid
        self.obs = [] # Si quieres PONER obstaculos comenta la linea 17 a la 19 ya que hace que se generen aleatoriamente
        tmp = random.randint(1,size-1)
        for i in range(tmp):
            self.obs.append([random.randint(1,size-1), random.randint(1,size-1)])
        self.drain = [[0,size]] # MODIFICABLE Drain position
        self.charge = [[0,0], [size,size]] # MODIFICABLE Charge position for each robot
        self.costs = [[1,0.5,0.5], [1,0.5,0.5]] #  MODIFICABLE Cost for each action and robot
        self.battery = [10, 10] # MODIFICABLE Battery for each robot
        self.vacum = [5, 5] # MODIFICABLE Capacity of the vacuum for each robot
        self.ground = [1, 1] # Ground for each robot
        self.zones = [[1 for _ in range(self.size[0]+1)] for _ in range(self.size[1]+1)]

        for obs_pos in self.obs:
            self.zones[obs_pos[1]][obs_pos[0]] = 2
    
        for i in self.drain:
            x, y = i
            self.zones[x][y] = 3

        for i, charge_pos in enumerate(self.charge):
            self.zones[charge_pos[1]][charge_pos[0]] = 4 if i == 0 else 5

        self.zones[self.init_pos[0][1]][self.init_pos[0][0]] = 6 + (self.init_pos[0][2] / 4)

        self.zones[self.init_pos[1][1]][self.init_pos[1][0]] = 7 + (self.init_pos[1][2] / 4)

        self.angles = [0, 1, 2, 3]
        self.actions = ['Move', 'TurnWest', 'TurnEast']

    def list_to_tuple(self, lst):
        return tuple(self.list_to_tuple(x) if isinstance(x, list) else x for x in lst)

    def tuple_to_list(self, tup):
        return list(self.tuple_to_list(x) if isinstance(x, tuple) else x for x in tup)

    def is_position_invalid(self, state, x, y):
        # Verificar si x e y están dentro de los límites de la matriz
        if x < 0 or y < 0 or x > self.size[0] or y > self.size[1]:
            return True
        # Verificar si la posición es un obstáculo o la posición de un robot
        cell_value = state[y][x]
        return (cell_value == 2 or int(cell_value) == 6 or int(cell_value) == 7)
    
    def get_robots(self, state):
        robots = [[],[]]
        for y in range(len(state)):
            for x in range(len(state[y])):
                cell_value = state[y][x]
                # Check if the cell_value indicates a robot's presence
                if isinstance(cell_value, float) and (int(cell_value) == 6 or int(cell_value) == 7):
                    # Extract the integer part (robot ID) and the decimal part (rotation)
                    robot_id = int(cell_value)
                    rotation_decimal = cell_value - robot_id
                    # Convert the decimal rotation to the desired integer rotation
                    rotation = int(rotation_decimal * 4)
                    
                    # Store the robot's information according to its ID
                    if robot_id == 6:
                        robots[0] = [x, y, rotation]
                    elif robot_id == 7:
                        robots[1] = [x, y, rotation]

        return robots

    # Return a set of valid actions according to the current position of the robot in the grid
    def getValidActions(self, state, battery, vacum, robot_index):
        combo = []
        robots = self.get_robots(state)

        actions = copy.copy(self.actions)
        if battery[robot_index] > 0.5:
            # Current position of the robot i
            posX, posY, theta = robots[robot_index]

            # Compute valid actions for the robot i
            if theta == 0 and self.is_position_invalid(state, posX - 1, posY):
                actions.remove('Move')
            elif theta == 1 and self.is_position_invalid(state, posX, posY + 1):
                actions.remove('Move')
            elif theta == 2 and self.is_position_invalid(state, posX + 1, posY):
                actions.remove('Move')
            elif theta == 3 and self.is_position_invalid(state, posX, posY - 1):
                actions.remove('Move')

            if theta == 0 and self.is_position_invalid(state, posX, posY + 1):
                actions.remove('TurnEast')
            elif theta == 1 and self.is_position_invalid(state, posX + 1, posY):
                actions.remove('TurnEast')
            elif theta == 2 and self.is_position_invalid(state, posX, posY - 1):
                actions.remove('TurnEast')
            elif theta == 3 and self.is_position_invalid(state, posX - 1, posY):
                actions.remove('TurnEast')

            if theta == 0 and self.is_position_invalid(state, posX, posY - 1):
                actions.remove('TurnWest')
            elif theta == 1 and self.is_position_invalid(state, posX - 1, posY):
                actions.remove('TurnWest')
            elif theta == 2 and self.is_position_invalid(state, posX, posY + 1):
                actions.remove('TurnWest')
            elif theta == 3 and self.is_position_invalid(state, posX + 1, posY):
                actions.remove('TurnWest')

        else:
            actions.remove('TurnWest')
            actions.remove('TurnEast')
            actions.remove('Move')

        combo = actions
        
        return combo

    def apply_action(self, state, robot_index, action, ground, battery, vacum):
        robots = self.get_robots(state)
        posXO, posYO, theta = robots[robot_index]
        temp = self.tuple_to_list(state)

        posX = posXO
        posY = posYO

        if action == 'Move':
            if ground[robot_index] == 1 and vacum[robot_index] > 0:
                temp[posYO][posXO] = 0
            else:
                temp[posYO][posXO] = ground[robot_index]

            if theta == 0:
                posX -= 1
            elif theta == 1:
                posY += 1
            elif theta == 2:
                posX += 1
            elif theta == 3:
                posY -= 1
        
        elif action == 'Behind':
            if ground[robot_index] == 1 and vacum[robot_index] > 0:
                temp[posYO][posXO] = 0
            else:
                temp[posYO][posXO] = ground[robot_index]

            if theta == 0:
                posX += 1
            elif theta == 1:
                posY -= 1
            elif theta == 2:
                posX -= 1
            elif theta == 3:
                posY += 1

        elif action == 'TurnWest':
            theta = (theta - 1) % 4

        elif action == 'TurnEast':
            theta = (theta + 1) % 4

        battery[robot_index] -= self.cost_of_action(action)

        if action == 'Move':
            ground[robot_index] = temp[posY][posX]

            vacum[robot_index] -= 1 if ground[robot_index] == 1 else 0

        if ground[robot_index] == 3:
            vacum[robot_index] = self.vacum[robot_index]
        if ground[robot_index] == 4 and robot_index == 0:
            battery[robot_index] = self.battery[robot_index]
        if ground[robot_index] == 5 and robot_index == 1:
            battery[robot_index] = self.battery[robot_index]

        temp[posY][posX] = (6 + robot_index) + (theta / 4)
        
        state = self.list_to_tuple(temp)

        return state, ground, battery, vacum

    def get_successors(self, state, ground, battery, vacum):
        successors = []

        # Obtener las acciones válidas para cada robot
        valid_actions_robot1 = self.getValidActions(state, battery, vacum, 0)
        
        # Generar todos los pares posibles de acciones para ambos robots
        for action_robot1 in valid_actions_robot1:
                # Copiar las variables antes de aplicar las acciones
                ground_copy = ground.copy()
                battery_copy = battery.copy()
                vacum_copy = vacum.copy()
                state_copy = copy.deepcopy(state)

                # Aplicar la acción del robot 1
                state_after_robot1, ground_after_robot1, battery_after_robot1, vacum_after_robot1 = self.apply_action(state_copy, 0, action_robot1, ground_copy, battery_copy, vacum_copy)

                valid_actions_robot2 = self.getValidActions(state_after_robot1, battery_after_robot1, vacum_after_robot1, 1)

                for action_robot2 in valid_actions_robot2:
                    # Copiar las variables antes de aplicar las acciones
                    ground_copy_1 = ground_after_robot1.copy()
                    battery_copy_1 = battery_after_robot1.copy()
                    vacum_copy_1 = vacum_after_robot1.copy()
                    state_copy_1 = copy.deepcopy(state_after_robot1)

                    # Aplicar la acción del robot 2 al estado resultante de la acción del robot 1
                    new_state, nground, nbattery, nvacum = self.apply_action(state_copy_1, 1, action_robot2, ground_copy_1, battery_copy_1, vacum_copy_1)

                    # Añadir el nuevo estado a la lista de sucesores
                    successors.append(((action_robot1, action_robot2), new_state, nground, nbattery, nvacum))

        return successors

    def cost_of_action(self, action):
        if action == 'Move':
            return self.costs[0][0]
        elif action == 'TurnWest':
            return self.costs[0][1]
        elif action == 'TurnEast':
            return self.costs[0][2]
        elif action == 'None':
            return 0

    def heuristic(self, state, battery, vacum, last_actions):
        # Obtener la posición de los robots
        robots = self.get_robots(state)

        # Inicializar la heurística a 0
        h = 0

        # Configuraciones de la heurística
        costo_cambio_direccion = 10
        costo_lejania_carga = 15
        umbral_bateria_regreso = 5  # Cuando la batería es menor que esto, considerar el regreso a la carga

        # Recorrer cada celda del tablero para evaluar la suciedad
        for y, row in enumerate(state):
            for x, cell in enumerate(row):
                if cell == 1:  # Celda sucia
                    dist_min = min(self.distancia_manhattan((x, y), (rx, ry)) for rx, ry, _ in robots)
                    h += dist_min

        # Penalizaciones y bonificaciones
        for i, (robot_x, robot_y, _) in enumerate(robots):
            # Penalizaciones por batería baja y depósito lleno
            h += self.penalizacion_estado_robot(battery[i], vacum[i])

            # Penalización por cambio de dirección
            if last_actions[i] in ['TurnWest', 'TurnEast'] and battery[i] > umbral_bateria_regreso:
                h += costo_cambio_direccion

            # Penalización por estar lejos de la carga cuando la batería está baja
            if battery[i] < umbral_bateria_regreso:
                carga_x, carga_y = self.charge[i]
                dist_carga = self.distancia_manhattan((robot_x, robot_y), (carga_x, carga_y))
                h += costo_lejania_carga * dist_carga

        return h

    def penalizacion_estado_robot(self, bateria, suciedad):
        costo_bateria_baja = 10
        costo_deposito_lleno = 10
        umbral_bateria_baja = 4
        umbral_suciedad_alta = 2

        penalizacion = 0
        if bateria < umbral_bateria_baja:
            penalizacion += costo_bateria_baja
        if suciedad > umbral_suciedad_alta:
            penalizacion += costo_deposito_lleno

        return penalizacion

    def distancia_manhattan(self, punto1, punto2):
        return abs(punto1[0] - punto2[0]) + abs(punto1[1] - punto2[1])

    def executeSearch_1(self):
        initial_state = self.list_to_tuple(self.zones)
        costs = {initial_state: 0}
        pq = [(0, 0, 0, self.zones, [], self.ground, self.battery, self.vacum)]
        node_count = 0  # Inicialización del contador de nodos
        
        # Medimos el tiempo de ejecución
        start_time = time.time()

        while pq:
            # Extraer el estado con el menor valor de f (costo total estimado)
            f, _, g, state, path, ground, battery, vacum = heapq.heappop(pq)

            # Verificar si hemos alcanzado el objetivo
            if all(cell != 1 for row in state for cell in row) and (ground[0] == 4 and ground[1] == 5):
                # Medimos el tiempo de ejecución
                end_time = time.time()
                return state, path, node_count, battery, vacum, (end_time - start_time)

            # Expandir el estado actual
            for moves, new_state, nground, nbattery, nvacum in self.get_successors(state, ground, battery, vacum):
                for move in moves:
                   new_g = g + self.cost_of_action(move)  # Aquí debes definir 'cost_of_action'
                new_h = self.heuristic(new_state, nbattery, nvacum, moves)  # Aquí debes definir 'heuristic'
                new_f = new_g + new_h

                # Si el nuevo estado no se ha visitado o tiene un costo menor, lo procesamos
                if new_state not in costs or new_f < costs[new_state]:
                    node_count += 1  # Incrementar el contador por cada nodo generado
                    costs[new_state] = new_f

                    heapq.heappush(pq, (new_f, new_h, new_g, new_state, path + [moves], nground, nbattery, nvacum))
        
        return None

    def executeSearch_0(self):
        initial_state = self.list_to_tuple(self.zones)
        costs = {initial_state: 0}
        pq = [(0, 0, 0, self.zones, [], self.ground, self.battery, self.vacum)]
        node_count = 0  # Inicialización del contador de nodos
        
        # Medimos el tiempo de ejecución
        start_time = time.time()

        while pq:
            # Extraer el estado con el menor valor de f (costo total estimado)
            f, _, g, state, path, ground, battery, vacum = heapq.heappop(pq)

            # Verificar si hemos alcanzado el objetivo
            if all(cell != 1 for row in state for cell in row) and (ground[0] == 4 and ground[1] == 5):
                # Medimos el tiempo de ejecución
                end_time = time.time()
                return state, path, node_count, battery, vacum, (end_time - start_time)
            
            # Expandir el estado actual
            for moves, new_state, nground, nbattery, nvacum in self.get_successors(state, ground, battery, vacum):
                new_g = g + sum(self.cost_of_action(move) for move in moves)
                new_h = 0 # Aquí debes definir 'heuristic'
                new_f = new_g

                # Si el nuevo estado no se ha visitado o tiene un costo menor, lo procesamos
                if new_state not in costs or new_f < costs[new_state]:
                    node_count += 1  # Incrementar el contador por cada nodo generado
                    costs[new_state] = new_f

                    heapq.heappush(pq, (new_f, new_h, new_g, new_state, path + [moves], nground, nbattery, nvacum))

        # Retornar None si no se encuentra un camino
        
        return None

if __name__ == '__main__':
    p = VacuumCleaner(4) # MODIFICABLE. El numero es el tamaño del mapa

    file_stats = 'statistics.txt'
    file_moves = 'plan.txt'

    a0, b0, c0, d0, e0, f0 = p.executeSearch_0()
    a1, b1, c1, d1, e1, f1 = p.executeSearch_1()

    with open(file_stats, 'w') as file:
        file.write("Estado inicial")
        file.write("\n")
        tmp = np.transpose(p.zones)
        [file.write(str(tmp[i]) + "\n") for i in range(len(tmp))]
        file.write("\n")
        file.write("----------------------")
        file.write("\n")
        file.write("Heuristica 0")
        file.write("\n")
        file.write("Estado final")
        file.write("\n")
        tmp = np.transpose(a0)
        [file.write(str(tmp[i]) + "\n") for i in range(len(tmp))]
        file.write("\n")
        file.write(str(len(b0)) + " numero de acciones")
        file.write("\n")
        file.write(str(c0) + " nodos generados")
        file.write("\n")
        file.write(str(f0.__round__(2)) + " segundos")
        file.write("\n")
        file.write("----------------------")
        file.write("\n")
        file.write("Heuristica 1")
        file.write("\n")
        file.write("Estado final")
        file.write("\n")
        tmp = np.transpose(a1)
        [file.write(str(tmp[i]) + "\n") for i in range(len(tmp))]
        file.write("\n")
        file.write(str(len(b1)) + " numero de acciones")
        file.write("\n")
        file.write(str(c1) + " nodos generados")
        file.write("\n")
        file.write(str(f1.__round__(2)) + " segundos")
        file.write("\n")
    
    with open(file_moves, 'w') as file:
        for i in b1:
            file.write(str(i[0]) + " robot 1 (ROJO); " + str(i[1]) + " robot 2 (VERDE)")
            file.write("\n")