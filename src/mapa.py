import heapq
#import networkx as nx
#import matplotlib.pyplot as plt
from queue import PriorityQueue, Queue

node_info = {
    'Porto': {'population': 1500, 'catastrophe_level': 2},
    'Lisboa': {'population': 1500, 'catastrophe_level': 6},
    'Coimbra': {'population': 1000, 'catastrophe_level': 1},
    'Aveiro': {'population': 800, 'catastrophe_level': 2},
    'Viana do Castelo': {'population': 200, 'catastrophe_level': 4},
    'Braga': {'population': 500, 'catastrophe_level': 0},
    'Faro': {'population': 400, 'catastrophe_level': 3},
    'Vila Real': {'population': 100, 'catastrophe_level': 2},
    'Guarda': {'population': 50, 'catastrophe_level': 3},
    'Castelo Branco': {'population': 150, 'catastrophe_level': 1},
    'Leiria': {'population': 250, 'catastrophe_level': 1},
    'Évora': {'population': 100, 'catastrophe_level': 3},
    'Santarém': {'population': 150, 'catastrophe_level': 2},
    'Setúbal': {'population': 250, 'catastrophe_level': 0},
    'Bragança': {'population': 50, 'catastrophe_level': 4},
    'Portalegre': {'population': 100, 'catastrophe_level': 0},
    'Beja': {'population': 30, 'catastrophe_level': 2},
    'Viseu': {'population': 200, 'catastrophe_level': 1},
    'Guimarães': {'population': 200, 'catastrophe_level': 1}
}

class Vehicle:
    def __init__(self, tipo, capacidade, autonomia, restricoes):
        self.tipo = tipo
        self.capacidade = capacidade  # em kg
        self.autonomia = autonomia    # em km
        self.restricoes = restricoes  # lista de zonas restritas

    def __str__(self):
        return f"Vehicle(tipo={self.tipo}, capacidade={self.capacidade}kg, autonomia={self.autonomia}km)"

    def pode_acessar(self, zona):
        return zona not in self.restricoes

class Zone:
    def __init__(self, nome, prioridade, janela_tempo):
        self.nome = nome
        self.prioridade = prioridade
        self.janela_tempo = janela_tempo

class Node:
    def __init__(self, name, population=0, catastrophe_level=0):
        self.m_name = name
        self.population = population
        self.catastrophe_level = catastrophe_level

    def __str__(self):
        return "node " + self.m_name

    def getName(self):
        return self.m_name
    
    def getPopulation(self):
        return self.population
    
    def getCatastropheLevel(self):
        return self.catastrophe_level
    
    def setPopulation(self, population):
        self.population = population
        
    def setCatastropheLevel(self, catastrophe_level):
        self.catastrophe_level = catastrophe_level

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.m_name == other.m_name
        return False

    def __hash__(self):
        return hash(self.m_name)

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class Mapa:
    def __init__(self, directed=False):
        self.m_directed = directed
        self.m_nodes = []
        self.m_graph = {}
        self.zone_priorities = {}
        self.vehicle_limitations = {}
        self.metereologic_conditions = {}
        self.disconnected_edges = []
        self.heuristics = {}

    def __str__(self):
        out = ""
        for key in self.m_graph.keys():
            out += "node " + str(key) + ": " + str(self.m_graph[key]) + "\n"
        return out

    def add_edge(self, name1, name2, weight):
        n1 = Node(name1)
        n2 = Node(name2)
        if n1 not in self.m_nodes:
            n1.setPopulation(node_info[name1]['population'])
            n1.setCatastropheLevel(node_info[name1]['catastrophe_level'])
            self.m_graph[name1] = []
            self.m_nodes.append(n1)

        if n2 not in self.m_nodes:
            n2.setPopulation(node_info[name2]['population'])
            n2.setCatastropheLevel(node_info[name2]['catastrophe_level'])
            self.m_graph[name2] = []
            self.m_nodes.append(n2)

        self.m_graph[name1].append((name2, weight))
        if not self.m_directed:
            self.m_graph[name2].append((name1, weight))

    def delete_edge(self, node1, node2):
        if node1 in self.m_graph:
            self.m_graph[node1] = [(adj, peso) for (adj, peso) in self.m_graph[node1] if adj != node2]
        if node2 in self.m_graph:
            self.m_graph[node2] = [(adj, peso) for (adj, peso) in self.m_graph[node2] if adj != node1]
            
    def add_priorities_to_queue(self, priorities):
        """
        Adiciona os nós à PriorityQueue com base nas suas prioridades.
        """
        priority_queue = PriorityQueue()
        for node, priority in priorities.items():
            priority_queue.put(node, priority)
        return priority_queue

    def simulate_distribution(self):
        # Simulação da distribuição para uma rota com A*
        self.desenha()
        start, goal = "Viana do Castelo", "Faro"
        came_from, cost_so_far = self.m_graph.a_star_search(start, goal)
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        print("Melhor path: ", path)
        print("Custo total: ", cost_so_far[goal])

    def getNodes(self):
        return self.m_nodes

    def get_zone_priority(self, zone):
        return self.zone_priorities[zone]

    def set_zone_priority(self, zone, priority):
        self.zone_priorities[zone] = priority

    def set_vehicle_limitation(self, zone, vehicles):
        self.vehicle_limitations[zone] = vehicles

    def get_arc_cost(self, current, neighbor):
        base_cost = next((peso for (adjacente, peso) in self.m_graph[current] if adjacente == neighbor), float('inf'))
        # Ajustar o custo com base em condições meteorológicas
        return base_cost

    def calculate_cost(self, path):
        custo = 0
        for i in range(len(path) - 1):
            custo += self.get_arc_cost(path[i], path[i + 1])
        return custo

    def a_star(self, start, node_list):
        visited = set()
        came_from = {}
        g_score = {node: float('inf') for node in self.m_graph}
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.m_graph}
        f_score[start] = self.heuristics[start]

        current_node = start
        while len(node_list)!=0:
            if current_node in node_list:
                node_list.remove(current_node)
                visited.add(current_node)
            for neighbor in self.m_graph[current_node]:
                if neighbor in visited:
                    continue
                tentative_g_score = g_score[current_node] + self.get_arc_cost(current_node, neighbor)
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristics[neighbor]

        return self.reconstruct_path(came_from, start)
    
    def procura_aStar(self, start, list):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        
        while not frontier.empty():
            current = frontier.get()
            if current in list:
                list.remove(current)
            
            if len(list)==0:
                break
            
            for next in self.m_graph[current]:
                new_cost = cost_so_far[current] + self.get_arc_cost(current, next[0])
                if next[0] not in cost_so_far or new_cost < cost_so_far[next[0]]:
                    cost_so_far[next[0]] = new_cost
                    priority = new_cost + self.heuristics[next[0]]
                    frontier.put(next[0], priority)
                    came_from[next[0]] = current
                last = current

        # Reconstruct the path
        path = []
        current = last
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()

        return path

    def reconstruct_path(self, came_from, start):
        total_path = []
        current_node = start
        while current_node in came_from:
            total_path.append(current_node)
            current_node = came_from[current_node]
        total_path.append(current_node)
        return total_path[::-1]

    def heuristic_with_priority(self, node, priorities):
        """
        Heurística que leva em consideração as prioridades dos vizinhos.
        """
        adj_priority_cost = 0
        for neighbor in self.m_graph[node]:
            adj_priority_cost += priorities.get(neighbor, 0)  # Prioridade maior a
        return adj_priority_cost

    def greedy(self, start):
        """
        Implementação da busca Greedy (prioriza apenas a heurística, não o custo real).
        """
        open_list = PriorityQueue()
        open_list.put(start, self.heuristic_with_priority(start))

        came_from = {}
        visited = set()
        
        while not open_list.empty():
            current_node = open_list.get()
            
            if current_node in visited:
                continue
            visited.add(current_node)

            # Expande os vizinhos, priorizando os de maior heurística
            for neighbor, cost in self.m_graph[current_node]:
                if neighbor not in came_from:
                    came_from[neighbor] = current_node
                    open_list.put(neighbor, self.heuristic_with_priority(neighbor, priorities))

        # Reconstruir o caminho
        path = []
        current = start
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]

    def dfs(self, start):
        """
        Implementação da busca em profundidade (DFS).
        """
        stack = [start]
        came_from = {start: None}
        visited = set()

        while stack:
            current_node = stack.pop()
            
            if current_node in visited:
                continue
            visited.add(current_node)

            # Expande os vizinhos, adicionando-os ao topo da pilha
            for neighbor, cost in self.m_graph[current_node]:
                if neighbor not in came_from:
                    came_from[neighbor] = current_node
                    stack.append(neighbor)
                    
        # Reconstruir o caminho
        path = []
        current = start
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]

    def bfs(self, start):
        """
        Implementação da busca em largura (BFS).
        """
        queue = [start]
        came_from = {start: None}
        visited = set()

        while queue:
            current_node = queue.pop(0)
            
            if current_node in visited:
                continue
            visited.add(current_node)

            # Expande os vizinhos, adicionando-os à fila
            for neighbor, cost in self.m_graph[current_node]:
                if neighbor not in came_from:
                    came_from[neighbor] = current_node
                    queue.append(neighbor)

        # Reconstruir o caminho
        path = []
        current = start
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]

    def explore_zones(self, start, algorithm=''):
        """
        Função para explorar as zonas com base no algoritmo escolhido pelo usuário.
        """
        list = []
        for node in self.zone_priorities:
            if self.zone_priorities[node] != 0:
                list.append(node)
        if algorithm == 'a_star':
            print("Usando A*...")
            path = self.procura_aStar(start,list)
            print(f"Caminho percorrido (A*): {path}")
        elif algorithm == 'greedy':
            print("Usando Greedy...")
            #path = self.greedy(start)
            print(f"Nós visitados (Greedy): {path}")
        elif algorithm == 'dfs':
            print("Usando DFS...")
            path = self.dfs(start)
            print(f"Nós visitados (DFS): {path}")
        elif algorithm == 'bfs':
            print("Usando BFS...")
            path = self.bfs(start)
            print(f"Nós visitados (BFS): {path}")
        else:
            print("Algoritmo inválido.")
    
    def compare_search_strategies(self, start, goal, vehicle):
        results = {}
        
        # 1. Busca não informada - DFS com restrições
        def dfs_with_constraints(start, goal, vehicle, visited=None, path=None):
            if visited is None:
                visited = set()
            if path is None:
                path = []
                
            path = path + [start]
            visited.add(start)
            
            if start == goal:
                return path
                
            for next_node, weight in self.m_graph[start]:
                # Verificar restrições do veículo
                if (next_node not in visited and 
                    next_node not in self.vehicle_limitations.get(vehicle.tipo, []) and
                    weight <= vehicle.autonomia):
                    new_path = dfs_with_constraints(next_node, goal, vehicle, visited, path)
                    if new_path:
                        return new_path
            return None
        
        # 2. Busca informada - A* com prioridades e restrições
        def modified_a_star(start, goal, vehicle):
            frontier = PriorityQueue()
            frontier.put(start, 0)
            came_from = {start: None}
            cost_so_far = {start: 0}
            
            while not frontier.empty():
                current = frontier.get()
                
                if current == goal:
                    break
                    
                for next_node, weight in self.m_graph[current]:
                    # Verificar restrições do veículo
                    if (next_node in self.vehicle_limitations.get(vehicle.tipo, []) or
                        weight > vehicle.autonomia):
                        continue
                        
                    new_cost = cost_so_far[current] + weight
                    # Ajustar custo baseado na prioridade da zona
                    priority_bonus = self.zone_priorities.get(next_node, 0)
                    new_cost -= priority_bonus  # Zonas com maior prioridade têm custo reduzido
                    
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(next_node, goal)
                        frontier.put(next_node, priority)
                        came_from[next_node] = current
            
            # Reconstruir caminho
            path = []
            current = goal
            while current is not None:
                path.append(current)
                current = came_from.get(current)
            path.reverse()
            return path if path[0] == start else None
        
        # 3. Busca gulosa com prioridades
        def greedy_with_priorities(start, goal, vehicle):
            visited = set()
            path = []
            current = start
            
            while current != goal:
                path.append(current)
                visited.add(current)
                
                best_next = None
                best_score = float('inf')
                
                for next_node, weight in self.m_graph[current]:
                    if (next_node not in visited and 
                        next_node not in self.vehicle_limitations.get(vehicle.tipo, []) and
                        weight <= vehicle.autonomia):
                        # Combinar heurística com prioridade da zona
                        score = self.heuristic(next_node, goal) - self.zone_priorities.get(next_node, 0)
                        if score < best_score:
                            best_score = score
                            best_next = next_node
                
                if best_next is None:
                    return None
                current = best_next
                
            path.append(goal)
            return path
        
        # Executar e coletar resultados
        results['DFS'] = {
            'path': dfs_with_constraints(start, goal, vehicle),
            'description': 'Busca em profundidade com restrições de veículo'
        }
        
        results['A*'] = {
            'path': modified_a_star(start, goal, vehicle),
            'description': 'A* modificado com prioridades e restrições'
        }
        
        results['Greedy'] = {
            'path': greedy_with_priorities(start, goal, vehicle),
            'description': 'Busca gulosa com prioridades e restrições'
        }
        
        # Calcular e adicionar custos para cada caminho encontrado
        for strategy, result in results.items():
            if result['path']:
                cost = self.calculate_cost(result['path'])
                result['cost'] = cost
                # Ajustar custo final considerando prioridades das zonas visitadas
                priority_bonus = sum(self.zone_priorities.get(node, 0) for node in result['path'])
                result['adjusted_cost'] = cost - priority_bonus
            else:
                result['cost'] = float('inf')
                result['adjusted_cost'] = float('inf')
                
        return results

    def analyze_results(self, results):
        print("\nAnálise comparativa das estratégias de busca:")
        print("-" * 50)
        
        for strategy, result in results.items():
            print(f"\nEstratégia: {strategy}")
            print(f"Descrição: {result['description']}")
            if result['path']:
                print(f"Caminho encontrado: {' -> '.join(result['path'])}")
                print(f"Custo total: {result['cost']}")
                print(f"Custo ajustado (com prioridades): {result['adjusted_cost']}")
            else:
                print("Nenhum caminho válido encontrado")

    def desenha(self):
        g = nx.Graph()
        for node in self.m_nodes:
            # Adicionar o nó ao grafo, assumindo que node.m_name é o nome do nó
            g.add_node(node.m_name)
            for (adjacente, peso) in self.m_graph[node.m_name]:  # Acesse pelo nome
                g.add_edge(node.m_name, adjacente, weight=peso)

        # Configuração do layout e desenho do grafo
        pos = nx.spring_layout(g)
        nx.draw_networkx(g, pos, with_labels=True, font_weight='bold')
        labels = nx.get_edge_attributes(g, 'weight')
        nx.draw_networkx_edge_labels(g, pos, edge_labels=labels)
        plt.draw()
        plt.show()

    def initialize_heuristics(self):
        for node in self.m_graph.keys():
            self.heuristics[node] = self.calculate_heuristic(node)

    def calculate_heuristic(self, node):
        # Replace this with the actual heuristic calculation logic
        total_adj_priority = 0
        for node2 in self.m_graph[node]:
            total_adj_priority += self.zone_priorities[node2[0]] * 0.1
        return self.zone_priorities[node]*0.5 + total_adj_priority

    def initialize_priority(self):
        for node in self.m_nodes:
            self.zone_priorities[node.getName()] = self.calculate_priority(node)

    def calculate_priority(self,node):
        population = node.getPopulation()
        catastrophes_level = node.getCatastropheLevel()
        number_paths = len(self.m_graph[node.getName()])
        return population/(number_paths*10)*catastrophes_level