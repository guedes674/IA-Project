import heapq
import networkx as nx
import matplotlib.pyplot as plt
from queue import PriorityQueue, Queue

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
    def __init__(self, name, id=-1):
        self.m_id = id
        self.m_name = name

    def __str__(self):
        return "node " + self.m_name

    def setId(self, id):
        self.m_id = id

    def getId(self):
        return self.m_id

    def getName(self):
        return self.m_name

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
        self.m_nodes = []
        self.m_directed = directed
        self.m_graph = {}
        self.zone_priorities = {}
        self.vehicle_limitations = {}
        self.condicoes_meteorologicas = {}

    def __str__(self):
        out = ""
        for key in self.m_graph.keys():
            out += "node " + str(key) + ": " + str(self.m_graph[key]) + "\n"
        return out

    def add_edge(self, node1, node2, weight):
        n1 = Node(node1)
        n2 = Node(node2)
        if n1 not in self.m_nodes:
            n1_id = len(self.m_nodes)
            n1.setId(n1_id)
            self.m_nodes.append(n1)
            self.m_graph[node1] = []

        if n2 not in self.m_nodes:
            n2_id = len(self.m_nodes)
            n2.setId(n2_id)
            self.m_nodes.append(n2)
            self.m_graph[node2] = []

        self.m_graph[node1].append((node2, weight))
        if not self.m_directed:
            self.m_graph[node2].append((node1, weight))

    def simulate_distribution(self):
        # Simulação da distribuição para uma rota com A*
        self.desenha()
        #start, goal = "Base", "ZonaE"
        #came_from, cost_so_far = self.m_graph.a_star_search(start, goal, heuristic)

        #path = []
        #current = goal
        #while current != start:
        #    path.append(current)
        #    current = came_from[current]
        #path.append(start)
        #path.reverse()

        #print("Melhor path: ", path)
        #print("Custo total: ", cost_so_far[goal])

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
        if (current, neighbor) in self.condicoes_meteorologicas:
            return base_cost * self.condicoes_meteorologicas[(current, neighbor)]
        return base_cost

    def calculate_cost(self, path):
        custo = 0
        for i in range(len(path) - 1):
            custo += self.get_arc_cost(path[i], path[i + 1])
        return custo

    def heuristic(self,zone1, zone2):
        # Função heurística simulada: pode ser substituída com base na distância geográfica
        return abs(hash(zone1) - hash(zone2)) % 10

    def search_DFS(self, start, end, path=None, visited=None):
        if path is None:
            path = []
        if visited is None:
            visited = set()

        path = path + [start]
        visited.add(start)

        if start == end:
            custoT = self.calculate_cost(path)
            return path, custoT

        for (adjacente, _) in self.m_graph[start]:
            if adjacente not in visited:
                resultado = self.search_DFS(adjacente, end, path, visited)
                if resultado is not None:
                    return resultado
        return None

    def search_BFS(self, start, end, priorities):
        visited = set()
        fila = Queue()
        custo = 0

        fila.put(start)  # (prioridade, nó)
        visited.add(start)

        parent = dict()
        parent[start] = None

        path_found = False
        while not fila.empty() and not path_found:
            nodo_atual = fila.get()
            if nodo_atual == end:
                path_found = True
            else:
                for (adjacente, peso) in self.m_graph[nodo_atual]:
                    if adjacente not in visited:
                        visited.add(adjacente)
                        parent[adjacente] = nodo_atual
                        fila.put(adjacente)

        path = []
        if path_found:
            path.append(end)
            while parent[end] is not None:
                path.append(parent[end])
                end = parent[end]
            path.reverse()
            custo = self.calculate_cost(path)

        return path, custo

    def procura_DLS(self, start, end, limit, prioritys, path=None, visited=None):
        if path is None:
            path = []
        if visited is None:
            visited = set()

        path = path + [start]
        visited.add(start)

        if len(path) > limit:
            return None

        if start == end:
            custoT = self.calculate_cost(path)
            custoT -= prioritys.get(end, 0)
            return (path, custoT)

        for (adjacente, peso) in self.m_graph[start]:
            if adjacente not in visited:
                resultado = self.procura_DLS(adjacente, end, limit, prioritys, path, visited)
                if resultado is not None:
                    return resultado
        return None

    def search_greedy(self, start, end, path=None, visited=None):
        if path is None:
            path = []
        if visited is None:
            visited = set()

        path = path + [start]
        visited.add(start)

        if start == end:
            custoT = self.calculate_cost(path)  # Calcula o custo total com base no caminho encontrado
            return (path, custoT)

        # Criar uma lista prioritária com base na heurística
        adjacentes = []

        for (adjacente, peso) in self.m_graph[start]:
            if adjacente not in visited:
                # Usar a função heurística para calcular a prioridade
                heuristica = self.heuristic(adjacente, end)
                heapq.heappush(adjacentes, (heuristica, adjacente))

        # Escolher o nó com a menor estimativa heurística
        while adjacentes:
            _, proximo_nodo = heapq.heappop(adjacentes)

            resultado = self.search_greedy(proximo_nodo, end, path, visited)
            if resultado is not None:
                return resultado

        return None

    def a_star_search(self, start, goal):
        if start not in self.m_graph:
            raise KeyError(f"O nó inicial '{start}' não está no grafo.")
        if goal not in self.m_graph:
            raise KeyError(f"O nó final '{goal}' não está no grafo.")

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            if current not in self.m_graph:
                raise KeyError(f"O nó '{current}' não está no grafo.")

            for next in self.m_graph[current]:
                if not isinstance(next, tuple) or len(next) != 2:
                    raise ValueError(f"O formato das arestas está incorreto: {next}")

                print(f"Processando nó atual: {current}, Próximo: {next}")

                new_cost = cost_so_far[current] + self.get_arc_cost(current, next[0])
                if next[0] not in cost_so_far or new_cost < cost_so_far[next[0]]:
                    cost_so_far[next[0]] = new_cost
                    priority = new_cost + self.heuristic(next[0], goal)
                    frontier.put(next[0], priority)
                    came_from[next[0]] = current

        # Reconstruir o caminho
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()

        return path, cost_so_far[goal]
    
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