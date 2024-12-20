import heapq
import networkx as nx
import matplotlib.pyplot as plt
from queue import PriorityQueue, Queue

class Vehicle:
    def __init__(self, tipo, capacidade, autonomia, restricoes):
        self.tipo = tipo
        self.capacidade = capacidade
        self.autonomia = autonomia
        self.restricoes = restricoes

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

class Graph:
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

    def get_node_by_name(self, name):
        search_node = Node(name)
        for node in self.m_nodes:
            if node == search_node:
                return node
        return None

    def print_edge(self):
        listaA = ""
        for nodo in self.m_graph.keys():
            for (nodo2, custo) in self.m_graph[nodo]:
                listaA += nodo + " ->" + nodo2 + " custo:" + str(custo) + "\n"
        return listaA

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
        g.desenha()
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

    def search_BFS(self, start, end, prioritys):
        visited = set()
        fila = Queue()
        custo = 0

        fila.put(start)
        visited.add(start)

        parent = dict()
        parent[start] = None

        path_found = False
        while not fila.empty() and path_found == False:
            nodo_atual = fila.get()
            if nodo_atual == end:
                path_found = True
            else:
                for (adjacente, peso) in self.m_graph[nodo_atual]:
                    if adjacente not in visited:
                        fila.put(adjacente)
                        parent[adjacente] = nodo_atual
                        visited.add(adjacente)

        path = []
        if path_found:
            path.append(end)
            while parent[end] is not None:
                path.append(parent[end])
                end = parent[end]
            path.reverse()
            custo = self.calcula_custo(path)
    
        # Ajustar custo final pela prioridade
        custo -= prioritys.get(end, 0)
        return (path, custo)
    
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
            custoT = self.calcula_custo(path)
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
            custoT = self.calcula_custo(path)  # Calcula o custo total com base no caminho encontrado
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

            resultado = self.procura_gulosa(proximo_nodo, end, path, visited)
            if resultado is not None:
                return resultado

        return None

    def a_star_search(self, start, goal, heuristica):
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

            for next in self.m_graph[current]:
                new_cost = cost_so_far[current] + self.get_arc_cost(current, next[0])
                if next[0] not in cost_so_far or new_cost < cost_so_far[next[0]]:
                    cost_so_far[next[0]] = new_cost
                    # Prioridade integrada na função de avaliação
                    priority = new_cost + heuristic(next[0], goal)
                    frontier.put(priority, next[0])
                    came_from[next[0]] = current

        # Reconstruir o caminho
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()

        return path, cost_so_far[goal]

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

    def heuristic(zone1, zone2):
        # Função heurística simulada: pode ser substituída com base na distância geográfica
        return abs(hash(zone1) - hash(zone2)) % 10

g = Graph()
g.add_edge('Porto', 'Braga', 4.9)
g.add_edge('Braga', 'Viana do Castelo', 3.2)
g.add_edge('Viana do Castelo', 'Vila Real', 4.0)
g.add_edge('Vila Real', 'Bragança', 2.5)
g.add_edge('Bragança', 'Viana do Castelo', 3.5)
g.add_edge('Braga', 'Vila Real', 3.5)

# Simular a distribuição
g.simulate_distribution()