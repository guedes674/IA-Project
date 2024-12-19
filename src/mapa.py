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
        self.m_name = str(name)

    def __str__(self):
        return "node " + self.m_name

    def setId(self, id):
        self.m_id = id

    def getId(self):
        return self.m_id

    def getName(self):
        return self.m_name

    def __eq__(self, other):
        return self.m_name == other.m_name

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
            _, current = frontier.get()

            if current == goal:
                break

            for next in self.m_graph[current]:
                new_cost = cost_so_far[current] + self.get_arc_cost(current, next[0])
                if next[0] not in cost_so_far or new_cost < cost_so_far[next[0]]:
                    cost_so_far[next[0]] = new_cost
                    # Prioridade integrada na função de avaliação
                    priority = new_cost + heuristica[next[0]]
                    frontier.put((priority, next[0]))
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
            g.add_node(node)
            for (adjacente, peso) in self.m_graph[node]:
                g.add_edge(node, adjacente, weight=peso)

        pos = nx.spring_layout(g)
        nx.draw_networkx(g, pos, with_labels=True, font_weight='bold')
        labels = nx.get_edge_attributes(g, 'weight')
        nx.draw_networkx_edge_labels(g, pos, edge_labels=labels)
        plt.draw()
        plt.show()

def heuristic(zone1, zone2):
    # Função heurística simulada: pode ser substituída com base na distância geográfica
    return abs(hash(zone1) - hash(zone2)) % 10

# Configuração do g com zonas afetadas
g = Graph(directed=True)
# Adicionar arestas (baseado no exemplo fornecido)
edges = {
    'Health Planet': [('Celeirós', 4.9), ('Real', 4.5), ('Vimieiro', 2.7), ('São Vicente', 5)],
    'Celeirós': [('Panoias', 2.5), ('Pedralva', 2.6), ('Arentim', 3.2), ('São Vitor', 3)],
    # ... Adicione o restante das arestas aqui
}

for node1, edges in edges.items():
    for edge in edges:
        node2, weight = edge
        g.add_edge(node1, node2, weight)

# Adicionar condições meteorológicas dinâmicas
g.condicoes_meteorologicas[('Celeirós', 'Panoias')] = 1.5  # Aumentar o custo devido à tempestade

# Definir veículos
drones = Vehicle("Drone", capacidade=10, autonomia=50, restricoes=["Nenhuma"])
caminhao = Vehicle("Caminhão", capacidade=200, autonomia=500, restricoes=["Terrenos Acidentados"])

# Definir zonas de entrega com prioritys e janelas de tempo
zonas = {
    'Arentim': Zone('Arentim', prioridade=5, janela_tempo=120),
    'Ferreiros': Zone('Ferreiros', prioridade=3, janela_tempo=90),
    # ... Adicione outras zonas
}

# Heurística (estimativa para o objetivo)
heuristica = {
    'Health Planet': 0,
    'Celeirós': 4.9,
    'Panoias': 7.4,
    # ... Complete com os outros nós
}

# Testar busca A*
start = 'Health Planet'
goal = 'Arentim'
path, cost = g.a_star_search(start, goal, heuristica)
print(f"Melhor caminho: {path} com custo: {cost}")

# Desenhar o g
g.desenha()

def simulate_distribution():
    # Simulação da distribuição para uma rota com A*
    start, goal = "Base", "ZonaE"
    came_from, cost_so_far = g.a_star_search(start, goal, heuristic)

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    print("Melhor path: ", path)
    print("Custo total: ", cost_so_far[goal])

simulate_distribution()