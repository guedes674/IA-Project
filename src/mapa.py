import heapq
import networkx as nx
import matplotlib.pyplot as plt
from itertools import permutations
from queue import PriorityQueue, Queue
import heapq

node_info = {
    'Porto': {'population': 1500, 'catastrophe_level': 0},
    'Lisboa': {'population': 1500, 'catastrophe_level': 6},
    'Coimbra': {'population': 1000, 'catastrophe_level': 0},
    'Aveiro': {'population': 800, 'catastrophe_level': 2},
    'Viana do Castelo': {'population': 200, 'catastrophe_level': 4},
    'Braga': {'population': 500, 'catastrophe_level': 0},
    'Faro': {'population': 400, 'catastrophe_level': 0},
    'Vila Real': {'population': 100, 'catastrophe_level': 0},
    'Guarda': {'population': 50, 'catastrophe_level': 3},
    'Castelo Branco': {'population': 150, 'catastrophe_level': 1},
    'Leiria': {'population': 250, 'catastrophe_level': 1},
    'Évora': {'population': 100, 'catastrophe_level': 0},
    'Setúbal': {'population': 250, 'catastrophe_level': 0},
    'Bragança': {'population': 50, 'catastrophe_level': 4},
    'Beja': {'population': 30, 'catastrophe_level': 0},
    'Viseu': {'population': 200, 'catastrophe_level': 1},
    'Guimarães': {'population': 200, 'catastrophe_level': 1}
}

class Vehicle:
    def __init__(self, tipo, capacidade, autonomia, restricoes):
        self.tipo = tipo
        self.capacidade = capacidade
        self.autonomia = autonomia
        self.restricoes = restricoes

    def __str__(self):
        return f"Vehicle(tipo={self.tipo}, capacidade={self.capacidade}kg, autonomia={self.autonomia}km)"
    
    def getTipo(self):
        return self.tipo

    def getRestrictions(self):
        return self.restricoes

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
        for (adjacente, peso) in self.m_graph[current]:
            base_cost = peso 
        # Ajustar o custo com base em condições meteorológicas
        return base_cost

    def calculate_cost(self, path):
        custo = 0
        for i in range(len(path) - 1):
            custo += self.get_arc_cost(path[i], path[i + 1])
        return custo

    def real_procura_dfs(self,start,list,vehicle = None):
        final_path = []
        for end in list :
            if end in final_path:
                continue
            partial = self.procura_DFS(start,end)[0]
            if len(final_path)!=0 and partial[0] == final_path[-1]:
                partial.pop(0)
            final_path = final_path + partial
            start=end
        return final_path

    
    def procura_DFS(self, start, end, path=None, visited=None):
        if path is None:
            path = []
        if visited is None:
            visited = set()

        path = path + [start]
        visited.add(start)

        if start == end:
            # calcular o custo do caminho funçao calcula custo.
            custoT = self.calculate_cost(path)
            return (path, custoT)
        print(start)
        for adjacente,custo in self.m_graph[start]:
            if adjacente not in visited:
                resultado = self.procura_DFS(adjacente, end, path, visited)
                if resultado is not None:
                    return resultado
        return None

    def real_procura_bfs(self,start,list,vehicle = None):
        final_path = []
        for end in list :
            if end in final_path:
                continue
            partial = self.procura_BFS(start,end)[0]
            if len(final_path)!=0 and partial[0] == final_path[-1]:
                partial.pop(0)
            final_path = final_path + partial
            start=end
        return final_path


    def procura_BFS(self, start, end):
        # definir nodos visitados para evitar ciclos
        visited = set()
        fila = Queue()
        custo = 0
        # adicionar o nodo inicial à fila e aos visitados
        fila.put(start)
        visited.add(start)

        # garantir que o start node nao tem pais...
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

        # reconstruir o caminho

        path = []
        if path_found:
            path.append(end)
            while parent[end] is not None:
                path.append(parent[end])
                end = parent[end]
            path.reverse()
            # funçao calcula custo caminho
            custo = self.calculate_cost(path)
        return (path, custo)
    
    def real_procura_a_estrela(self,start,list,vehicle = None):
        final_path = []
        for end in list :
            if end in final_path:
                continue
            partial = self.a_star_search(start,end)[0]
            if len(final_path)!=0 and partial[0] == final_path[-1]:
                partial.pop(0)
            final_path = final_path + partial
            start=end
        return final_path

    def a_star_search(self, start, goal):
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
                    priority = new_cost + self.heuristics[next[0]]
                    frontier.put(next[0], priority)
                    came_from[next[0]] = current

        # Reconstruct the path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()

        return path, cost_so_far[goal]

    def real_procura_gulosa(self,start,list,vehicle = None):
        final_path = []
        for end in list :
            if end in final_path:
                continue
            partial = self.procura_gulosa(start,end)[0]
            if len(final_path)!=0 and partial[0] == final_path[-1]:
                partial.pop(0)
            final_path = final_path + partial
            start=end
        return final_path

    def procura_gulosa(self, start, end, path=None, visited=None):
        if path is None:
            path = []
        if visited is None:
            visited = set()

        path = path + [start]
        visited.add(start)

        if start == end:
            # calcular o custo do caminho funçao calcula custo.
            custoT = self.calculate_cost(path)
            return (path, custoT)

        # Criar uma lista prioritária para armazenar os nós adjacentes
        adjacentes = []

        for (adjacente, peso) in self.m_graph[start]:
            if adjacente not in visited:
                # Adicionar o nó adjacente e a estimativa para o final à lista prioritária
                heapq.heappush(adjacentes, (self.heuristics[adjacente], adjacente))

        # Enquanto houver nós na lista prioritária
        while adjacentes:
            # Obter o nó com a menor estimativa
            (_, proximo_nodo) = heapq.heappop(adjacentes)

            resultado = self.procura_gulosa(proximo_nodo, end, path, visited)
            if resultado is not None:
                return resultado

        return None

    def explore_zones(self, start, algorithm=''):
        """
        Função para explorar as zonas com base no algoritmo escolhido pelo usuário.
        """

        # Pegar em todos os nodos com prioridade diferente de 0
        list = []
        for node in self.zone_priorities:
            if self.zone_priorities[node] != 0:
                list.append(node)


        if algorithm == 'a_star':
            print("Usando A*...")
            path = self.real_procura_a_estrela(start,list)
            print(f"Caminho percorrido (A*): {path}")
        elif algorithm == 'greedy':
            print("Usando Greedy...")
            path = self.real_procura_gulosa(start,list)
            print(f"Nós visitados (Greedy): {path}")
        elif algorithm == 'dfs':
            print("Usando DFS...")
            path = self.real_procura_dfs(start,list)
            print(f"Nós visitados (DFS): {path}")
        elif algorithm == 'bfs':
            print("Usando BFS...")
            path = self.real_procura_bfs(start,list)
            print(f"Nós visitados (BFS): {path}")
        else:
            print("Algoritmo inválido.")


    def compare_search_strategies(self, start, vehicle):
        results = {}
        
        # Pegar em todos os nodos com prioridade diferente de 0
        list = []
        for node in self.zone_priorities:
            if self.zone_priorities[node] != 0:
                list.append(node)


        # Executar e coletar resultados
        results['BFS'] = {
            'path': self.real_procura_bfs(start, list, vehicle),
            'description': 'Busca em profundidade com restrições de veículo'
        }
        results['DFS'] = {
            'path': self.real_procura_dfs(start, list, vehicle),
            'description': 'Busca em profundidade com restrições de veículo'
        }
        
        results['A*'] = {
            'path': self.real_procura_a_estrela(start, list, vehicle),
            'description': 'A* modificado com prioridades e restrições'
        }
        
        results['Greedy'] = {
            'path': self.real_procura_gulosa(start, list, vehicle),
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
        number = 0
        for node2 in self.m_graph[node]:
            number +=1
            total_adj_priority += self.zone_priorities[node2[0]]
        return round(self.zone_priorities[node]*0.7 + total_adj_priority/number*0.3,2)

    def initialize_priority(self):
        for node in self.m_nodes:
            self.zone_priorities[node.getName()] = self.calculate_priority(node)

    def calculate_priority(self,node):
        population = node.getPopulation()
        catastrophes_level = node.getCatastropheLevel()
        number_paths = len(self.m_graph[node.getName()])
        return round(population/(number_paths*100)*catastrophes_level,2)