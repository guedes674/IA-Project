import networkx as nx
from zonas import Zona

class Mapa:

    def __init__(self):
        self.grafo = nx.Graph()
    
    def adicionar_zona(self,zona):
        self.grafo.add_node(zona.nome, dados=zona)
    
    def adicionar_rota(self, origem, destino, distancia, terreno, bloqueada = False):
        self.grafo.add_edge(origem, destino, distancia, terreno, bloqueada)
    
    def atualizar_condicoes_rota(self, origem, destino, bloqueada):
        if self.grafo.has_edge(origem, destino):
            self.grafo.edges[origem, destino]['bloqueada'] = bloqueada
    
    def zonas_prioritarias(self):
        zonas = [self.grafo.nodes[n]['dados'] for n in self.grafo.nodes]
        return sorted(zonas, key = lambda z: (-z.get_prioridade(), -z.get_populacao()))

    def obter_zona(self, nome):
        return self.grafo.nodes[nome]['dados']
    
    def obter_rota(self, origem, destino):
        if self.grafo.has_edge(origem, destino):
            return self.grafo.edges[origem, destino]
        else:
            return None