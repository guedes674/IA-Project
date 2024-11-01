import networkx as nx
import zonas

class MapaDistribuicao:

    def __init__(self):
        self.grafo = nx.Graph()
    
    def adicionar_zona(self,zona):
        self.grafo.add_node(zona.nome, dados=zona)
    
    def adicionar_rota(self, origem, destino, distancia, terreno):
        self.grafo.add_edge(origem, destino, distancia, terreno)