from veiculos import Veiculo
from zonas import Zona
from mapa import Mapa
import networkx as nx

def main():
    mapa = Mapa()

    zona1 = Zona("Braga", tipo_terreno="urbano", mant_necessarios=100, prioridade=5, janela_tempo=24, populacao=2000, dificuldade_acesso=3)
    zona2 = Zona("Porto", tipo_terreno="urbano", mant_necessarios=70, prioridade=3, janela_tempo=40, populacao=4000, dificuldade_acesso=2)

    mapa.adicionar_zona(zona1)
    mapa.adicionar_zona(zona2)

    mapa.adicionar_rota("Braga", "Porto", distancia=10, terreno="rural")

if __name__ == "__main__":
    main()