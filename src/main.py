from mapa import Mapa, Vehicle
import networkx as nx
import matplotlib.pyplot as plt

def main():
    mapa = Mapa()  # Instância do mapa
    
    mapa.zone_priorities = {'Porto': 5, 'Braga': 2, 'Lisboa': 8, 'Faro': 1}

    # Adicionar nós e arestas representando cidades e distâncias aproximadas
    # Norte
    mapa.add_edge('Porto', 'Braga', 4.9)
    mapa.add_edge('Porto', 'Viana do Castelo', 6.0)
    mapa.add_edge('Porto', 'Vila Real', 8.0)
    mapa.add_edge('Braga', 'Viana do Castelo', 3.2)
    mapa.add_edge('Braga', 'Vila Real', 3.5)
    mapa.add_edge('Viana do Castelo', 'Vila Real', 4.0)
    mapa.add_edge('Vila Real', 'Bragança', 2.5)
    mapa.add_edge('Bragança', 'Viana do Castelo', 3.5)

    # Centro
    mapa.add_edge('Porto', 'Aveiro', 4.5)
    mapa.add_edge('Aveiro', 'Coimbra', 4.3)
    mapa.add_edge('Coimbra', 'Leiria', 3.8)
    mapa.add_edge('Leiria', 'Lisboa', 7.0)
    mapa.add_edge('Coimbra', 'Castelo Branco', 6.5)
    mapa.add_edge('Castelo Branco', 'Guarda', 3.7)
    mapa.add_edge('Guarda', 'Vila Real', 5.0)
    mapa.add_edge('Castelo Branco', 'Leiria', 5.2)

    # Sul
    mapa.add_edge('Lisboa', 'Setúbal', 3.0)
    mapa.add_edge('Setúbal', 'Évora', 4.5)
    mapa.add_edge('Évora', 'Beja', 3.5)
    mapa.add_edge('Beja', 'Faro', 5.0)
    mapa.add_edge('Lisboa', 'Évora', 5.8)

    # Ligações adicionais para tornar o grafo mais interconectado
    mapa.add_edge('Aveiro', 'Viseu', 4.0)
    mapa.add_edge('Viseu', 'Guarda', 4.5)
    mapa.add_edge('Viseu', 'Coimbra', 4.2)
    mapa.add_edge('Vila Real', 'Guarda', 5.0)
    mapa.add_edge('Braga', 'Guimarães', 2.5)
    mapa.add_edge('Guimarães', 'Porto', 2.8)
    mapa.add_edge('Guarda', 'Castelo Branco', 3.7)
    mapa.add_edge('Évora', 'Lisboa', 5.0)

    # Algumas rotas diretas mais rápidas
    mapa.add_edge('Braga', 'Lisboa', 18.0)
    mapa.add_edge('Viana do Castelo', 'Lisboa', 19.0)

    while True:
        print("\n--------------------Menu---------------------")
        print("1 - Funcionalidades")
        print("2 - Configurar Prioridades e Restrições")
        print("3 - Comparar Estratégias")
        print("4 - Sair")
        print("---------------------------------------------")

        opcao = int(input("Digite a opção desejada: "))
        if opcao == 1:
            funcionalidades(mapa)
        elif opcao == 2:
            configurar_prioridades_restricoes(mapa)
        elif opcao == 3:
            comparar_estrategias(mapa)
        elif opcao == 4:
            break

def funcionalidades(mapa):
    while True:
        print("--------------------Funcionalidades---------------------")
        print("1 - Exibir mapa")
        print("2 - Testar algoritmos de busca")
        print("3 - Voltar")
        print("-------------------------------------------------------")

        opcao = int(input("Digite a funcionalidade desejada: "))
        
        if opcao == 1:
            exibir_mapa(mapa)
        elif opcao == 2:
            explorar_zonas(mapa)
        elif opcao == 3:
            break

def exibir_mapa(mapa):
    mapa.desenha()  # Usa a função desenha() definida anteriormente para mostrar o grafo

def explorar_zonas(mapa):
    start = input("Digite o nó inicial: ")
    priorities = {'Porto': 5, 'Braga': 2, 'Lisboa': 8, 'Faro': 1}
    algorithm = input("Digite o algoritmo a ser utilizado (a_star, greedy, dfs, bfs): ")
    mapa.explore_zones(start, priorities, algorithm)

def configurar_prioridades_restricoes(mapa):
    while True:
        print("\n--------------------Configurações---------------------")
        print("1 - Definir prioridade de zona")
        print("2 - Configurar restrições de veículo")
        print("3 - Visualizar configurações atuais")
        print("4 - Voltar")
        print("----------------------------------------------------")
        
        opcao = int(input("Digite a opção desejada: "))
        
        if opcao == 1:
            zona = input("Digite o nome da zona: ")
            prioridade = int(input("Digite a prioridade (1-10): "))
            mapa.set_zone_priority(zona, prioridade)
            print(f"Prioridade definida: {zona} -> {prioridade}")
            
        elif opcao == 2:
            zona = input("Digite o nome da zona: ")
            print("\nTipos de veículos disponíveis:")
            print("1 - Caminhão Grande")
            print("2 - Van")
            print("3 - Carro")
            tipo = int(input("Escolha o tipo de veículo restrito: "))
            
            tipos = {1: "caminhao_grande", 2: "van", 3: "carro"}
            if tipo in tipos:
                mapa.set_vehicle_limitation(zona, [tipos[tipo]])
                print(f"Restrição definida: {zona} não permite {tipos[tipo]}")
                
        elif opcao == 3:
            print("\nPrioridades das Zonas:")
            for zona, prioridade in mapa.zone_priorities.items():
                print(f"{zona}: {prioridade}")
                
            print("\nRestrições de Veículos:")
            for zona, veiculos in mapa.vehicle_limitations.items():
                print(f"{zona}: {', '.join(veiculos)}")
                
        elif opcao == 4:
            break

def comparar_estrategias(mapa):
    print("\n--------------------Comparação de Estratégias---------------------")
    
    # Criar veículo
    print("\nConfigurações do Veículo:")
    print("1 - Caminhão Grande (alta capacidade, baixa mobilidade)")
    print("2 - Van (média capacidade, média mobilidade)")
    print("3 - Carro (baixa capacidade, alta mobilidade)")
    
    tipo_veiculo = int(input("Escolha o tipo de veículo: "))
    
    veiculos = {
        1: Vehicle("caminhao_grande", 2000, 80, []),
        2: Vehicle("van", 1000, 100, []),
        3: Vehicle("carro", 500, 120, [])
    }
    
    if tipo_veiculo not in veiculos:
        print("Tipo de veículo inválido")
        return
        
    veiculo = veiculos[tipo_veiculo]
    
    # Definir pontos
    start = input("Digite o ponto de partida: ")
    end = input("Digite o ponto de destino: ")
    
    # Executar comparação
    try:
        resultados = mapa.compare_search_strategies(start, end, veiculo)
        mapa.analyze_results(resultados)
    except Exception as e:
        print(f"Erro ao comparar estratégias: {e}")

if __name__ == "__main__":
    main()