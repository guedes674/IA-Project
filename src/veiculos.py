class Veiculo:
    #Contrutor do tipo de classe Veículo
    def __init__(self, tipo, capacidade_max_carga, autonomia_max, autonomia_atual, velocidade, terrenos_possiveis):
        self.tipo = tipo #str
        self.capacidade_max_carga = capacidade_max_carga #float
        self.carga_atual = 0 #float
        self.autonomia_max = autonomia_max #float
        self.autonomia_atual = autonomia_atual #float
        self.velocidade = velocidade #float (km/h)
        self.terrenos_possiveis = terrenos_possiveis # list
        self.localizacao_atual = None
    
    def pode_acessar(self,zona):
        return zona in self.acessibilidade
    
    def calcular_tempo_viagem(self, distancia):
        if self.velocidade > 0:
            return distancia / self.velocidade
        else:
            print("veículo parado.")
            return float('inf')
    
    def pode_realizar_viagem(self, distancia):
        return self.autonomia > distancia
    
    def reabastece_combustivel(self):
        self.autonomia_atual = self.autonomia_max
        print(f"{self.tipo} foi reabastecido.")

    def carregar(self, quantidade):
        if self.carga_atual + quantidade <= self.capacidade_max_carga:
            self.carga_atual += quantidade
            print(f"{self.tipo} carregou {quantidade}kg de mantimentos.")
            return True
        else:
            print(f"{self.tipo} não tem capacidade suficiente para carregar os mantimentos.")
            return False
    
    def descarregar(self):
        print(f"{self.tipo} descarregou {self.carga_atual}kg de mantimentos.")
        self.carga_atual = 0

    def __str__(self):
        return (f"{self.tipo} - Capacidade de Carga: {self.capacidade_max_carga} kg, "
                f"Capacidade Ocupada: {self.carga_atual} kg, "
                f"Autonomia Máxima: {self.autonomia_max} km, "
                f"Autonomia Atual: {self.autonomia_atual} km, "
                f"Velocidade: {self.velocidade} km/h, "
                f"Terrenos Acessíveis: {', '.join(self.terrenos_possiveis)}")