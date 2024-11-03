class Zona:
    def __init__(self, nome, tipo_terreno, mant_necessarios, prioridade, janela_tempo, populacao, dificuldade_acesso, atendida):
        self.nome = nome #str
        self.tipo_terreno = tipo_terreno #str
        self.mant_necessarios = mant_necessarios #int
        self.prioridade = prioridade #int (1 a 10)
        self.janela_tempo = janela_tempo #int h
        self.populacao = populacao #int
        self.dificuldade_acesso = dificuldade_acesso #int (0 a 10)
        self.atendida = False
    
    def atualizar_mant_necessarios(self, quantidade):
        self.mant_necessarios = max(0, self.mant_necessarios - quantidade)

        if(self.mant_necessarios == 0):
            self.atendida = True
    
    def avaliar_prioridade(self):
        #pensar a lógica depois
        return 0

    #getters da classe zona
    def get_nome(self):
        return self.nome
    
    def get_tipo_terreno(self):
        return self.tipo_terreno
    
    def get_mant_necessarios(self):
        return self.mant_necessarios
    
    def get_prioridade(self):
        return self.prioridade
    
    def get_janela_tempo(self):
        return self.janela_tempo
    
    def get_populacao(self):
        return self.populacao

    def get_dificuldade_acesso(self):
        return self.dificuldade_acesso
    
    def is_atendida(self):
        return self.atendida

    def __str__(self):
        return (f"Zona {self.nome} (Terreno: {self.tipo_terreno})"
                f"Mantimentos necessários: {self.mant_necessarios}kg"
                f"Prioridade: {self.prioridade}"
                f"Janela de Tempo: {self.janela_tempo} h"
                f"Populacao: {self.populacao} pessoas"
                f"Dificuldade de Acesso: {self.dificuldade_acesso}"
                f"Atentidade: {'Sim' if self.atendidade else 'Não'}")