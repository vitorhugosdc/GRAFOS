class Vertice():
    def __init__(self, indice):
        self.indice = indice
        self.adj = []
        self.visitado = False
        
        self.pai = None ##criado dps
        self.distancia = float('inf') ##criado dps

        # bfs
        #self.bfs_pai = None

        # busca_em_profundidade   
        #self.dfs_pai = None

        # bellmanFord
        #self.bf_pai = None
        #self.bf_distancia = float('inf')

        # dijkstra
        #self.dj_pai = None
        #self.dj_distancia = float('inf')

        # prim 
        #self.pm_pai = None
        #self.pm_distancia = float('inf')

        # ford_fulkerson
        #self.pai = None


    def __str__(self):
        return f'indice: {self.indice}'

    def addAresta(self, distancia, peso):
        self.adj.append(Aresta(distancia, peso))

    def aresta_do_vertice(self, distancia): ##encontra aresta do vértice
        for aresta in self.adj:
            if aresta.distancia == distancia:
                return aresta
    def getArestaMenorCusto(self):
        menor = self.adj[0]
        for aresta in self.adj:
            if aresta.peso < menor.peso:
                menor = aresta
        return menor

class Aresta():
    def __init__(self, distancia, peso, **kwargs):
        self.distancia = distancia
        self.peso = peso
        self.capacidade = kwargs.get('capacidade', float('inf'))
        self.flow = 0

def imprime_dist_pred(distancia,predecessores,tamanho):
    for i in range(tamanho): ##printa arvore de distancias
        for j in range(tamanho): ##colocar algum if distancia[i][j] == float('inf'), printa outra coisa pra sair certinho
            print(f'{distancia[i][j]}\t', end='')
        print()
    print()
    for i in range(tamanho): #printa arvore de predecessores
        for j in range(tamanho):
            print(f'{predecessores[i][j]}\t', end='')
        print()
    print() 


def busca_em_largura(g, pai): ##########     BFS ############################################################################### MEXIDO
       
    Q = []
    Q.append(pai)

    print(f'Sequencia: {pai.indice} ', end='')

    while len(Q) != 0 :
        pai = Q.pop(0) # remove o pai           
        for aresta in pai.adj: #pra cada aresta adj ao pai
            if aresta.distancia.visitado == False: #se ainda não foi visitada
                aresta.distancia.pai = pai
                Q.append(aresta.distancia) #coloca na fila
                aresta.distancia.visitado = True
                print(f'- {aresta.distancia.indice} ', end='')
        if len(Q) == 0:
            for vertice in g:
                if vertice.visitado == False:
                    Q.append(vertice)
                    break
        pai.visitado = True # marca como visitado
    print('\n')

def busca_em_profundidade(g, pai): ########## DFS ############################################################################### MEXIDO

    Q = []
    Q.append(pai)

    print(f'Sequencia: {pai.indice} ', end = '')

    while len(Q) != 0:
        pai = Q.pop(0)          
        for aresta in pai.adj:
            if aresta.distancia.visitado == False:
                Q = [aresta.distancia] + Q
                aresta.distancia.pai = pai
                aresta.distancia.visitado = True
                print(f'- {aresta.distancia.indice} ', end='')
        if len(Q) == 0:
            for vertice in g:
                if vertice.visitado == False:
                    Q.append(vertice)
                    break
        pai.visitado = True
    print('\n')


def relaxamento(u, vertice, peso): ############################################################################### MEXIDO
    if  vertice.distancia > u.distancia + peso:
        vertice.distancia = u.distancia + peso
        vertice.pai = u

def bellman_ford(vertices, pai): ############################################################################### MEXIDO
    for vertice in vertices[:-1]: #relaxa todos os vértices do grafo
        for aresta in vertice.adj:
            relaxamento(vertice, aresta.distancia, vertice.aresta_do_vertice(aresta.distancia).peso)
    for vertice in vertices:
        print(f'{vertice.indice} - ', end='')
        if vertice.distancia != float("inf"):
            print(vertice.distancia)
        else: #caso ainda tenha distancia com valor infinito
            print(f'Nao ha caminho! Pai: {pai.indice}')
    print('\n')

def dijkstra(vertices, pai): 
    Q = []
    Q.append(pai)
    while len(Q) != 0:
        extraido = Q.pop(0)
        extraido.visitado = True
        for aresta in extraido.adj:
            if aresta.distancia.visitado == False:
                Q.append(aresta.distancia)
                aresta.distancia.visitado = True
            relaxamento(extraido, aresta.distancia, aresta.peso)
    for vertice in vertices:
        print(f'{vertice.indice} - ', end='')
        if vertice.distancia != float("inf"):
            print(vertice.distancia)
        else: #caso ainda tenha distancia com valor infinito
            print(f'Nao ha caminho! Pai: {pai.indice}')
    print('\n')
# nao funciona com grafo nao orientado com valores negativos
# mas funciona com grafo orientado com valores negativos
def floyd_warshall(vertices):
    distancias = [[0 for i in range(len(vertices))]for j in range(len(vertices))] #cria a arvore de distâncias
    predecessores = [[0 for i in range(len(vertices))]for j in range(len(vertices))]   #cria a arvore de predecessores
    for i in range(0, len(vertices)): ##preenche as matrizes com infinito
        for j in range(0, len(vertices)):
            distancias[i][j] = float('inf')
            predecessores[i][j] = float('inf')            
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i == j:
                distancias[i][j] = 0 #se i for = j o peso será 0
            elif vertices[i].aresta_do_vertice(vertices[j]):
                distancias[i][j] = vertices[i].aresta_do_vertice(vertices[j]).peso ##distancia recebe o peso da aresta
            predecessores[i][j] = i #a arvore de predecessores recebe i
    for i in range(len(vertices)): #busca o menor caminho e atualiza a lista de distância e predecessores
        for j in range(len(vertices)):
            for k in range(len(vertices)): 
                if distancias[i][j] > distancias[i][k] + distancias[k][j]: #funciona como um "relaxamento" só que da arvore
                    distancias[i][j] = distancias[i][k] + distancias[k][j]
                    predecessores[i][j] = predecessores[k][j]
    imprime_dist_pred(distancias,predecessores,len(vertices)) #imprime as duas matrizes

def printa_prim(l): #imprimir a prim
    Q = []
    for i in range(len(l)):
        if l[i].indice not in Q:
            print(f'{l[i].indice} ', end='')
            Q.append(l[i].indice)

def prim(vertices):
    arvore = [0 for i in range(len(vertices))]
    arvore[0] = True    
    saida = []
    for i in range(len(vertices)-1): #todos os vértices
        chave, pai , b = float('inf'), 0 , 0
        for j in range(len(vertices)):
            if arvore[j] == True:
                for k in range(len(vertices)):
                    if not arvore[k] and vertices[j].aresta_do_vertice(vertices[k]): #se arvore[k] for falso e tiver a aresta
                        if vertices[j].aresta_do_vertice(vertices[k]).peso < chave: #se a aresta do vértice tiver peso menor que a chave
                            chave = vertices[j].aresta_do_vertice(vertices[k]).peso #a chave passa a valer o novo peso
                            pai = j
                            b = k
        arvore[b] = True
        saida.extend([vertices[pai],vertices[b]])
        print(f'{vertices[pai].indice} - {vertices[b].indice}')
    printa_prim(saida)
    print('\n')

def busca_em_largura_para_ford_fulkerson(g, pai, distancia):
    print("\nFF-BFS\n")
    for vertice in g: ######## inicializar
        vertice.visitado = False
        vertice.pai = None
    Q = []
    saida = []
    Q.append(pai)
    saida.append(pai)
    while len(Q) != 0:
        extraido = Q.pop(0)
        extraido.visitado = True
        for aresta in extraido.adj: #pra cada aresta adj ao elemento extraido da fila
            if aresta.distancia.visitado != True and aresta.capacidade > 0: #se a aresta ainda n foi visitada e sua capacidade for maior que 0
                aresta.distancia.pai = extraido #pai da aresta se torna o elemento extraido
                Q.append(aresta.distancia) #coloca na fila a distancia da aresta
                saida.append(aresta.distancia) #coloca na lista de saída a distancia
                aresta.distancia.visitado = True #marca a aresta como visitada    
                print(f'{extraido.indice} - {aresta.distancia.indice}')
    if pai not in saida or distancia not in saida:
        print(f"sem caminho de {pai.indice} ate {distancia.indice}")
        return None

    print(f'\nSequencia de visita em lista: ')
    for vertice in saida:
        print(vertice.indice, end=" ") 
    print('\n')
    return saida
# update_flow recebe a lista de arestas de s a t no grafo
# para cada aresta na lista, atualiza o fluxo de s a t
# acha a aresta com menor capacidade e atualiza o fluxo das outros baseadas nesse vaor
# por fim retorna o valor subtraido 
def update_flow(arestas):
    menor = arestas[0]
    for aresta in arestas:
        if aresta.capacidade < menor.capacidade:
            menor = aresta
    temp = menor.capacidade
    for aresta in arestas:
        print(f' - {aresta.distancia.indice}  cap of {aresta.capacidade}')
        aresta.capacidade -= temp
    return temp

def ford_fulkerson(vertices, s, t):
    print("Ford-Fulkerson")
    print(f'comeco: {s.indice}   fim: {t.indice}')

    for vertice in vertices:
        vertice.pai = None
        vertice.visitado = False

    # printa as arestas e a capacidade de cada uma
    for vertice in vertices:
        for aresta in vertice.adj:
            print(f'{vertice.indice} - {aresta.distancia.indice} with {aresta.capacidade}')

    max_flow = 0
    
    # busca_em_largura_para_ford_fulkerson para encontrar o caminho de s a t
    # caso nao exista um caminho de s a t retorna None
    # para percorer pelo caminnho comecamos de t e voltamos a s, 
    # usnado o atributo pai para saber o pai do vertice
    vef = busca_em_largura_para_ford_fulkerson(vertices, s, t)
    while vef != None:

        # c guarda as arestas que estao no caminho de s a t
        c = []
        
        # percore o caminho reverso, de t ate s, adicionando
        # as arestas no caminho para 'c'        
        extraido = t
        while extraido.pai != None:
                c.append(extraido.pai.aresta_do_vertice(extraido))
                extraido = extraido.pai
        
        max_flow += update_flow(c)

        for vertice in vertices:
            for aresta in vertice.adj:
                print(f'{vertice.indice} - {aresta.distancia.indice} with {aresta.capacidade}')

        vef = busca_em_largura_para_ford_fulkerson(vertices, s, t)
    print(f'flow maximo = {max_flow}')
    return max_flow


def main():
    # vertices ficao guardados como ponteiros dentro de 'vertices'
    vertices = []
    lenv = 8
   
    # arestas determina quais adj entre os nos sao feitas
    # arestas = [(1,2, 2), (1,4, 2), 
    #            (2,3, -3), (2,4, -1), 
    #            (3,7, -2), 
    #            (4,8, 2), (4,1, -1),
    #            (5,7, 1), (5,6, 2),
    #            (6,5, 2), 
    #            (8,7, 4)]
    # arestas = [(1,2, 1), (1,5, 1), (2,3, 2), (2,4, 2), (2,5, 2), (3,5, 3), (4,6, 4), (5,6, 3)]
    
    # para ford_fulkerson
    arestas = [(1,2, 20), (1,4, 15), 
               (2,3, 7), (2,4, 8), 
               (3,7, 18), 
               (4,8, 33), (4,1, 6),
               (5,7, 9), (5,6, 17),
               (6,5, 33), 
               (8,7, 20)]

    for i in range(lenv):
        vertice = Vertice(i+1)
        vertices.append(vertice)
   
    #para ford_fulkerson
    for src, distancia, c in arestas:
        vertices[src-1].adj.append(Aresta(vertices[distancia-1], 1, capacidade=c))

    # orientado
    # for src, distancia, peso in arestas:
    #     vertices[src-1].addAresta(vertices[distancia-1], peso)
    
    # nao orientado
    # for src, distancia, peso, in arestas:
    #     vertices[src-1].addAresta(vertices[distancia-1], peso)
    #     vertices[distancia-1].addAresta(vertices[src-1], peso)

    print(vertices[0].indice)

    for vertice in vertices: ##sempre pra busca em largura e profundidade
        vertice.visitado = False
        vertice.pai = None

    print("Busca em Largura: ")     
    busca_em_largura(vertices, vertices[0])
    for vertice in vertices: ##sempre pra busca em largura e profundidade
        vertice.visitado = False
        vertice.pai = None
    print("Busca em Profundidade: ")
    busca_em_profundidade(vertices, vertices[0])

    for vertice in vertices: #coloca a distancia de todos os vértices com infinito e os pais como nulos
        vertice.distancia = float('inf')
        vertice.pai = None
    vertices[0].distancia = 0
    print("Bellman-Ford")
    bellman_ford(vertices, vertices[0])

    print("Dijkstra")  
    for vertice in vertices:
        vertice.distancia = float('inf')
        vertice.pai = None
        vertice.visitado = False
    vertices[0].distancia = 0
    dijkstra(vertices, vertices[0])

    print("Floyd-Warshall")
    floyd_warshall(vertices)

    print("Prim")
    for vertice in vertices:
        vertice.distancia = float('inf')
        vertice.pai = None
        vertice.visitado = False
    prim(vertices)


    ford_fulkerson(vertices, vertices[0], vertices[6])

if __name__ == "__main__":
    main()