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
        #self.ff_pai = None


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
        self.capacity = kwargs.get('capacity', float('inf'))
        self.flow = 0

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
                print(f'-> {aresta.distancia.indice} ', end='')
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
                print(f'-> {aresta.distancia.indice} ', end='')
        if len(Q) == 0:
            for vertice in g:
                if vertice.visitado == False:
                    Q.append(vertice)
                    break
        pai.visitado = True
    print('\n')


def relaxamento(u, v, peso): ############################################################################### MEXIDO
    if  v.distancia > u.distancia + peso:
        v.distancia = u.distancia + peso
        v.pai = u

def bellman_ford(vertices, pai): ############################################################################### MEXIDO
    for vertice in vertices[:-1]: #relaxa todos os vértices do grafo
        for aresta in vertice.adj:
            relaxamento(vertice, aresta.distancia, vertice.aresta_do_vertice(aresta.distancia).peso)
    for vertice in vertices:
        print(f'{vertice.indice} -> ', end='')
        if vertice.distancia != float("inf"):
            print(vertice.distancia)
        else: #caso ainda tenha distancia com valor infinito
            print(f'Nao ha caminho! Pai: {pai.indice}')
    print('\n')

def dijkstra(vertices, pai): 
    Q = []
    Q.append(pai)
    while len(Q) != 0:
        p = Q.pop(0)
        p.visitado = True
        for aresta in p.adj:
            if aresta.distancia.visitado == False:
                Q.append(aresta.distancia)
                aresta.distancia.visitado = True
            relaxamento(p, aresta.distancia, aresta.peso)
    for vertice in vertices:
        print(f'{vertice.indice} -> ', end='')
        if vertice.distancia != float("inf"):
            print(vertice.distancia)
        else: #caso ainda tenha distancia com valor infinito
            print(f'Nao ha caminho! Pai: {pai.indice}')
    print('\n')
# nao funciona com grafo nao orientado com valores negativos
# mas funciona com grafo orientado com valores negativos
def floyd_warshall(vertices):
    distancia = [[0 for i in range(len(vertices))]for j in range(len(vertices))] #cria a matriz de distâncias
    predecessores = [[0 for i in range(len(vertices))]for j in range(len(vertices))]   #cria a matriz de predecessores
    for i in range(0, len(vertices)): ##preenche as matrizes com infinito
        for j in range(0, len(vertices)):
            distancia[i][j] = float('inf')
            predecessores[i][j] = float('inf')
            
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i == j:
                distancia[i][j] = 0 #se i for = j o peso será 0
            elif vertices[i].aresta_do_vertice(vertices[j]):
                distancia[i][j] = vertices[i].aresta_do_vertice(vertices[j]).peso ##distancia recebe o peso da aresta
            predecessores[i][j] = i #a matriz de predecessores recebe i

    for i in range(len(vertices)): ##busca o menor caminho e atualiza a lista de distância e predecessores
        for j in range(len(vertices)):
            for k in range(len(vertices)): 
                if distancia[i][j] > distancia[i][k] + distancia[k][j]: #funciona como um "relaxamento" só que da matriz
                    distancia[i][j] = distancia[i][k] + distancia[k][j]
                    predecessores[i][j] = predecessores[k][j]

    for i in range(len(vertices)): ##printa matriz de distancias
        for j in range(len(vertices)): ##colocar algum if distancia[i][j] == float('inf'), printa outra coisa pra sair certinho
            print(f'{distancia[i][j]}\t', end='')
        print()

    print()
    for i in range(len(vertices)): #printa matriz de predecessores
        for j in range(len(vertices)):
            print(f'{predecessores[i][j]}\t', end='')
        print()
    print()
    # predecessores funciona assim:
    # caso vc queria saber o caminho de menor distancia do vertice 0 ao 6
    # vc olha predecessores[0][6] e ve que o valor eh 2,
    # entao vc olha o menor caminho de 0 a 2, ou seja, predecessores[0][2] e ve que o valor eh 1,
    # entao vc olha predecessores[0][1] e ve que o valor eh 0, que eh o vertice que queremos comecar
    # logo temos o menor caminho de 0 ate 6, 0->1->2->6
    # neste caso, menor caminho == menor custo

    return distancia, predecessores

def prim(vertices, pai):
    print("Prim")
    for v in vertices:
        v.pm_distancia = float('inf')
        v.pm_pai = None
        v.visitado = False

    v = len(vertices)
    noEdge = 0
    selected = [0]*v
    selected[0] = True
    
    l = []

    while noEdge < v-1:
        minimum = float('inf')
        x, y = 0, 0
        for i in range(v):
            if selected[i] == True:
                for j in range(v):
                    if not selected[j] and vertices[i].aresta_do_vertice(vertices[j]):
                        if vertices[i].aresta_do_vertice(vertices[j]).peso < minimum:
                            minimum = vertices[i].aresta_do_vertice(vertices[j]).peso
                            x = i
                            y = j


        print(f'{vertices[x].indice} -> {vertices[y].indice}')
        selected[y] = True
        if vertices[x] not in l:
            l.append(vertices[x])
        if vertices[y] not in l:
            l.append(vertices[y])
        noEdge+=1
    for i in range(len(l)):
        print(f'{l[i].indice} ', end='')
    print()
    return l

def ff_bfs(g, pai, distancia):
    print("FF-BFS")
    for v in g:
        v.visitado = False
        v.ff_pai = None

    q = []
    seq = []

    q.append(pai)
    seq.append(pai)

    print(f'Comecou em {pai.indice}')

    while q != []:
        p = q.pop(0)
        p.visitado = True  

        for aresta in p.adj:
            if not aresta.distancia.visitado and aresta.capacity > 0:
                aresta.distancia.ff_pai = p
                print(f'{p.indice} -> {aresta.distancia.indice}')
                q.append(aresta.distancia)
                seq.append(aresta.distancia)
                aresta.distancia.visitado = True
                # print(f'-> {aresta.distancia.indice} ', end='')
        # if q == []:
        #     for v in g:
        #         if v.visitado == False:
        #             q.append(v)
        #             break

    
    if pai not in seq or distancia not in seq:
        print(f"sem caminho de {pai.indice} ate {distancia.indice}")
        return None

    print(f'\nSequencia de visita em lista: ')
    for v in seq:
        print(v.indice, end=" ") 
    print('\n')
    return seq
# update_flow recebe a lista de arestas de s a t no grafo
# para cada aresta na lista, atualiza o fluxo de s a t
# acha a aresta com menor capacidade e atualiza o fluxo das outros baseadas nesse vaor
# por fim retorna o valor subtraido 
def update_flow(arestas):
    menor = arestas[0]
    for aresta in arestas:
        if aresta.capacity < menor.capacity:
            menor = aresta
    temp = menor.capacity
    for aresta in arestas:
        print(f' -> {aresta.distancia.indice}  cap of {aresta.capacity}')
        aresta.capacity -= temp
    return temp
def ford_fulkerson(vertices, s, t):
    print("Ford-Fulkerson")
    print(f'comeco: {s.indice}   fim: {t.indice}')

    for v in vertices:
        v.ff_pai = None
        v.visitado = False

    # printa as arestas e a capacidade de cada uma
    for v in vertices:
        for aresta in v.adj:
            print(f'{v.indice} -> {aresta.distancia.indice} with {aresta.capacity}')

    max_flow = 0
    
    # ff_bfs para encontrar o caminho de s a t
    # caso nao exista um caminho de s a t retorna None
    # para percorer pelo caminnho comecamos de t e voltamos a s, 
    # usnado o atributo ff_pai para saber o pai do vertice
    vef = ff_bfs(vertices, s, t)
    while vef != None:

        # c guarda as arestas que estao no caminho de s a t
        c = []
        
        # percore o caminho reverso, de t ate s, adicionando
        # as arestas no caminho para 'c'        
        p = t
        while p.ff_pai != None:
                c.append(p.ff_pai.aresta_do_vertice(p))
                p = p.ff_pai
        
        max_flow += update_flow(c)

        for v in vertices:
            for aresta in v.adj:
                print(f'{v.indice} -> {aresta.distancia.indice} with {aresta.capacity}')

        vef = ff_bfs(vertices, s, t)
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
        v = Vertice(i+1)
        vertices.append(v)
   
    #para ford_fulkerson
    for src, distancia, c in arestas:
        vertices[src-1].adj.append(Aresta(vertices[distancia-1], 1, capacity=c))

    # orientado
    # for src, distancia, peso in arestas:
    #     vertices[src-1].addAresta(vertices[distancia-1], peso)
    
    # nao orientado
    # for src, distancia, peso, in arestas:
    #     vertices[src-1].addAresta(vertices[distancia-1], peso)
    #     vertices[distancia-1].addAresta(vertices[src-1], peso)

    print(vertices[0].indice)

    for v in vertices: ##sempre pra busca em largura e profundidade
        v.visitado = False
        v.pai = None

    print("Busca em Largura: ")     
    busca_em_largura(vertices, vertices[0])
    for v in vertices: ##sempre pra busca em largura e profundidade
        v.visitado = False
        v.pai = None
    print("Busca em Profundidade: ")
    busca_em_profundidade(vertices, vertices[0])

    for vertice in vertices: #coloca a distancia de todos os vértices com infinito e os pais como nulos
        vertice.distancia = float('inf')
        vertice.pai = None
    vertices[0].distancia = 0
    print("Bellman-Ford")
    bellman_ford(vertices, vertices[0])

    for vertice in vertices:
        vertice.distancia = float('inf')
        vertice.pai = None
        vertice.visitado = False
    vertices[0].distancia = 0
    print("Dijkstra")  
    dijkstra(vertices, vertices[0])

    print("Floyd-Warshall")
    floyd_warshall(vertices)


    prim(vertices, vertices[0])
    ford_fulkerson(vertices, vertices[0], vertices[6])

if __name__ == "__main__":
    main()