class Vertice():
    def __init__(self, indice):
        self.indice = indice
        self.visitado = False        
        self.pai = None
        self.distancia = float('inf')
        self.adj = []
        
    def aresta_do_vertice(self, distancia): #encontra aresta do vértice
        for aresta in self.adj:
            if aresta.distancia == distancia:
                return aresta
    def addAresta(self, distancia, peso):
        self.adj.append(Aresta(distancia, peso))

class Aresta():
    def __init__(self, distancia, peso, **kwargs):
        self.distancia = distancia
        self.peso = peso
        self.capacidade = kwargs.get('capacidade', float('inf'))
        self.flow = 0

def inicializar(vertices):
        for vertice in vertices:
            vertice.pai = None
            vertice.visitado = False
        vertices[0].distancia = 0

def printa_dist_pred(distancia,predecessores,tamanho):
    print('\nMatriz de Distancias:\n')
    for i in range(tamanho): ##printa arvore de distancias
        for j in range(tamanho): 
            if distancia[i][j] == float ('inf'):
                print(f'+\t', end='') #printa + se o valor for infinito
            else:
                print(f'{distancia[i][j]}\t', end='')
        print()
    print('\nMatriz de Predecessores:\n')
    for i in range(tamanho): #printa arvore de predecessores
        for j in range(tamanho):
            print(f'{predecessores[i][j]}\t', end='')
        print()

def busca_em_largura(grafo, pai):
    Q = []
    Q.append(pai)
    print(f'Sequencia: {pai.indice} ', end='')
    while len(Q) != 0 :
        pai = Q.pop(0) # remove o pai           
        for aresta in pai.adj: #pra cada aresta adj ao pai
            if aresta.distancia.visitado == False: #se ainda não foi visitada
                print(f'- {aresta.distancia.indice} ', end='')
                aresta.distancia.pai = pai
                Q.append(aresta.distancia) #coloca na fila
                aresta.distancia.visitado = True
        if len(Q) == 0:
            for vertice in grafo:
                if vertice.visitado == False:
                    Q.append(vertice)
                    break
        pai.visitado = True # marca como visitado
    print('\n')

def busca_em_profundidade(grafo, pai): ########## DFS ############################################################################### MEXIDO
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
            for vertice in grafo:
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
        print(f'vertice: {vertice.indice} - ', end='')
        if vertice.distancia != float("inf"):
            print(f'distancia: {vertice.distancia}')
        else: #caso ainda tenha distancia com valor infinito
            print(f'distancia: Nao tem!!!')
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
        print(f'vertice: {vertice.indice} - ', end='')
        if vertice.distancia != float("inf"):
            print(f'distancia: {vertice.distancia}')
        else: #caso ainda tenha distancia com valor infinito
            print(f'Nao tem!!!')
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
                if distancias[i][k] + distancias[k][j] < distancias[i][j]: #funciona como um "relaxamento" só que da arvore
                    distancias[i][j] = distancias[i][k] + distancias[k][j]
                    predecessores[i][j] = predecessores[k][j]
    printa_dist_pred(distancias,predecessores,len(vertices)) #imprime as duas matrizes

def printa_prim(l): #imprimir a prim
    Q = []
    print('\nOrdem:')
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
            if arvore[j]:
                for k in range(len(vertices)):
                    if arvore[k] == False and vertices[j].aresta_do_vertice(vertices[k]): #se arvore[k] for falso e tiver a aresta
                        if chave > vertices[j].aresta_do_vertice(vertices[k]).peso: #se o peso da chave for maior que a da aresta
                            chave = vertices[j].aresta_do_vertice(vertices[k]).peso #a chave passa a valer o novo peso
                            pai = j
                            b = k
        arvore[b] = True
        saida.extend([vertices[pai],vertices[b]])
        print(f'({vertices[pai].indice} - {vertices[b].indice}), ', end="")
    printa_prim(saida)
    print('\n')

def printa_busca_em_largura_ford_fulkerson(vertices): ################################# EXCLUIR
    print(f'\nSequencia de visita em lista: ')
    for vertice in vertices:
        print(vertice.indice, end=" ") 
    print('\n')

def busca_em_largura_para_ford_fulkerson(grafo, pai, distancia):
    for vertice in grafo:
        vertice.visitado = False
        vertice.pai = None
    Q = []
    saida = []
    Q.append(pai)
    saida.append(pai)
    while len(Q) != 0:
        extraido = Q.pop(0)
        extraido.visitado = True
        for aresta in extraido.adj: #pra cada aresta adj
            if aresta.distancia.visitado != True and aresta.capacidade > 0:
                aresta.distancia.visitado = True    
                aresta.distancia.pai = extraido
                Q.append(aresta.distancia)
                saida.append(aresta.distancia)
    if pai not in saida or distancia not in saida:
        return None
    return saida

# update_flow recebe a lista de arestas de s a t no grafo
# para cada aresta na lista, atualiza o fluxo de s a t
# acha a aresta com menor capacidade e atualiza o fluxo das outros baseadas nesse vaor
# por fim retorna o valor subtraido 

def ford_fulkerson(vertices, s, t):
    # printa as arestas e a capacidade de cada uma
    print('\nArestas e capacidades antes de Ford-Fulkerson\n')
    for vertice in vertices:
        for aresta in vertice.adj:
            print(f'Aresta: ({vertice.indice} - {aresta.distancia.indice}) tem a capacidade: {aresta.capacidade}')
    flowMaximo = 0 #menor flow possível
    x = busca_em_largura_para_ford_fulkerson(vertices, s, t)
    while x != None:
        saida = []           
        extraido = t
        while extraido.pai != None:
                saida.append(extraido.pai.aresta_do_vertice(extraido))
                extraido = extraido.pai
        menor = saida[0]
        for aresta in saida:
            if aresta.capacidade < menor.capacidade:
                menor = aresta
        temp = menor.capacidade
        for aresta in saida:
            aresta.capacidade -= temp
        flowMaximo += temp
        print('\n')
        for vertice in vertices:
            for aresta in vertice.adj:
                print(f'({vertice.indice} - {aresta.distancia.indice}) tem a capacidade: {aresta.capacidade}')

        x = busca_em_largura_para_ford_fulkerson(vertices, s, t)
    print(f'\nFlow Maximo do grafo e de: {flowMaximo}')

def main():
    # vertices ficao guardados como ponteiros dentro de 'vertices'
    vertices = []   
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
    arestas = [(1,2, 20), (1,4, 15), (2,3, 7), (2,4, 8), (3,7, 18), (4,8, 33), (4,1, 6),(5,7, 9), (5,6, 17),(6,5, 33), (8,7, 20)]

    for i in range(8):
        vertice = Vertice(i+1)
        vertices.append(vertice)
   
    #para ford_fulkerson
    for src, distancia, peso in arestas:
        vertices[src-1].adj.append(Aresta(vertices[distancia-1], 1, capacidade=peso))

     #orientado
    #for src, distancia, peso in arestas:
        #vertices[src-1].addAresta(vertices[distancia-1], peso)
    
    #nao orientado
    #for src, distancia, peso, in arestas:
         #vertices[src-1].addAresta(vertices[distancia-1], peso)
         #vertices[distancia-1].addAresta(vertices[src-1], peso)

    print("Busca em Largura: ")     
    inicializar(vertices)
    busca_em_largura(vertices, vertices[0])

    print("Busca em Profundidade: ")
    inicializar(vertices)
    busca_em_profundidade(vertices, vertices[0])

    print("Bellman-Ford")
    inicializar(vertices)
    bellman_ford(vertices, vertices[0])

    print("Dijkstra")  
    inicializar(vertices)
    dijkstra(vertices, vertices[0])

    print("Floyd-Warshall")
    floyd_warshall(vertices)

    print("Prim")
    inicializar(vertices)
    prim(vertices)
    
    print("Ford-Fulkerson")
    inicializar(vertices)
    ford_fulkerson(vertices, vertices[0], vertices[6])

if __name__ == "__main__":
    main()