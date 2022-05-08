def inicializar(vertices): #inicializa os vértices
        for vertice in vertices:
            vertice.distancia = float('inf')
            vertice.s = None
            vertice.visitado = False
        vertices[0].distancia = 0

def relaxamento(u, vertice, peso): #relaxa o vértice
    if  vertice.distancia > u.distancia + peso:
        vertice.distancia = u.distancia + peso
        vertice.s = u

def printa_dist_pred(distancia,predecessores,tamanho): #printa as matrizes de distâncias e predecessores do algoritmo de floyd warshall
    print('\nMatriz de Distancias:\n')
    for i in range(tamanho): #printa matriz de distancias
        for j in range(tamanho): 
            if distancia[i][j] == float ('inf'):
                print(f'+   ', end='') #printa + se o valor for infinito
            else:
                print(f'{distancia[i][j]}   ', end='')
        print()
    print('\nMatriz de Predecessores:\n')
    for i in range(tamanho): #printa matriz de predecessores
        for j in range(tamanho):
            print(f'{predecessores[i][j]}   ', end='')
        print()
    print('\n')

def busca_em_largura(grafo, s):
    Lista = [] #fila
    Lista.append(s)
    print(f'Sequencia: {s.indice} ', end='') #printa o s na tela
    while len(Lista) != 0 :
        s = Lista.pop(0) #remove o s           
        for aresta in s.adjacente: #pra cada aresta adjacente ao s
            if aresta.distancia.visitado == False: #se ainda não foi visitada
                print(f'- {aresta.distancia.indice} ', end='')
                aresta.distancia.s = s
                Lista.append(aresta.distancia) #coloca na fila
                aresta.distancia.visitado = True
        if len(Lista) == 0:
            for vertice in grafo:
                if vertice.visitado == False:
                    Lista.append(vertice)
                    break
        s.visitado = True # marca como visitado
    print('\n')

def busca_em_profundidade(grafo, s):
    Lista = [] #fila
    Lista.append(s)
    print(f'Sequencia: {s.indice} ', end = '') #printa o s na tela
    while len(Lista) != 0:
        s = Lista.pop(0) #remove o s            
        for aresta in s.adjacente: #pra cada aresta adjacente ao s
            if aresta.distancia.visitado == False:  #se ainda não foi visitada
                aresta.distancia.visitado = True #marca como visitado
                aresta.distancia.s = s
                Lista += [aresta.distancia]
                print(f'- {aresta.distancia.indice} ', end='')
        if len(Lista) == 0:
            for vertice in grafo:
                if vertice.visitado == False: #adiciona algum vertice nao visitado na fila
                    Lista.append(vertice)
                    break
        s.visitado = True #marca o s como visitado
    print('\n')

class Vertice():
    def __init__(self, indice): #atributos de um vértice
        self.s = None
        self.visitado = False        
        self.indice = indice
        self.distancia = float('inf')
        self.adjacente = []
        
    def aresta_do_vertice(self, distancia): #encontra aresta do vértice
        for aresta in self.adjacente:
            if aresta.distancia == distancia:
                return aresta
                
class Aresta(): #atributos de uma aresta
    def __init__(self, peso, distancia, **capacidade):
        self.peso = peso
        self.distancia = distancia
        self.fluxo = 0
        self.capacidade = capacidade.get('capacidade', float('inf'))
        self.indice = 0

def dijkstra(vertices, s): 
    Lista = [] #fila
    Lista.append(s)
    while len(Lista) != 0:
        extraido = Lista.pop(0) #extrai o s
        extraido.visitado = True #marca como visitado
        for aresta in extraido.adjacente: #pra cada aresta adjacente ao extraido
            if aresta.distancia.visitado == False:
                Lista.append(aresta.distancia)
                aresta.distancia.visitado = True
            relaxamento(extraido, aresta.distancia, aresta.peso) #relaxa o vertice
    for vertice in vertices: #printa na tela o vertice e a sua distância
        print(f'vertice: {vertice.indice} - ', end='')
        if vertice.distancia != float("inf"):
            print(f'distancia: {vertice.distancia}')
        else: #caso ainda tenha distancia com valor infinito
            print(f'distancia: Nao ha como chegar a partir s !!!')
    print('\n')

def busca_em_largura_para_ford_fulkerson(grafo, s, distancia):
    inicializar(grafo)
    Lista = []
    Lista.append(s)
    saida = []
    saida.append(s)
    while len(Lista) != 0:
        extraido = Lista.pop(0)
        extraido.visitado = True
        for aresta in extraido.adjacente: #para cada aresta adjacente
            if aresta.capacidade > 0 and aresta.distancia.visitado != True:
                aresta.distancia.visitado = True    
                aresta.distancia.s = extraido
                Lista.append(aresta.distancia)
                saida.append(aresta.distancia)
    if distancia not in saida or s not in saida:
        return -1
    else:
        return saida

def printa_arestas(vertices):
    for vertice in vertices:
        for aresta in vertice.adjacente:
            print(f'Aresta: ({vertice.indice} - {aresta.distancia.indice}) tem a capacidade: {aresta.capacidade}')

def ford_fulkerson(fonte, sumidouro, vertices):
    x = busca_em_largura_para_ford_fulkerson(vertices, fonte, sumidouro)
    print('\nArestas e capacidades antes de Ford-Fulkerson\n')
    printa_arestas(vertices)
    fluxoMaximo = 0 #menorfluxo possível
    while x != -1: #se retornar nada
        extraido = sumidouro
        saida = []           
        while extraido.s != None:
                saida.append(extraido.s.aresta_do_vertice(extraido))
                extraido = extraido.s
        menorCaminho = saida[0]
        for aresta in saida:
            if  menorCaminho.capacidade > aresta.capacidade:
                menorCaminho = aresta
        y = menorCaminho.capacidade
        for aresta in saida:
            aresta.capacidade -= y
        fluxoMaximo += y
        x = busca_em_largura_para_ford_fulkerson(vertices, fonte, sumidouro)
    print('\nArestas e capacidades depois de Ford-Fulkerson\n')
    printa_arestas(vertices)
    print(f'\nFlow Maximo do grafo e de: {fluxoMaximo}\n')

def bellman_ford(vertices):
    for vertice in vertices[:-1]: #relaxa todos os vértices do grafo
        for aresta in vertice.adjacente:
            relaxamento(vertice, aresta.distancia, vertice.aresta_do_vertice(aresta.distancia).peso)
    for vertice in vertices:
        print(f'vertice: {vertice.indice} - ', end='')
        if vertice.distancia != float("inf"): #se a distância do vértice for diferente de infinito printa a distância
            print(f'distancia: {vertice.distancia}')
        else: #caso ainda tenha distancia com valor infinito
            print(f'distancia: Nao ha como chegar a partir de s !!!')
    print('\n')

def relaxamento_floyd_warshall(distancias,predecessores,i,j,k): #é um "relaxamento" de elementos da matriz de distancias
    if distancias[i][k] + distancias[k][j] < distancias[i][j]: 
        distancias[i][j] = distancias[i][k] + distancias[k][j]
        predecessores[i][j] = predecessores[k][j]

def floyd_warshall(vertices):
    distancias = [[0 for i in range(len(vertices))]for j in range(len(vertices))] #cria a arvore de distâncias
    predecessores = [[0 for i in range(len(vertices))]for j in range(len(vertices))]   #cria a arvore de predecessores
    for i in range(0, len(vertices)): ##preenche predecessores com infinito
        for j in range(0, len(vertices)):
            predecessores[i][j] = float('inf')            
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i == j:
                distancias[i][j] = 0 #se i for = j o peso será 0
            elif vertices[i].aresta_do_vertice(vertices[j]): #se i != j e a aresta pertence ao vértice
                distancias[i][j] = vertices[i].aresta_do_vertice(vertices[j]).peso ##distancia recebe o peso da aresta
            else:
                distancias[i][j] = float('inf') #se i !=j e a aresta não pertence ao vértice
            predecessores[i][j] = i #a arvore de predecessores recebe o índice i
    for i in range(len(vertices)): #busca o menorCaminho caminho e atualiza a lista de distância e predecessores
        for j in range(len(vertices)):
            for k in range(len(vertices)): 
                relaxamento_floyd_warshall(distancias,predecessores,i,j,k)
    printa_dist_pred(distancias,predecessores,len(vertices)) #imprime as duas matrizes

def prim(vertices):
    arvore = [0 for i in range(len(vertices))] #cria a arvore
    for i in range(len(vertices)-1): #todos os vértices
        chave = float('inf')
        s = 0
        arvore[0] = True   
        a = 0
        for j in range(len(vertices)):
            if arvore[j]:
                for k in range(len(vertices)):
                    if vertices[j].aresta_do_vertice(vertices[k]) and arvore[k] == False: #se arvore[k] for falso e tiver a aresta
                        if chave > vertices[j].aresta_do_vertice(vertices[k]).peso: #se o peso da chave for maior que a da aresta
                            chave = vertices[j].aresta_do_vertice(vertices[k]).peso #a chave passa a valer o novo peso
                            s = j #salva os índices 
                            a = k
        arvore[a] = True
        print(f'({vertices[s].indice} - {vertices[a].indice}), ', end="")
    print('\n')

def main():

    arestas = [(1,2, 5), (1,4, 8), (2,7, 3), (3,6, 15), (4,5, 21), (4,6, 7), (5,7, 1),(2,5, 2), (1,6, 17),(7,6, 13), (3,4, 4)] #arestas do grafo
    vertices = []   
    for i in range(7): #cria os vértices e coloca na lista vertices
        vertice = Vertice(i+1)
        vertices.append(vertice)   
    for a, distancia, peso in arestas: #adiciona as arestas aos vértices
        vertices[a-1].adjacente.append(Aresta(1,vertices[distancia-1], capacidade=peso))
    print("Busca em Largura: ")     
    inicializar(vertices)
    busca_em_largura(vertices, vertices[0])

    print("Busca em Profundidade: ")
    inicializar(vertices)
    busca_em_profundidade(vertices, vertices[0])

    print("Dijkstra")  
    inicializar(vertices)
    dijkstra(vertices, vertices[0])
    
    print("Bellman-Ford")
    inicializar(vertices)
    bellman_ford(vertices)

    print("Ford-Fulkerson")
    inicializar(vertices)
    ford_fulkerson(vertices[0], vertices[6], vertices)

    print("Floyd-Warshall")
    floyd_warshall(vertices)

    print("Prim")
    inicializar(vertices)
    prim(vertices)
    
if __name__ == "__main__":
    main()