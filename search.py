# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from util import Queue

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    
    ### VARIABLES PARA IMPLEMENTACION ####
    
    visitado = []          #estados visitados
    ruta = []              # rutas
    cola = util.Queue()         # se tienen las coordenadas (x,y) y la ruta para llegar a ella ((x,y),ruta)
    
    ##### COMIENZA EL ALGORITMO ###
    
    #El algoritmo comienza preguntando si el estado en el que se encuentra
    # es el estado meta
    estado = problem.getStartState()
    if problem.isGoalState(estado) == True:
        return ruta
    
    # Si el estado no es meta entonces comienza a expandir y visitar a los 
    # nodos más cercanos
    cola.push((estado,[]))
    
    
    #Mientras tenga memoria, es decir, la cola no este llena:
    while not cola.isEmpty():
        
        # Agregamos el estado y la ruta a la cola
        posicion,ruta = cola.pop()
        visitado.append(posicion)

        #Si es meta, se devuelve la ruta
        if problem.isGoalState(posicion) == True:
            return ruta
        
        #Si no es meta se obtiene a los sucesores del estado actual
        sucesores = problem.getSuccessors(posicion)
        
        #Se añaden estos sucesores a la cola
        if sucesores:
            for sucesor in sucesores:
                if sucesor[0] not in visitado:
                    nuevaRuta = ruta + [sucesor[1]]
                    cola.push((sucesor[0],nuevaRuta))
                        
        

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0




def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    ### VARIABLES PARA IMPLEMENTACION ####

    visitado = []          #estados visitados
    ruta = []              # rutas
    cola = util.PriorityQueue()         # se tienen las coordenadas (x,y) y la ruta para llegar a ella ((x,y),ruta)
    costo = 0

    ##### COMIENZA EL ALGORITMO ###

    #Obetenemos el estado en el que se encuentra
    estado = problem.getStartState()
    #if problem.isGoalState(estado) == True:
    #   return ruta
    #heuristic = heuristic(estado,problem)
    cola.push((estado,ruta),costo)

    while not cola.isEmpty():
        
        estado_actual = cola.pop()
        pos,ruta = estado_actual[0],estado_actual[1]
        
        
        if problem.isGoalState(pos):
            return ruta
        
        else:
            #Si no se ha visitado, lo agregamos a la lista
            if pos not in visitado:
                visitado.append(pos)
                
                #Obtenemos los sucesores de donde nos enocontramos
                sucesores = problem.getSuccessors(pos)
                
                for sucesor in sucesores:
                    if sucesor[0] not in visitado:
                        #Se calcula la nueva ruta y la posicion de cada sucesor
                        posNueva = sucesor[0]
                        nuevaRuta = ruta + [sucesor[1]]
                        
                        #Usamos las funciones de util.py para obtener el costo
                        
                        #Costo de la nueva ruta
                        g = problem.getCostOfActions(nuevaRuta)
                        #Costo menor del estado a la meta
                        h = heuristic(posNueva,problem)
                        costo = g + h
                        
                        cola.push((posNueva,nuevaRuta),costo)
 



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
