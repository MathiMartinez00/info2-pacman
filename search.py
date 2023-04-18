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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from util import Stack
    from game import Directions
    closed = set() # conjunto de nodos visitados para la busqueda en grafo
    fringe = Stack() # Implementacion del fringe tipo LIFO para el DFS
    actions = [Directions.STOP] # lista con las acciones que el agante debe hacer para llegar a cierto estado (sirve para retornar)
    fringe.push((problem.getStartState(), actions)) # agrego (estado, [accion inicial = quedate en donde estas])
    # nodo := (estado, lista de acciones para llegar a este estado)
    # estado := (x,y)
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop()
        if problem.isGoalState(node[0]): 
            return node[1] # retorno la lista de acciones que el agente debe hacer para llegar a el goal state
        if node[0] not in closed:
            closed.add(node[0])
            for sucessor in problem.getSuccessors(node[0]): # expando el nodo que recien quite del fringe
                #sucessor := (estado, accion, costo de paso)
                fringe.push((sucessor[0], node[1] + [sucessor[1]])) # agrego los sucesores del nodo al fringe

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    from game import Directions
    closed = set() # conjunto de nodos visitados para la busqueda en grafo
    fringe = Queue() # Implementacion del fringe tipo FIFO para el BFS
    actions = [Directions.STOP] # lista con las acciones que el agante debe hacer para llegar a cierto estado (sirve para retornar)
    fringe.push((problem.getStartState(), actions)) # agrego (estado, [accion inicial = quedate en donde estas])
    # nodo := (estado, lista de acciones para llegar a este estado)
    # estado := (x,y)
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop()
        if problem.isGoalState(node[0]): 
            return node[1] # retorno la lista de acciones que el agente debe hacer para llegar a el goal state
        if node[0] not in closed:
            closed.add(node[0])
            for sucessor in problem.getSuccessors(node[0]): # expando el nodo que recien quite del fringe
                #sucessor := (estado, accion, costo de paso)
                fringe.push((sucessor[0], node[1] + [sucessor[1]])) # agrego los sucesores del nodo al fringe

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    from game import Directions
    closed = set() # conjunto de nodos visitados para la busqueda en grafo
    fringe = PriorityQueue() # Implementacion del fringe tipo FIFO con prioridad para el UCS
    actions = [Directions.STOP] # lista con las acciones que el agante debe hacer para llegar a cierto estado (sirve para retornar)
    fringe.push((problem.getStartState(), actions, 0), 0) # agrego ((estado, [accion inicial = quedate en donde estas], costo acumulado), prioridad)
    # nodo := (estado, lista de acciones para llegar a este estado, costo acumulado)
    # estado := (x,y)
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop() # quita el que tenga menor prioridad
        if problem.isGoalState(node[0]): 
            return node[1] # retorno la lista de acciones que el agente debe hacer para llegar a el goal state
        if node[0] not in closed:
            closed.add(node[0])
            for sucessor in problem.getSuccessors(node[0]): # expando el nodo que recien quite del fringe
                #sucessor := (estado, accion, costo de paso)
                fringe.push((sucessor[0], node[1] + [sucessor[1]], node[2] + sucessor[2]), node[2] + sucessor[2]) # agrego los sucesores del nodo al fringe

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    from game import Directions
    closed = set() # conjunto de nodos visitados para la busqueda en grafo
    fringe = PriorityQueue() # Implementacion del fringe tipo FIFO con prioridad para el UCS
    actions = [Directions.STOP] # lista con las acciones que el agante debe hacer para llegar a cierto estado (sirve para retornar)
    fringe.push(
        (
            problem.getStartState(), 
            actions, 
            0 + heuristic(problem.getStartState(), problem)
        ), 
        0 + heuristic(problem.getStartState(), problem)
        ) # agrego ((estado, [accion inicial = quedate en donde estas], costo acumulado), prioridad)
    # nodo := (estado, lista de acciones para llegar a este estado, costo acumulado)
    # estado := (x,y)
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop() # quita el que tenga menor prioridad
        if problem.isGoalState(node[0]): 
            return node[1] # retorno la lista de acciones que el agente debe hacer para llegar a el goal state
        if node[0] not in closed:
            closed.add(node[0])
            for sucessor in problem.getSuccessors(node[0]): # expando el nodo que recien quite del fringe
                #sucessor := (estado, accion, costo de paso)
                fringe.push(
                    (
                        sucessor[0], 
                        node[1] + [sucessor[1]], 
                        node[2] + sucessor[2] + heuristic(node[0], problem)
                    ), 
                    node[2] + sucessor[2] + heuristic(node[0], problem)
                    ) # agrego los sucesores del nodo al fringe

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
