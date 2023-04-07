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

from builtins import object
import util

class SearchProblem(object):
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
    fringe = util.Stack()
    solution = myGraphSearchDfs(problem, fringe)
    return solution

def myOldRecursiveDfs(problem, state, stack, visited, solutionFound):
    """Performs Depth First Search in a recursive manner.\n
    This function is no longer used because a better way to implemented was
    discovered at a later day (thanks graph search!) but it is left here
    because I do not want to undo my hard work (it took me so many hours lmao).\n
    Parameters:
    problem: The problem object.
    state: A state ((x, y) tuple) with coordinates to search from.
    stack: A stack with the current path.
    visited: A list with all the nodes visited so far.
    solutionFound: Boolean that is set to true if a solution is found.
    Returns:
    True if a solution has been found."""
    if state not in visited:
        visited.append(state)
        if problem.isGoalState(state) or solutionFound[0] is True:
            solutionFound[0] = True
            return True
        else:
            # Get all the succesors and check if they've all been visited already.
            x = [s[0] for s in problem.getSuccessors(state)]
            if not set(x).issubset(visited):
                # If not every successor has been visited, visit the ones that haven't.
                for successor in problem.getSuccessors(state):
                    if successor[0] not in visited:
                        stack.append(successor)
                        mybool = mySearch(problem, successor[0], stack, visited, solutionFound)
                        if not mybool and solutionFound[0] is False:
                            stack.pop()
            else:
                return False

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    queue = util.Queue()
    solution = myGraphSearchBfs(problem, queue)
    return solution

def myGraphSearchDfs(problem, fringe):
    """Performs Breadth First Search.\n
    Parameters:
    problem: The problem object.
    fringe: The fringe, a LIFO queue.\n
    Returns:
    A list of directions to get to the goal or None if a solution wasn't found.\n
    The fringe is a tuple of (x, y) coordinates and a list of the
    current set of directions taken so far to get to those coordinates.
    When a solution is found, the tuple is (1, 1) and the list of directions
    is returned.
    """
    # Initialize values, add start node to visited and fringe.
    visited = list()
    state = problem.getStartState()
    fringe.push((state, ['']))
    visited.append(state)
    solutionFound = None

    # Loop until a solution is found or all the nodes were expanded.
    while solutionFound is None:
        if fringe.isEmpty():
            solutionFound = False
            return None
        state, current_route = fringe.pop()
        if problem.isGoalState(state):
            solutionFound = True
            # Return the list of directions without the initialized [''] value.
            return current_route[1:]
        for succesor in problem.getSuccessors(state):
            if succesor[0] not in visited:
                # If successor hasn't been visited then append it to visited.
                # Then make a copy of the route (because they're a reference)
                # and add the new direction from the successor.
                visited.append(succesor[0])
                route = current_route.copy()
                route.append(succesor[1])
                fringe.push((succesor[0], route))

def myGraphSearchBfs(problem, fringe):
    """Performs Breadth First Search.\n
    Parameters:
    problem: The problem object.
    fringe: The fringe, a LIFO queue.\n
    Returns:
    A list of directions to get to the goal or None if a solution wasn't found.\n
    The fringe is a tuple of (x, y) coordinates and a list of the
    current set of directions taken so far to get to those coordinates.
    When a solution is found, the tuple is (1, 1) and the list of directions
    is returned.
    """
    # Initialize values, add start node to visited and fringe.
    visited = list()
    state = problem.getStartState()
    fringe.push((state, ['']))
    visited.append(state)
    solutionFound = None

    # Loop until a solution is found or all the nodes were expanded.
    while solutionFound is None:
        if fringe.isEmpty():
            solutionFound = False
            return None
        state, current_route = fringe.pop()
        if problem.isGoalState(state):
            solutionFound = True
            # Return the list of directions without the initialized [''] value.
            return current_route[1:]
        for succesor in problem.getSuccessors(state):
            if succesor[0] not in visited:
                # If successor hasn't been visited then append it to visited.
                # Then make a copy of the route (because they're a reference)
                # and add the new direction from the successor.
                visited.append(succesor[0])
                route = current_route.copy()
                route.append(succesor[1])
                fringe.push((succesor[0], route))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    fringe = util.PriorityQueue()
    solution = myGraphSearchUcs(problem, fringe)
    return solution

def myGraphSearchUcs(problem, fringe):
    """Performs Uniform Cost Search.\n
    Parameters:
    problem: The problem object.
    fringe: The fringe, a priority queue.\n
    Returns:
    A list of directions to get to the goal or None if a solution wasn't found.\n
    The fringe is a tuple of (x, y) coordinates, a list of the
    current set of directions and the total cost of the route taken so far
    to get to those coordinates.
    When a solution is found, the tuple is (1, 1) and the list of directions
    is returned.
    """
    # Initialize values, add start node to visited and fringe.
    visited = list()
    state = problem.getStartState()
    fringe.push((state, [''], 0), 0)
    visited.append(state)
    solutionFound = None

    # Loop until a solution is found or all the nodes were expanded.
    while solutionFound is None:
        if fringe.isEmpty():
            solutionFound = False
            return None
        state, current_route, current_cost = fringe.pop()
        if problem.isGoalState(state):
            solutionFound = True
            # Return the list of directions without the initialized [''] value.
            return current_route[1:]
        for succesor in problem.getSuccessors(state):
            if succesor[0] not in visited:
                # If successor hasn't been visited then append it to visited.
                # Then make a copy of the route (because they're a reference)
                # and add the new direction from the successor.
                # Also calculate new 
                visited.append(succesor[0])
                route = current_route.copy()
                route.append(succesor[1])
                cost = current_cost + succesor[2]
                fringe.push((succesor[0], route, cost), cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
