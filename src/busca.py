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
In busca.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from lib.searchProblem import SearchProblem

import lib.util as util
from lib.util import Stack, Queue, PriorityQueue
'''
Uso da pilha:
stack = Stack()
stack.push(element)
l = stack.list # Lista de elementos na pilha
e = stack.pop()

Uso da fila:
queue = Queue()
queue.push(element)
l = queue.list # Lista de elementos na fila
e = queue.pop()

Uso da fila de prioridades:
queuep = PriorityQueue()
queuep.push(element, value) # Valores baixos indicam maior prioridade
queuep.update(element, value) # Atualiza o valor de prioridade  de um elemento
l = queuep.heap # Lista de elementos na fila
e = queuep.pop()
'''

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
    visited = []
    path = Stack()
    dfsRecursion(problem, problem.getStartState(), visited, path)
    return path.list

def dfsRecursion(problem, currentState, visited, path):
    visited.append(currentState)
    if problem.isGoalState(currentState):
        return True
    for successor in problem.getSuccessors(currentState):
        if successor[0] not in visited:
            path.push(successor[1])
            if dfsRecursion(problem, successor[0], visited, path):
                return True
    path.pop()
    return False

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    parent = {}
    queue = Queue()
    currentState = (problem.getStartState(), '', '')
    queue.push(currentState)
    visited = [currentState[0]]

    while queue.list:
        currentState = queue.pop()
        if problem.isGoalState(currentState[0]):
            return bfsFindPath(problem.getStartState(), currentState, parent)
        for sucessor in problem.getSuccessors(currentState[0]):
            if sucessor[0] not in visited:
                parent[sucessor[0]] = currentState
                visited.append(sucessor[0])
                queue.push(sucessor)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    parent = {}
    stateAttributes = {}
    priorityQueue = PriorityQueue()
    currentState = (problem.getStartState())
    stateAttributes[currentState] = (problem.getStartState(), '', '0.0')
    priorityQueue.push(currentState, 0)
    visited = [currentState]

    while priorityQueue.heap:
        currentState = priorityQueue.pop()
        if problem.isGoalState(currentState):
            return bfsFindPath(problem.getStartState(), stateAttributes.get(currentState), parent)
        for successor in problem.getSuccessors(currentState):
            queueStates = [row[2] for row in priorityQueue.heap]
            if(successor[0] not in visited) and (successor[0] not in queueStates):
                parent[successor[0]] = stateAttributes.get(currentState)
                stateAttributes[successor[0]] = (successor[0], successor[1], float(successor[2]) + float(stateAttributes.get(currentState)[2])) # Increment path weight
                priorityQueue.push(successor[0], stateAttributes.get(successor[0])[2])
                visited.append(successor[0])
            elif successor[0] in queueStates:
                for state in queueStates:
                    if (state == successor[0]) and (stateAttributes.get(state)[2] > successor[2]):
                        parent[state] = stateAttributes.get(currentState)
                        stateAttributes[successor[0]] = (successor[0], successor[1], successor[2] + stateAttributes.get(currentState)[2])
                        priorityQueue.update(state, stateAttributes.get(successor[0])[2])

def bfsFindPath(start, goal, parent):
    path = Queue()
    node = goal
    while node != None:
        if node[0] != start:
            path.push(node[1])
        node = parent.get(node[0])
    return path.list

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

  