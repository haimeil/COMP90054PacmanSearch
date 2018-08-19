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

    # initial open and closed lists
    open = Stack()
    closed = []

    # get start state into the open list
    open.push((problem.getStartState(), []))

    while not open.isEmpty():

        current_node, actions = open.pop()

        # if current state is the goal state
        # return list of actions
        if problem.isGoalState(current_node):
            return actions

        if current_node not in closed:
            # expand current node
            # add current node to closed list
            expand = problem.getSuccessors(current_node)
            closed.append(current_node)
            for location, direction, cost in expand:
                # if the location has not been visited, put into open list
                if (location not in closed):
                    open.push((location, actions + [direction]))

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    from game import Directions

    # initial open and closed lists
    open = Queue()
    closed = []

    # get start state into the open list
    open.push((problem.getStartState(), []))

    while not open.isEmpty():

        current_node, actions = open.pop()

        # if current state is the goal state
        # return list of actions
        if problem.isGoalState(current_node):
            return actions

        if current_node not in closed:
            # expand current node
            # add current node to closed list
            expand = problem.getSuccessors(current_node)
            closed.append(current_node)
            for location, direction, cost in expand:
                # if the location has not been visited, put into open list
                if (location not in closed):
                    open.push((location, actions + [direction]))

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # state [location, direction, cost]
    # prioritize by cost
    queue = util.PriorityQueueWithFunction(lambda x: x[2])

    # lists of state which have been visited
    visited = []

    # list of actions from start to goal
    actions = []

    cost = 0

    start = problem.getStartState()
    queue.push((start, None, cost))

    # parents is a dict with key[state, direction] value[parent state, direction of parent ]
    parents = {}
    parents[(start, None, 0)] = None

    while not queue.isEmpty():

        current_node = queue.pop()

        # if current state is the goal state
        if problem.isGoalState(current_node[0]):
            break
        else:
            current_node_state = current_node[0]
            if current_node_state not in visited:
                visited.append(current_node_state)
            else:
                continue

            expand = problem.getSuccessors(current_node_state)
            for state in expand:
                cost = current_node[2] + state[2]

                # if the location has not been visited, put into open list
                if (state[0] not in visited):
                    queue.push((state[0], state[1], cost))
                    parents[(state[0], state[1])] = current_node

    child = current_node

    while (child != None):
        actions.append(child[1])
        if child[0] != start:
            child = parents[(child[0], child[1])]
        else:
            child = None
    actions.reverse()
    return actions[1:]

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




    # lists of state which have been visited
    from util import Queue
    from game import Directions

    # initial open and closed lists
    open = util.PriorityQueue()
    closed = []
    cost = 0
    actions = []
    # get start state into the open list
    open.push((problem.getStartState(), []), 0)

    while not open.isEmpty():

        current_node, actions = open.pop()

        # if current state is the goal state
        # return list of actions
        if problem.isGoalState(current_node):
            return actions

        if current_node not in closed:
            # expand current node
            # add current node to closed list
            expand = problem.getSuccessors(current_node)
            closed.append(current_node)
            for location, direction, cost0 in expand:
                # if the location has not been visited, put into open list
                cost = problem.getCostOfActions(actions +[direction]) + heuristic(location,problem)
                if (location not in closed):
                    open.push((location, actions + [direction]),cost)

    return actions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
