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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    stack = util.Stack()
    visited = set([])
    # Node has a state, action, and previous node
    node = (problem.getStartState(), None, None)
    stack.push(node)
    cur_node = (problem.getStartState(), None, None)
    
    while True:
        if (stack.isEmpty()):
            return False

        cur_node = stack.pop()
        if (problem.isGoalState(cur_node[0])):
            rev_path = []
            while cur_node:
                if cur_node[1]:
                    rev_path.append(cur_node[1])
                    # Go to Previous Node
                    cur_node = cur_node[2]
                else:
                    rev_path.reverse()
                    return rev_path
        # Check if visited
        elif (cur_node[0] not in visited):
            # Add to visited list
            visited.add(cur_node[0])
            children = problem.getSuccessors(cur_node[0])
            for child in children:
                if(child[0] not in visited):
                    stack.push((child[0], child[1], cur_node))
    return False




    #util.raiseNotDefined()
    #return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    
    que = util.Queue()
    visited = set([])
    # Node has a state, action, and previous node
    node = (problem.getStartState(), None, None)
    que.push(node)
    cur_node = (problem.getStartState(), None, None)
    
    while True:
        if (que.isEmpty()):
            return False

        cur_node = que.pop()
        if (problem.isGoalState(cur_node[0])):
            rev_path = []
            while cur_node:
                if cur_node[1]:
                    rev_path.append(cur_node[1])
                    # Go to Previous Node
                    cur_node = cur_node[2]
                else:
                    rev_path.reverse()
                    return rev_path
        # Check if visited
        elif (cur_node[0] not in visited):
            # Add to visited list
            visited.add(cur_node[0])
            children = problem.getSuccessors(cur_node[0])
            for child in children:
                if(child[0] not in visited):
                    que.push((child[0], child[1], cur_node))
    return False


    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    p_que = util.PriorityQueue()
    visited = set([])
    cost = dict()

    # Node has a state, action, and previous node
    node = (problem.getStartState(), None, None)
    p_que.push(node, 0)
    cost[node[0]] = 0
    cur_node = (problem.getStartState(), None, None)

    while True:
        if (p_que.isEmpty()):
            return False

        cur_node = p_que.pop()
        if (problem.isGoalState(cur_node[0])):
            rev_path = []
            while cur_node:
                if cur_node[1]:
                    rev_path.append(cur_node[1])
                    # Go to Previous Node
                    cur_node = cur_node[2]
                else:
                    rev_path.reverse()
                    return rev_path
        # Check if visited
        elif (cur_node[0] not in visited):
            # Add to visited list
            visited.add(cur_node[0])
            children = problem.getSuccessors(cur_node[0])
            for child in children:
                if(child[0] not in visited):
                    # 3rd index is a 1 for cost
                    new_cost = cost[cur_node[0]] + child[2]
                    p_que.update((child[0], child[1], cur_node), new_cost)
                    cost[child[0]] = new_cost
    return False

    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    p_que = util.PriorityQueue()
    visited = set([])
    cost = dict()

    # Node has a state, action, and previous node
    node = (problem.getStartState(), None, None)
    p_que.push(node, 0)
    cost[node[0]] = 0
    cur_node = (problem.getStartState(), None, None)

    while True:
        if (p_que.isEmpty()):
            return False

        cur_node = p_que.pop()
        if (problem.isGoalState(cur_node[0])):
            rev_path = []
            while cur_node:
                if cur_node[1]:
                    rev_path.append(cur_node[1])
                    # Go to Previous Node
                    cur_node = cur_node[2]
                else:
                    rev_path.reverse()
                    return rev_path
        # Check if visited
        elif (cur_node[0] not in visited):
            # Add to visited list
            visited.add(cur_node[0])
            children = problem.getSuccessors(cur_node[0])
            for child in children:
                if(child[0] not in visited):
                    # 3rd index is a 1 for cost
                    new_cost = cost[cur_node[0]] + child[2]
                    new_cost_h = new_cost + heuristic(child[0], problem)

                    p_que.update((child[0], child[1], cur_node), new_cost_h)
                    cost[child[0]] = new_cost
    return False

    #util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
