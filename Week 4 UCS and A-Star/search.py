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
import numpy as np


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
    return [s, s, w, s, w, w, s, w]


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
    stack = util.Stack()
    return search_structure(stack,problem)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue()
    return search_structure(queue,problem)


def search_structure(structure, problem):
    """
    Searches the data structure for a solution to the given problem for Depth-
    First-Search and Breadth-First-Search algorithms where a start position and
    a goal position is given. The output is a path from start to goal.
    :param structure: Data structure that is a queue or stack
    :param problem: Problem with a given start and goal position to reach
    :return: The list of a path found by algorithms or empty list if none
    """
    start_position = problem.getStartState()

    path = []
    path_cost = 0
    visited = []

    #Format start position like successors before push
    structure.push(((start_position,path,path_cost), path, path_cost))

    #loop through structure to find paths to goal
    while not structure.isEmpty():
        item = structure.pop()
        state_info = item[0]
        position = state_info[0]
        path = np.append(item[1], state_info[1])
        path_cost = item[2] + state_info[2]

        if position not in visited:
            visited.append(position)

            # only add successors if not visited and not goal
            if (not problem.isGoalState(position)):
                successors = problem.getSuccessors(position)
                for member in successors:
                    structure.push((member, path, path_cost))
            else:
                return path.tolist()

    # did not find a path
    return [-1]


def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    Algorithm for search is placed in A* search given that the only difference
    between the two searches is that one inputs an additional heuristic
    function for determining priority order while uniform does not (or null
    heuristic where h(x) = 0).
    """
    return aStarSearch(problem, nullHeuristic)

def inFrontier(position,frontier):
    while (not frontier.isEmpty()):
        node = frontier.pop()
        if (node.isSame(position)):
            return True
    return False


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    Pseudocode for uniform cost search algorithm was obtained from the book.
    """
    start_position = problem.getStartState()
    priority_queue = util.PriorityQueue()
    path = []
    path_cost = 0
    visited = []

    # Format start position as a node before pushing
    start_node = Node(start_position,path,path_cost)
    priority_queue.push(start_node, start_node.path_cost +
                        heuristic(start_position, problem))

    #loop through queue to find best path (cheapest path first)
    while not priority_queue.isEmpty():
        node = priority_queue.pop()
        position = node.position
        current_path_taken = node.path
        current_path_cost = node.path_cost
        if problem.isGoalState(position):
            return current_path_taken.tolist()

        visited.append(position)

        successors = problem.getSuccessors(position)
        for member in successors:
            position = member[0]
            if position not in visited:
                path = np.append(current_path_taken, member[1])
                path_cost = current_path_cost + member[2]
                node = Node(position, path, path_cost)
                #update replaces node if duplicate/better cost or queues if not
                priority_queue.update(node, path_cost +
                                      heuristic(position, problem))

    # did not find a path
    return [-1]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

class Node:
    '''
    Helper class to represent the nodes in the search algorithms.
    '''
    def __init__(self, position, path, path_cost):
        self.position = position
        self.path = path
        self.path_cost = path_cost
    #override to do equal comparison between nodes to look at position
    def __eq__(self, obj):
        return self.position == obj.position