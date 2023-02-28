#!/usr/bin/env python
"""
An A* Search implementation using an admissible heuristic.

This implementation finds the cost-optimal path from start to goal_state 
using the Manhattan distance, if a solutions exists. A random world generator
is also included, as well as a default "full world" example. 
"""
__author__ = "CJ McGinnis"
__version__ = "1.0.0"
__maintainer__ = "CJ McGinnis"
__email__ = "cmcginn5@jh.edu"
__status__ = "Beta"

full_world = [
['🌾', '🌾', '🌾', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🗻', '🗻', '🗻', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🪨', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨'],
['🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🐊', '🐊', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🌾'],
['🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🌲', '🌲', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🪨', '🌾'],
['🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🪨', '🌾', '🌾'],
['🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🌾', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🐊', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🪨', '🪨', '🗻', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🌾', '🌾', '🪨', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🪨', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🌲', '🌲', '🪨', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🗻', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🪨', '🪨', '🪨', '🪨', '🌾', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🪨', '🪨', '🌾', '🐊', '🌾', '🪨', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🌾', '🌾', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🌲', '🌲', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🪨', '🪨', '🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🪨', '🗻', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🪨', '🪨', '🗻', '🗻', '🌾', '🗻', '🗻', '🪨', '🪨', '🐊', '🐊', '🐊', '🐊'],
['🪨', '🗻', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🗻', '🗻', '🪨', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🐊', '🐊', '🐊', '🐊'],
['🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾']
]

small_world = [
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾']
]

IMPASS_LIST = ["🗻" , "🌋", "🌊"]
MOVES = [(0,-1), (1,0), (0,1), (-1,0)]
COSTS = { '🌾': 1, '🌲': 3, '🪨': 5, '🐊': 7}

from typing import List, Tuple, Dict, Callable, Any
from copy import deepcopy


def heuristic(state: tuple[int, int], goal_state: tuple[int,int], 
              cost_estimate: int) -> int:
    """
    Provides a cost estimate of the cheapest path from start to goal_state 
    using the Manhattan distance. This is one component of the evaluation 
    function used in the A* search algorithm. If cost_estimate is less than or
    equal to the minimum cost of any location in the world, the heuristic will
    be optimistic. Used by: create_node
  
    Parameters
    ----------
    state: tuple[int, int]
        a tuple representing the [column, row] location of the node to be 
        evalutated in the world.
    goal_state: tuple[int, int]
        a tuple representing the [column, row] location of the goal in the 
        world.
    cost_estimate: int
        the cost value added to the estimate for each location moved towards the
        goal. If this value is less than or equal to all location costs in the 
        world, the heuristic will be optimistic. Costs must be greater than
        zero.
        
    Returns
    -------
    int: the cost estimate of moving from start to goal using the Manhattan 
    distance.
    
    """
    return (abs(state[0]-goal_state[0]) * cost_estimate
            + abs(state[1]-goal_state[1]) * cost_estimate)


def create_node(
        previous_node: dict[str, Any], node_state: tuple[int, int], 
        start_state: tuple[int, int], goal_state: tuple[int, int], 
        costs: dict[str, int], world: list[list[str]]) -> dict[str, Any]:
    """
    Returns a node corresponding to the state (location) in the world. The 
    metadata contained in the node is important for prioritzing the frontier and generating the path to the goal. Uses: heuristic
  
    Parameters
    ----------
    previous_node: dict[str, Any]
        a dictionary with str keys for the cumulative path cost ('path cost'),
        evaluation function value ('evaluation'), the state ('state'), and the
        previous node in the current path ('previous_node'). This represents the
        previous node from which the current node_state was expanded.
    node_state: tuple[int, int]
        a tuple representing the [column, row] location of the node to be 
        created in the world. Must be a valid state in world.
    start_state: tuple[int, int]
        a tuple representing the [column, row] location of the starting location
        in the world. Must be a valid state in world. Used to determine 
        whether the node should have a cost value (start state has no cost).
    goal_state: tuple[int, int]
        a tuple representing the [column, row] location of the goal in the 
        world.
    costs: dict[str, int]
        a dictionary of character representations for locations in the world 
        and their associated costs.
    world: list[list[str]]
        a representation of the world using a nested list of character values 
        for each location.
        
    Returns
    -------
    dict[str, Any]: a dictionary with str keys to the cumulative path cost 
    ('path cost'), evaluation function value ('evaluation'), the state 
    ('state'), and the previous node in the current path ('previous_node'). 
    Returns None if the node_state corresponds to a location with an invalid 
    terrain type or terrain type that cannot be visited.
    
    """
    terrain_type = world[node_state[1]][node_state[0]]
    if(terrain_type not in costs):
        return None
    prev_cost = 0 if previous_node is None else previous_node["path_cost"]
    path_cost = (0 if node_state == start_state else prev_cost 
                 + costs[terrain_type])
    return {
        "path_cost" : path_cost,
        "evaluation" : heuristic(
            node_state, goal_state, min(costs.values())) + path_cost,
        "previous_node" : previous_node,
        "state" : node_state}

def transition_model(
        current_state: tuple[int, int], move: tuple[int, int], 
        world: list[list[str]], costs: dict[str, int]) -> tuple[int,int]:
    """
    Applies a move (action) to the current_state and returns the resulting state
    or None if the move is not valid in the 'world'. Uses: None. Used by: 
    successors.
  
    Parameters
    ----------
    current_state: tuple[int, int]
        a tuple representing the [column, row] location of the current state in 
        the world. Must be a valid state in world. 
    move: tuple[int,int]
        a tuple representing the action to apply to the current_state in
        (column-offset, row-offset) format.
    world: list[list[str]]
        a representation of the world using a nested list of character values 
        for each location.
    costs: dict[str, int]
        a dictionary of character representations for locations in the world 
        and their associated costs.
        
    Returns
    -------
    tuple[int,int]: a tuple representing the [column, row] location for the 
    derived state after appling move. Returns None if the move would result in 
    an invalid state in the world.
    
    """
    x = current_state[0] + move[0]
    y = current_state[1] + move[1] 
    height = len(world)
    if (current_state[1] < 0 or current_state[1] > height - 1 or 
        y < 0 or y > height - 1):
        return None
    row_width = len(world[y]) 
    if (current_state[0] < 0 or current_state[0] > row_width - 1 or 
        x < 0 or x > row_width - 1):
        return None
    if world[y][x] not in costs.keys():
        return None
    return (x, y)

def successors(
        node: dict[str, Any], world: list[list[str]], start_state: tuple[int, int], 
        goal_state: tuple[int, int], moves: list[tuple[int,int]], 
        costs: dict[str, int]) -> list[dict[str, Any]]:
    """
    Expands the provided node and returns a list of valid successor nodes using 
    the transition model and moves (actions). Uses: create_node and 
    transition_model. Used by: a_star_search.
  
    Parameters
    ----------
    node: dict[str, Any]
        a dictionary with str keys for the cumulative path cost ('path cost'),
        evaluation function value ('evaluation'), the state ('state'), and the
        previous node in the current path ('previous_node'). This represents the
        current node to be expanded. 
    world: list[list[str]]
        a representation of the world using a nested list of character values 
        for each location.
    start_state: tuple[int, int]
        a tuple representing the [column, row] location of the starting location
        in the world. Must be a valid state in world. Used to determine 
        whether the node should have a cost value (start state has no cost).
    goal_state: tuple[int, int]
        a tuple representing the [column, row] location of the goal in the 
        world.    
    moves: list[tuple[int,int]]
        a list of tuples representing the available actions in (row-offset, 
        column-offset) format.       
    costs: dict[str, int]
        a dictionary of character representations for locations in the world 
        and their associated costs.
        
    Returns
    -------
    list[dict[str, Any]]: a list of nodes, each represented by a dictionary with
    str keys to the cumulative path cost ('path cost'), evaluation function 
    value ('evaluation'), the state ('state'), and the previous node in the 
    current path ('previous_node'). Returns an empty list if the node to be 
    expanded has no valid successors.
    """
    new_nodes = []
    for action in moves:
        next_state = transition_model(node["state"], action, world, costs)
        if next_state is None:
            continue
        new_nodes.append(create_node(node, next_state, start_state, goal_state, 
                                     costs, world))
    return new_nodes

def get_path(node: dict[str, Any], 
             terrain: bool = False) -> list[tuple[int,int]]: 
    """
    Generates the path to the node by traversting the previous_node entries. 
    This is used to generate the path once the goal state is found. The returned
    path is represented by a list of moves to be applied to the starting state. 
    Uses: None. Used by: a_star_search.
  
    Parameters
    ----------
    node: dict[str, Any]
        a dictionary with str keys for the cumulative path cost ('path cost'),
        evaluation function value ('evaluation'), the state ('state'), and the
        previous node in the current path ('previous_node'). This represents 
        the node to which the path should be generated.       
    terrain: bool
        whether to return the terrain associated with each movement.
        
    Returns
    -------
    list[tuple[int,int]]: a list of tuples representing the [column offset, row offest] moves to apply to the start state to arrive at the node. Note: if
    terrain is True, then a tuple of lists is returned as (moves, terrain).
    """
    path = []
    terr = []
    while(node["previous_node"] is not None):
        prev_state = node["previous_node"]["state"]
        curr_state = node["state"]
        path.append((curr_state[0]-prev_state[0], curr_state[1] 
                     - prev_state[1]))
        terr.append(curr_state)
        node = node["previous_node"]
    path.reverse()
    terr.reverse()
    return (path,terr) if terrain else path

from bisect import bisect_left
def priority_queue_insert(priority_queue: list[dict[str, Any]], 
                          entry: dict[str,Any], 
                          priority_key: str) -> list[dict[str, Any]]:
    """
    Returns a copy of the priority_queue after inserting the new_entry node into
    the copy using the priority_key in ascending order. This is important for representing the frontier. The priority queue is implemented using a list of dictionaries, and the order is maintained using the specified priority key, 
    which is a string key to the dictionary entries. If the an entry was already present with the same priority value, the new entry is inserted before the previously added entries. The priority_queue must be ordered prior (this 
    function does not sort the queue). Uses: None. Used by:  a_star_search.
  
    Parameters
    ----------
    priority_queue: list[dict[str, Any]]
        a list of dictionaries sorted in order of priority_key.
    priority_key: str
        a string of the key used for priority ordering of the queue. Must be one of the valid keys for the given dictionary structure.
        
    Returns
    -------
    list[dict[str, Any]]: a copy of the priority_queue with the entry inserted according to priority_key. 
    """
    index = bisect_left(priority_queue, entry[priority_key], 
                        key=lambda entry:entry[priority_key])
    updated_priority_queue = deepcopy(priority_queue)
    updated_priority_queue.insert(index, entry)
    return updated_priority_queue


def priority_queue_pop(
        priority_queue: list[dict[str, Any]]) -> tuple[dict, list[dict]]:
    """
    Creates a copy of the priority_queue and removes the first entry. The 
    removed entry and an updated copy of the priority_queue are returned as a 
    tuple. This is important for obtaining the next node to explore from the frontier. Uses: None. Used by: a_star_search.
  
    Parameters
    ----------
    priority_queue: list[dict[str, Any]]
        a list of dictionaries sorted in order of priority_key.
  
    Returns
    -------
    tuple[dict[str, Any], list[dict[str, Any]]]: a tuple where the first value 
    is the entry removed from the priority_queue and the second value is a copy 
    of the priority_queue with the entry removed. 
    """
    updated_priority_queue = deepcopy(priority_queue)
    if(len(priority_queue) == 0):
        entry = None
    else:
        entry = updated_priority_queue.pop(0)
    return entry, updated_priority_queue

def a_star_search(
        world: List[List[str]], start: Tuple[int, int], goal: Tuple[int, int], 
        costs: Dict[str, int], moves: List[Tuple[int, int]], 
        heuristic: Callable, terrain: bool = False) -> List[Tuple[int, int]]:
    """
    Using an implementation of the A* search algorithm, a_star_search uses the
    provided heuristic and the costs to find a path from the start location to 
    the goal location if one exists in the world. This is accomplished by
    prioritizing the nodes on the frontier based on minimizing the evaluation
    function f(n) = g(n) + h(n), where g(n) is the cumulative path cost to 
    the node (n), and h(n) is the heuristic estimate of the path from the 
    node (n) to the goal. If the heuristic is optimistic, the solution will be
    cost optimal. If a path exists, the list of moves (actions) required to 
    traverst the path from the start location is returned. Uses: create_node, successors, get_path, priority_queue_insert, and priority_queue_pop. Used 
    by: None (main program)
  
    Parameters
    ----------
    world: list[list[str]]
        the actual context for the navigation problem.
    start: tuple[int, int]
        the starting location of the bot, (x, y).
    goal: tuple[int, int]
        the desired goal position for the bot, (x, y). 
    moves: list[tuple[int,int]]
        the legal movement model expressed in offsets in world.       
    heuristic: Callable
        is a heuristic function, h(n)
    terrain: bool
        whether to return the terrain associated with each movement.
        
    Returns
    -------
    List[Tuple[int, int]]: the offsets needed to get from start state to the 
    goal as a List, or None if no solution exists. Note: if terrain is True, 
    then a tuple of lists is returned as (moves, terrain).
    """
    curr_node = create_node(None, start, start, goal, costs, world)
    frontier = [curr_node]
    reached = {start : curr_node}
    priority_key = "evaluation"
    while len(frontier) != 0:
        curr_node, frontier = priority_queue_pop(frontier)
        if curr_node["state"] == goal:
            return get_path(curr_node, terrain)
        for child in successors(curr_node, world, start, goal, moves, costs):
            state = child["state"]
            if ((state not in reached) 
                or (child[priority_key] < reached[state][priority_key])):
                reached[state] = child
                frontier = priority_queue_insert(frontier, child, priority_key)
    return None

def pretty_print_path(
        world: List[List[str]], path: List[Tuple[int, int]], 
        start: Tuple[int, int], goal: Tuple[int, int], costs: dict[str, int], 
        line_char: str = '\n\r', print_out: bool = True) -> tuple[int,str]:
    """
    The pretty_print_path function prints a character representation of the 
    path generated by the a_star_search on top of the world terrain map using 
    ⏩,⏪,⏫ ⏬ to represent actions and 🎁 to represent the goal. The total cost 
    of the path is returned. If no path is provied, the map will be printed 
    wihtout a path and ▶️ will show the start location. The path must be valid in
    the world (this function does not check boundaries or conditions). 
    Uses: None. Used by: None (main program).
  
    Parameters
    ----------
    world: list[list[str]]
        the world (terrain map) for the path to be printed upon.
    path: list[tuple[int,int]]
        the path from start to goal, in offsets.
    start: tuple[int, int]
        the starting location for the path.
    goal: tuple[int, int]
        the goal location for the path. 
    costs: dict[str, int] 
        the costs for each action.      
    line_char: str
        the characters used as new line
    print_out: bool
        whether to print the path
        
    Returns
    -------
    tuple[int,str]: the path cost and the string representation of the path.  
    """
    curr = start
    new_map = deepcopy(world)
    move_symbols = {(1,0): "⏩", (-1,0): "⏪", (0,1): "⏬", (0,-1): "⏫"}
    goal_symbol="🎁"
    start_symbol = "🏁"
    path_cost = 0
    if(path is not None and path != []):
        for move in path:
            new_map[curr[1]][curr[0]] = move_symbols[move]
            curr = (curr[0] + move[0], curr[1] + move[1])
            terrain = world[curr[1]][curr[0]]
            if(terrain in costs):
                path_cost = path_cost + costs[terrain]
    else:
        new_map[start[1]][start[0]] = start_symbol   
    new_map[goal[1]][goal[0]] = goal_symbol
    str_map = line_char.join(["".join(row) for row in new_map])
    if(print_out): 
        print(str_map)
    return path_cost, str_map

def print_map(
        world: List[List[str]], marks: list[tuple[tuple[int, int],str]] = None, 
        line_char: str = '\n', print_out: bool = True) -> str:
    """
    Prints a character representation of the map. If marks are provides, they 
    will be printed over the map at the corresponding locations. Uses: None. 
    Used by: None (main program).
  
    Parameters
    ----------
    world: list[list[str]]
        the world (terrain map) to be printed.
    marks: list[tuple[tuple[int,int],str]]
        the list of marks to print overtop of the map.     
    line_char: str
        the characters used as new line
    print_out: bool
        whether to print the path
        
    Returns
    -------
    str: a printable string representation of the world map 
    """
    if(marks is None or len(marks) == 0):
        str_map = line_char.join(["".join(row) for row in world])
    else:
        new_map = deepcopy(world)
        for m in marks:
            new_map[m[0][1]][m[0][0]] = m[1]
        str_map = line_char.join(["".join(row) for row in new_map])
    if(print_out):
        print(str_map)
    return str_map  
 
import random

def set_impassible(world: list[list[str]], impass_freq: float, width: int, 
                   height: int, impassible: list[str]) -> list[list[str]]: 
    """
    Sets a provided percentage of the world locations to impassible terrain 
    types. Uses: None. Used by: create_random_world.
  
    Parameters
    ----------
    world: list[list[str]]
        the world terrain map to alter
    impass_freq: float
        the percentage of the map to set to impassible terrain types (prior to 
        any adjustments to create a valid map).
    width: int
        the width of the map
    height: int
        the height of the map
    impassible: list[str]
        a list of strings representing impassible location types
        
    Returns
    -------
    list[list[str]]: a altered version of world with impassible locations added
    """
    impass_locations = []
    width_range = list(range(0,width))
    height_range = list(range(0,height))
    impass_freq = min(impass_freq, 1)
    num_impass = round(impass_freq * width * height)
    
    for i in range(0, num_impass):
        rand_impass = (random.choice(width_range), random.choice(height_range))
        while rand_impass in impass_locations:
            rand_impass = (random.choice(width_range), 
                           random.choice(height_range))
        impass_locations.append(rand_impass)
        world[rand_impass[1]][rand_impass[0]] = random.choice(impassible)
    return world

def create_path(
        world: list[list[str]], start: tuple[int,int], goal: tuple[int,int], 
        costs: dict[str, int], height: int, width: int) -> list[list[str]]: 
    """
    Creates a path from start to goal to ensure that the map has a valid path. 
    Uses: None. Used by: create_random_world.
  
    Parameters
    ----------
    world: list[list[str]]
        the world terrain map to alter
    start: tuple[int, int]
        the starting location for the path.
    goal: tuple[int, int]
        the goal location for the path. 
    costs: dict[str, int] 
        the costs for each action - used to generate random movements. 
    height: int
        the height of the map
    width: int
        the width of the map
        
    Returns
    -------
    list[list[str]]: a altered version of world with a random path from the 
    start to goal.
    """
    loc = [start[1], start[0]]
    direction = [goal[1] - start[1], goal[0] - start[0]]
    direction[0] = -1 if direction[0] < 0 else 1
    direction[1] = -1 if direction[1] < 0 else 1
    while loc[0] != goal[1] or loc[1] != goal[0]:
        next_dir = random.choice([0,1])
        if(next_dir == 0 and loc[next_dir] == goal[1]):
            next_dir = 1
        if(next_dir == 1 and loc[next_dir] == goal[0]):
            next_dir = 0
        loc[next_dir] = loc[next_dir] + direction[next_dir]
        world[loc[0]][loc[1]] = random.choice(list(costs.keys()))
    return world     

def create_random_world(
        height: int, width: int, costs: dict[str, int], impassible: list[str], 
        impass_freq: float, start: tuple[int,int] = None, 
        goal: tuple[int,int] = None, make_path: bool = True) -> list[list[str]]:
    """
    Creates a randomized world that conforms to the specifications provided. 
    Uses: None. Used by: None (main program).
  
    Parameters
    ----------
    height: int
        the height of the map to be created
    width: int
        the width of the map to be created
    costs: dict[str, int] 
        the costs for each action - used to generate random locations
    impassible: list[str]
        a list of strings representing impassible location types
    impass_freq: float
        the percentage of the map to set to impassible terrain types (prior to 
        any adjustments to create a valid map).
    start: tuple[int, int]
        the starting location for the path.
    goal: tuple[int, int]
        the goal location for the path. 
    make_path: bool
        whether to ensure a valid path exists from start to goal.
         
    Returns
    -------
    list[list[str]]: a randomized world grid that conforms to the specifications
    provided.
    """
    world = [[random.choice(list(costs.keys())) for _ in range(width)] 
             for _ in range(height)]
    set_impassible(world, impass_freq, width, height, impassible)
    start = (0,0) if start is None else start
    goal = (width-1, height-1) if goal is None else goal

    if(start is not None):
        world[start[1]][start[0]] = random.choice(list(costs.keys()))
    if(goal is not None):
        world[goal[1]][goal[0]] = random.choice(list(costs.keys()))
    if(make_path):
        world = create_path(world, start, goal, costs, height, width)
    
    return world
