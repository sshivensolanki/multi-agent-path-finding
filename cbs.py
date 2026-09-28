import random
import time as timer
import heapq
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

import concurrent.futures
import copy

DEBUG = False


def normalize_paths(pathA, pathB):
    
    """
    Normalize two paths to equal length by extending the shorter one
    with its final position.

    This ensures both paths can be compared timestep-by-timestep by
    assuming that once a robot reaches its goal, it remains there.

    Args:
        pathA (list): First path.
        pathB (list): Second path.

    Returns:
        tuple: (pathA, pathB) where both paths have equal length.
    """

    path1 = pathA.copy()
    path2 = pathB.copy()
    shortest, pad = (path1, len(path2) - len(path1)) if len(path1) < len(path2) else (path2, len(path1) - len(path2))
    for _ in range(pad):
        shortest.append(shortest[-1])
    return path1, path2


def detect_collision(pathA, pathB):
    
    """
    Detect the earliest collision between two robot paths.

    A collision can be one of two types:

    1. Vertex collision:
       Both robots occupy the same location at the same timestep.

    2. Edge collision:
       The robots traverse the same edge in opposite directions
       during the same timestep (i.e., they swap positions).

    Paths are normalized before comparison so that collision checking
    continues even after one robot has reached its goal. A robot that
    reaches its destination is assumed to remain at its final location
    for all subsequent timesteps.

    Args:
        pathA (list): Sequence of locations representing the first robot's path.
        pathB (list): Sequence of locations representing the second robot's path.

    Returns:
        tuple | None:
            - For a vertex collision:
                ([location], timestep, 'vertex')
            - For an edge collision:
                ([from_location, to_location], timestep, 'edge')
            - None if no collision exists.
    """

    path1, path2 = normalize_paths(pathA, pathB)
    length = len(path1)
    for t in range(length):
        # check for vertex collision
        pos1 = get_location(path1, t)
        pos2 = get_location(path2, t)
        if pos1 == pos2:
            # return the vertex and the timestep causing the collision
            return [pos1], t, 'vertex'
        # check for edge collision (not if we are in the last timestep)
        if t < length - 1:
            next_pos1 = get_location(path1, t + 1)
            next_pos2 = get_location(path2, t + 1)
            if pos1 == next_pos2 and pos2 == next_pos1:
                # return the edge and timestep causing the collision
                return [pos1, next_pos1], t + 1, 'edge'
    return None

def detect_collisions(paths):

    """
    Detect collisions between all pairs of robot paths.

    For each pair of robots, the earliest collision is identified and
    recorded. 

    Args:
        paths (list): A list of robot paths.

    Returns:
        list: A list of collision dictionaries containing the robot
        indices, collision location(s), timestep, and collision type.
        Returns an empty list if no collisions are found.
    """

    collisions = []
    normalized_pairs = {}  # store precomputed normalized paths

    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            # precompute normalization for this pair if not already done
            if (i, j) not in normalized_pairs:
                normalized_pairs[(i, j)] = normalize_paths(paths[i], paths[j])
            
            path1, path2 = normalized_pairs[(i, j)]
            # detect collision using precomputed normalized paths
            length = len(path1)
            for t in range(length):
                pos1 = get_location(path1, t)
                pos2 = get_location(path2, t)
                if pos1 == pos2:
                    collisions.append({'a1': i, 'a2': j, 'loc': [pos1], 'timestep': t, 'type': 'vertex'})
                    break
                if t < length - 1:
                    next_pos1 = get_location(path1, t + 1)
                    next_pos2 = get_location(path2, t + 1)
                    if pos1 == next_pos2 and pos2 == next_pos1:
                        collisions.append({'a1': i, 'a2': j, 'loc': [pos1, next_pos1], 'timestep': t + 1, 'type': 'edge'})
                        break
    return collisions


def standard_splitting(collision):
    """
    Generate the standard pair of constraints for resolving a collision.

    Given a collision between two agents, this function produces two
    constraints, one for each agent, that prohibit the conflicting
    vertex or edge at the collision timestep.

    Args:
        collision (dict): Collision information containing the agents
            involved, location(s), timestep, and collision type.

    Returns:
        list: A list of two constraint dictionaries, one for each agent.
    """
    constraints = []
    if collision['type'] == 'vertex':
        constraints.append({
            'agent': collision['a1'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
        constraints.append({
            'agent': collision['a2'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
    elif collision['type'] == 'edge':
        constraints.append({
            'agent': collision['a1'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
        constraints.append({
            'agent': collision['a2'],
            # convert to a list explicitly, since comparing a list to an
            # iterator evaluates to False without raising an error
            'loc': list(reversed(collision['loc'])),
            'timestep': collision['timestep'],
            'final': False
        })
    return constraints


def disjoint_splitting(collision):
    
    """
    Generate a pair of disjoint constraints for a collision.

    A random agent involved in the collision is selected. The returned
    constraints consist of one positive constraint that enforces the
    collision behavior and one negative constraint that prohibits it.

    Args:
        collision (dict): Collision information.

    Returns:
        list: Two constraints for the selected agent, one positive and
        one negative.
    """

    choice = random.randint(0, 1)
    agents = [collision['a1'], collision['a2']]
    agent = agents[choice]
    loc = collision['loc'] if choice == 0 else list(reversed(collision['loc']))
    return [
        {
            'agent': agent,
            'loc': loc,
            'timestep': collision['timestep'],
            'positive': True,
            'final': False
        },
        {
            'agent': agent,
            'loc': loc,
            'timestep': collision['timestep'],
            'positive': False,
            'final': False
        }
    ]


def paths_violate_constraint(constraint, paths):
    
    """
    Return the other agents whose paths collide with the given positive constraint.

    Depending on the constraint type, the check is delegated to either
    vertex_check() or edge_check(). The constrained agent itself is never
    reported, since its path is required to satisfy the constraint.

    Args:
        constraint (dict): Positive constraint to evaluate.
        paths (list): Collection of agent paths.

    Returns:
        list: Indices of agents that violate the constraint.
    """

    if len(constraint['loc']) == 1:
        return vertex_check(constraint, paths)
    else:
        return edge_check(constraint, paths)

def vertex_check(constraint, paths):
    agents_violate = []
    for agent in range(len(paths)):
        if agent == constraint['agent']:
            continue
        if constraint['loc'][0] == get_location(paths[agent], constraint['timestep']):
            agents_violate.append(agent)
    return agents_violate

def edge_check(constraint, paths):
    agents_violate = []
    u, v = constraint['loc']
    for agent in range(len(paths)):
        if agent == constraint['agent']:
            continue
        prev_loc = get_location(paths[agent], constraint['timestep'] - 1)
        loc = get_location(paths[agent], constraint['timestep'])
        # occupying either endpoint of the edge, or traversing it in the opposite direction
        if prev_loc == u or loc == v or (prev_loc == v and loc == u):
            agents_violate.append(agent)
    return agents_violate


def negative_constraints_for(constraint, agent):
    """
    Convert a positive constraint into the negative constraints it implies
    for another agent.

    If one agent must be at vertex v at timestep t, no other agent may be
    at v at t. If one agent must traverse edge (u, v) at timestep t, no
    other agent may be at u at t - 1, be at v at t, or traverse (v, u) at t.

    Args:
        constraint (dict): Positive constraint on some agent.
        agent (int): The other agent to constrain.

    Returns:
        list: Negative constraints for the given agent.
    """

    t = constraint['timestep']
    if len(constraint['loc']) == 1:
        blocked = [(constraint['loc'], t)]
    else:
        u, v = constraint['loc']
        blocked = [([u], t - 1), ([v], t), ([v, u], t)]
    return [{'agent': agent, 'loc': loc, 'timestep': timestep, 'positive': False, 'final': False}
            for loc, timestep in blocked]



class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals, max_time=None):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.start_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.max_time =  max_time if max_time else float('inf')

        self.open_list = []
        self.cont = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        if DEBUG:
            print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        if DEBUG:
            print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node
    
    def expand_child_node(self, parent_node, constraint):
        """
        Generate a child node by applying a single constraint to a parent node.
        Returns the new node or None if no valid path exists.
        """
        skip_node = False
        q = {
            'cost': 0,
            'constraints': list(parent_node['constraints']) + [constraint],  # create a new constraint set for the child node
            'paths': copy.deepcopy(parent_node['paths']),  # preserve parent paths while allowing modifications
            'collisions': []
        }
        agent = constraint['agent']
        path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                    agent, q['constraints'])
        if path is None:
            return None

        q['paths'][agent] = path

        # handle positive constraints
        if constraint.get('positive', False):
            # every other agent must keep clear of the vertex or edge this agent is forced onto
            for other in range(self.num_of_agents):
                if other != agent:
                    q['constraints'].extend(negative_constraints_for(constraint, other))
            # only agents whose current paths conflict need to be replanned
            violating_agents = paths_violate_constraint(constraint, q['paths'])
            for r_agent in violating_agents:
                r_path = a_star(self.my_map, self.starts[r_agent], self.goals[r_agent],
                                self.heuristics[r_agent], r_agent, q['constraints'])
                if r_path is None:
                    skip_node = True
                    break
                q['paths'][r_agent] = r_path

        if skip_node:
            return None

        q['collisions'] = detect_collisions(q['paths'])
        q['cost'] = get_sum_of_cost(q['paths'])
        return q


    def find_solution(self, disjoint=False, earliest=False):
        """
        Find collision-free paths for all agents using Conflict-Based Search.

        The search begins from a root node containing independently planned
        paths for each agent. Each iteration expands the most promising node,
        resolves one of its collisions by generating constraints, and adds the
        resulting child nodes to the open list. The search terminates when a
        collision-free solution is found, the open list is exhausted, or the
        time limit is reached.

        Args:
            disjoint (bool, optional): If True, use disjoint splitting when
                resolving collisions. Otherwise, use standard splitting.
            earliest (bool, optional): If True, prioritize resolving the
                earliest detected collision.

        Returns:
            list: A path for each agent from its start location to its goal.

        Raises:
            BaseException: If no solution exists.
        """
        self.start_time = timer.time()

        root = {
            'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': []
        }
        for i in range(self.num_of_agents):  # find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        if DEBUG:
            print(root['collisions'])

        if DEBUG:
            for collision in root['collisions']:
                print(standard_splitting(collision))

        while self.open_list and timer.time() - self.start_time < self.max_time:
            p = self.pop_node()
            # a node with no collisions represents a valid solution
            if not p['collisions']:
                return p['paths']
            # select a collision to resolve
            if earliest:
                collision = min(p['collisions'], key=lambda c: c['timestep'])
            else:
                collision = random.choice(p['collisions'])
            # generate constraints using either standard or disjoint splitting
            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)
            # expand one child node per generated constraint
            # child nodes are created in parallel since each re-planning step is independent
            with concurrent.futures.ThreadPoolExecutor() as executor:
                futures = [executor.submit(self.expand_child_node, p, c) for c in constraints]
                for future in concurrent.futures.as_completed(futures):
                    child_node = future.result()
                    if child_node:
                        self.push_node(child_node)
