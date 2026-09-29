#!/usr/bin/python
import argparse
import glob
import os
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from random_instance import random_map, save_map, OUTPUT_MAP
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVERS = ["CBS", "Independent", "Prioritized"]
SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += 'X '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # read map dimensions (rows, columns)
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # read the map grid and convert obstacles/free cells to booleans
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == 'X':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # read the number of agents
    line = f.readline()
    num_agents = int(line)
    # read each agent's start and goal locations
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    validate_mapf_instance(filename, my_map, starts, goals)
    return my_map, starts, goals


def validate_mapf_instance(filename, my_map, starts, goals):
    # reject instances that no solver could handle, with a message pointing at the problem
    rows, columns = len(my_map), len(my_map[0]) if my_map else 0
    for r, row in enumerate(my_map):
        if len(row) != columns:
            raise ValueError("{}: map row {} has {} cells, expected {}".format(filename, r, len(row), columns))
    for kind, locations in (("start", starts), ("goal", goals)):
        for agent, (x, y) in enumerate(locations):
            if not (0 <= x < rows and 0 <= y < columns):
                raise ValueError("{}: agent {} {} {} is outside the {}x{} map".format(
                    filename, agent, kind, (x, y), rows, columns))
            if my_map[x][y]:
                raise ValueError("{}: agent {} {} {} is on an obstacle".format(filename, agent, kind, (x, y)))
        if len(set(locations)) != len(locations):
            raise ValueError("{}: two agents share the same {} location".format(filename, kind))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run multi-agent path finding (MAPF) algorithms.")

    parser.add_argument("--random", action="store_true",
                        help="Generate a random map with agents instead of loading an instance file.")
    
    parser.add_argument("--instance", type=str, default=None,
                        help="Path for MAPF instance file(s).")
    
    parser.add_argument("--batch", action="store_true",
                        help="Run in batch mode (no animation).")
    
    parser.add_argument("--solver", type=str, choices=SOLVERS, default=SOLVER,
                        help=f"Solver to use. Options: {', '.join(SOLVERS)}. Default: {SOLVER}.")

    parser.add_argument("--disjoint", action="store_true",
                        help="Enable disjoint splitting for CBS.")

    args = parser.parse_args()

    if args.random:
        files = ["random.generated"]
    else:
        files = glob.glob(args.instance)
        print(files)

    for file in files:
        my_map, starts, goals = random_map(8,8,5,0.3) if args.random else import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)
        if args.random:
            # a generated map is not stored anywhere else, so keep a copy
            save_map(my_map, starts, goals, OUTPUT_MAP)
        if args.solver == "CBS":
            print("***Run CBS***")
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        cost = get_sum_of_cost(paths)
        print("Cost:", cost)

        if not args.batch:
            animation = Animation(my_map, starts, goals, paths)
            os.makedirs("output", exist_ok=True)
            animation.save("output/output.gif", 1.0)
            animation.show()
    print("***Done***")
