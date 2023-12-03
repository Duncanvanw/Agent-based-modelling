"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""

#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for a open cell, and 
    a number for the start location of each agent.
    
    Example:
        @ @ @ @ @ @ @ 
        @ 0 1 . . . @ 
        @ @ @ . @ @ @ 
        @ @ @ @ @ @ @ 
    """
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    """
    Imports mapf instance from instances folder. Expects input as a .txt file in the following format:
        Line1: #rows #columns (number of rows and columns)
        Line2-X: Grid of @ and . symbols with format #rows * #columns. The @ indicates an obstacle, whereas . indicates free cell.
        Line X: #agents (number of agents)
        Line X+1: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 1)
        Line X+2: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 2)
        Line X+n: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent n)
        
    Example:
        4 7             # grid with 4 rows and 7 columns
        @ @ @ @ @ @ @   # example row with obstacle in every column
        @ . . . . . @   # example row with 5 free cells in the middle
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
        2               # 2 agents in this experiment
        1 1 1 5         # agent 1 starts at (1,1) and has (1,5) as goal
        1 2 1 4         # agent 2 starts at (1,2) and has (1,4) as goal
    """
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--optimal', action='store_true', default=False,
                        help='See if goal priority, lower or higher index agents is optimal')

    args = parser.parse_args()
    # Hint: Command line options can be added in Spyder by pressing CTRL + F6 > Command line options. 
    # In PyCharm, they can be added as parameters in the configuration.

    result_file = open("results.csv", "a", buffering=1)
    still_print = True

    for file in sorted(glob.glob(args.instance)):
        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        if not args.optimal:  
            if args.solver in ["CBS", "Distributed"]:
                	
                if args.solver == "CBS":
                    print("***Run Conflict Based Search***")
                    solver = CBSSolver(my_map, starts, goals)                                       # Solver is CBS

                else:
                    print("***Run Distributed Planning***")
                    solver = DistributedPlanningSolver(my_map, starts, goals)                       # Solver is Distributed Planning

                try:
                    print('-----Running with priority for agents on their goal-----')
                    goal_priority = True                    
                    paths = solver.find_solution(args.disjoint, goal_priority)                      # Try to find a solution with priority for agents on their goal
                except BaseException:
                    try:
                        print('-----Running with priority for lower index agents-----')
                        goal_priority = 'lower'
                        paths = solver.find_solution(args.disjoint, goal_priority)                  # Try to find a solution with priority for lower index agents
                    except BaseException:
                        try:
                            print('-----Running with priority for higher index agents-----')
                            goal_priority = 'higher'
                            paths = solver.find_solution(args.disjoint, goal_priority)              # Try to find a solution with priority for higher index agents
                        except BaseException:
                            print('-----Running with priority for agents not on their goal-----')
                            goal_priority = False
                            paths = solver.find_solution(args.disjoint, goal_priority)              # Try to find a solution with priority for agents not on their goal

                finally:
                    if args.batch:    
                        cost = get_sum_of_cost(paths)
                        print(f'Cost: {cost}')
                        result_file.write("{},{},{}\n".format(file, cost, goal_priority))           # Write the result to the result file
                        still_print = False  

            elif args.solver in ['CBStrue', 'CBSfalse', 'CBSlower', 'CBShigher', 'Distributedtrue', 'Distributedfalse', 'Distributedlower', 'Distributedhigher']:       # if the solver is CBS or sistributed with a priority
                solvers = {'CBS': CBSSolver, 'Distributed': DistributedPlanningSolver}
                priority_values = {'true': True, 'false': False, 'lower': 'lower', "higher": 'higher'}

                for solver_name, solverclass in solvers.items():
                    for priority_name, priority_value in priority_values.items():
                        if args.solver == solver_name + priority_name:
                            print(f"***Run {solver_name} with priority {priority_name}***")
                            solver = solverclass(my_map, starts, goals)                                                         # Solver is CBS or Distributed Planning
                            paths = solver.find_solution(args.disjoint, priority_value)                                         # Find a solution with inputted priority

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

        elif args.optimal:
            print(f"***Finding optimal version of {args.solver} solver***")


            if args.solver == 'Distributed':
                solver = DistributedPlanningSolver(my_map, starts, goals)
            elif args.solver == 'CBS':
                solver = CBSSolver(my_map, starts, goals)

            results = []
            for priority in [True, 'lower', 'higher', False]:
                try:
                    path = {'priority' : priority, 'path' : solver.find_solution(args.disjoint, priority)}          # Try to find a solution with inputted priority
                except BaseException:
                    path = {'priority' : priority, 'path' : None}                                                   # If no solution is found, set path to None
                results.append(path)

        
            best_cost = 10000               # Initialize best cost to a high number
            best_result = None

            for result in results:
                if result['path'] is not None:
                    cost = get_sum_of_cost(result['path'])                              # Get the cost of the path
                    result['cost'] = cost
                    if result['cost'] < best_cost:                                      
                        best_cost = result['cost']                                      # If the cost is lower than the best cost, set the best cost to the cost of the path
                        best_result = result
                else:
                    result['cost'] = None
                   
            if best_result is None:
                raise BaseException('No solutions found for all four priority options')
            else:    
                paths = best_result['path']
                cost = best_result['cost']
                priority = best_result['priority']
                print(f'\nBest option was priority {priority} with cost {cost}')                    # Print the best option and its cost

                if args.batch:
                    result_file.write("{},{},{}\n".format(file, best_result['priority'] , best_result['cost']))          # Write the result to the result file if --batch
              
        if args.batch and not args.optimal and still_print:    
            cost = get_sum_of_cost(paths)
            print(f'Cost: {cost}')
            result_file.write("{},{}\n".format(file, cost))                                                              # Write the result to the result file if --batch

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
            animation.show()
    result_file.close()
