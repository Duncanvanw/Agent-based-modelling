import heapq
import numpy as np

np.random.seed(0)

def move(loc, dir):
    '''Return the location after applying one of five possible moves.'''
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    """Return the sum of path lengths."""
    rst = 0
    for path in paths:
        for i in range(len(path)-1):
            if path[i] != path[i+1]:        
                rst += 1                    # Add 1 to the sum of cost for each move

    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    """Build a table of constraints for the given agent, from given constraints."""

    constraint_table = dict()
    for constraint in constraints:
        if type(constraint) == list:                            # If multiple constraints are given

            for sub_constraint in constraint:
                if sub_constraint['agent'] == agent:
                    if sub_constraint['time_step'] not in constraint_table:                         
                        constraint_table[sub_constraint['time_step']] = list()                              # If the timestep is not in the constraint table, add it
                    constraint_table[sub_constraint['time_step']].append(sub_constraint['loc'])             # Add the location of the constraint to the constraint table

        elif type(constraint) == dict:                                                                      # If one constraint is given
            if constraint['agent'] == agent:
                if constraint['time_step'] not in constraint_table:
                    constraint_table[constraint['time_step']] = list()                                      # If the timestep is not in the constraint table, add it
                constraint_table[constraint['time_step']].append(constraint['loc'])                         # Add the location of the constraint to the constraint table

    return constraint_table

def get_location(path, time):
    """Return the location of the agent at the given time step."""

    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    """Return the path by tracing back from the goal node."""

    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    '''Return true if the next location is constrained at the next time step, False otherwise.'''

    for time, constraints in constraint_table.items():              # Loop through the constraint table
        vertex_constraint = next_loc                                # The vertex constraint is the next location
        if next_time == time:                                       # If the next time step is the same as the time step in the constraint table
            if vertex_constraint in constraints:                    
                return True                                         # If the vertex constraint is in the constraints
            
            edge_constraint = [curr_loc, next_loc]                  # The edge constraint is the current location and the next location
            if edge_constraint in constraints:                      
                return True                                         # If the edge constraint is in the constraints

    return False                                                    # If the next location is not constrained at the next time step, return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['time_step'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_max_manhattan_distance(map):
    """Return the maximum Manhattan distance between any two cells in the map."""
    nr_rows = len(map)
    nr_cols = len(map[0])

    max_distance = 0
    for row1 in range(nr_rows):                                 # Loop through rows
        for col1 in range(nr_cols):                             # Loop through columns
            if not map[row1][col1]:                             # If cell 1 is not an wall
                for row2 in range(nr_rows):         
                    for col2 in range(nr_cols):
                        if not map[row2][col2]:                 # If cell 2 is also not a wall
                            distance = abs(row1 - row2) + abs(col1 - col2)      # Calculate Manhattan distance
                            if distance > max_distance:
                                max_distance = distance

    return max_distance

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    constraint_table = build_constraint_table(constraints, agent)               # Build constraint table for the given agent
    open_list = []
    closed_list = dict()

    earliest_goal_timestep = get_max_manhattan_distance(my_map) * 3             # Set the earliest goal timestep to the maximum Manhattan distance between any two cells in the map times 3, After earliest goal timestep, there are no more constraints generated for that agent
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['loc'] == goal_loc and curr['time_step'] == earliest_goal_timestep:
            return get_path(curr)                                                   # Return the path if the current location is the goal location and the current time step is the earliest goal timestep
        elif curr['time_step'] > earliest_goal_timestep:
            return None                                                             # Return None if the current time step is greater than the earliest goal timestep
        
        for dir in range(5):
            child_loc = move(curr['loc'], dir)                                      # Get the location of the child node

            if child_loc[0] >= 0 and child_loc[1] >= 0 and child_loc[0] < len(my_map) and child_loc[1] < len(my_map[0]):    # If the child location is within the map
                if my_map[child_loc[0]][child_loc[1]]:                                                                          
                    continue                                                          # If the child location is a wall, continue to next move
                
                if my_map[child_loc[0]][child_loc[1]] is False:
                    
                    # Penalty for waiting actions Doesnt work
                    # if dir == 4:            # Wait action
                    #     g_val = curr['g_val'] + 0.5
                    # else:                   # Move action
                    #     g_val = curr['g_val'] + 1
                    
                    # child = {'loc': child_loc,
                    #         'g_val': g_val,
                    #         'h_val': h_values[child_loc],
                    #         'parent': curr,
                    #         'time_step': curr['time_step'] + 1}
                    
                    child = {'loc': child_loc,
                            'g_val': curr['g_val'] + 1,
                            'h_val': h_values[child_loc],
                            'parent': curr,
                            'time_step': curr['time_step'] + 1}            # Create child node with the location of the child, the g value of the child, the h value of the child, the parent node, and the time step of the child
                    
                    if is_constrained(curr['loc'], child_loc, child['time_step'], constraint_table):
                        continue                                           # If the child location is constrained at the child time step, continue to next move

                    if (child['loc'], child['time_step']) in closed_list:
                        existing_node = closed_list[(child['loc'], child['time_step'])]     # If the child node is in the closed list, set the existing node to the child node
                        if compare_nodes(child, existing_node):
                            closed_list[(child['loc'], child['time_step'])] = child         
                            push_node(open_list, child)                                     # If the child node is better than the existing node, set the child node to the existing node
                    else:
                        closed_list[(child['loc'], child['time_step'])] = child
                        if child not in open_list:                                          
                            push_node(open_list, child)                                     # If the child node is not in the open list, push the child node to the open list

    return None  # Failed to find solutions
