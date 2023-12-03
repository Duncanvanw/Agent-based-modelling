import time as timer
import heapq
import random
import numpy as np
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

np.random.seed(0)

def detect_collision(path1, path2):
    '''Returns the first collision that occurs between two robot paths (or None if there is no collision).'''

    for time in range( min(len(path1), len(path2))):
        loc1 = get_location(path1, time)                                        # Get the location of agent 1 at time t
        loc2 = get_location(path2, time)                                        # Get the location of agent 2 at time t

        """Vertex Collision"""
        if loc1 == loc2: 
            return {'loc': loc1, 'timestep': time}                              # Return the collision location and timestep if there is a vertex collision
        
        """Edge Collision"""
        if time > 0:                                                            # If time is greater than 0, check for edge collision
            prev_loc1 = get_location(path1, time - 1)                               
            prev_loc2 = get_location(path2, time - 1)
            if loc1 == prev_loc2 and loc2 == prev_loc1:                         # If the agents swap locations, there is an edge collision
                return {'loc': [prev_loc1, loc1], 'timestep': time}             # Return the collision location and timestep if there is an edge collision
        
    pass


def detect_collisions(paths):
    '''Returns a list of first collisions between all robot pairs, which are represented in dictionaries.'''
   
    collisions = []
    for i in range(len(paths)):                                                                     
        for j in range(i + 1, len(paths)):                                                     
            """Detect vertex collision between two agents"""
            collision = detect_collision(paths[i], paths[j])                                    # Detect collision between agent i and agent j
            if collision:
                collisions.append({'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep']})       # Add collision to list of collisions

    return collisions      

def standard_splitting(collision):
    '''Returns a list of (two) constraints to resolve the given collision, i.e. a constraint for BOTH agents.'''
    
    constraints = []
    for agent in [collision['a1'], collision['a2']]: 
        if type(collision['loc']) == list:                                  # Edge collision
            if agent == collision['a1']:                                   
                constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'time_step': collision['timestep']})             # Add edge constraint for agent 1
                constraints.append({'agent': collision['a2'], 'loc': collision['loc'][::-1], 'time_step': collision['timestep']})       # Add edge constraint for agent 2
            
        else:       
            constraints.append({'agent': agent, 'loc': collision['loc'], 'time_step': collision['timestep']})                           # Add Vertex collision constraint for both agents
    
    return constraints

def determine_priority_agent(collision, goals, goal_priority = 'lower'):   
    '''Returns the priority agent for the given collision, based on the goal priority setting.'''

    if goal_priority == 'lower':
        priority_agent = collision['a1']                        # Agent with lower index is the priority agent if goal priority is lower
        return priority_agent
    elif goal_priority == 'higher':                             
        priority_agent = collision['a2']                        # Agent with higher index is the priority agent if goal priority is higher
        return priority_agent

    if type(collision['loc']) == list:                                          # Edge collision     
        if collision['loc'][0]  == goals[collision['a1']]:                      # If agent 1 is at its goal location it is the priority agent
            priority_agent = collision['a1']
        if collision['loc'][1]  == goals[collision['a2']]:                      # If agent 2 is at its goal location it is the priority agent
            priority_agent = collision['a2']
        else:                                                                   # If neither agent is at its goal location, choose the lowest index agent as the priority agent
            priority_agent = collision['a1']

    else:                                                                   # Vertex collision
        if collision['loc'] == goals[collision['a1']]:                      
            priority_agent = collision['a1']                                # If agent 1 is at its goal location it is the priority agent
        elif collision['loc'] == goals[collision['a2']]:                    
            priority_agent = collision['a2']                                # If agent 2 is at its goal location it is the priority agent
        else:
            priority_agent = collision['a1']                                # If neither agent is at its goal location, choose the lowest index agent as the priority agent
    
    if not goal_priority:                                                   # If goal priority is False, choose the other agent as the priority agent
        if priority_agent == collision['a1']:
            priority_agent = collision['a2']
        elif priority_agent == collision['a2']:
            priority_agent = collision['a1']
        
    return priority_agent                


def disjoint_splitting(collision, goals, goal_priority = 'lower'):
    '''Returns a list of (two) constraints to resolve the given collision, i.e. a constraint for ONE agent.
    The priority agent is determined using determine_priority_agent(), which is dependent on the goal_priority entry.'''
    
    constraints = []
    priority_agent = determine_priority_agent(collision, goals, goal_priority)         # Determine the priority agent using the determine_priority_agent() function
    

    if type(collision['loc']) == list:              # Edge collision    
        if priority_agent != collision['a1']:
            constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'time_step': collision['timestep']})         # Add edge constraint for agent 1 if it is not the priority agent

        elif priority_agent != collision['a2']:    
            constraints.append({'agent': collision['a2'], 'loc': collision['loc'][::-1], 'time_step': collision['timestep']})   # Add edge constraint for agent 2 if it is not the priority agent
        
    else:                                           # Vertex collision
        for agent in [collision['a1'], collision['a2']]:
            if agent != priority_agent:
                constraints.append({'agent': agent, 'loc': collision['loc'], 'time_step': collision['timestep']})               # Add vertex constraint for agent if it is not the priority agent
     
    return constraints

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
    
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        if node['collisions'] is None:
            heapq.heappush(self.open_list, (node['cost'], 0, self.num_of_generated, node))
        else:
            heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, goal_priority = 'lower'):
        """ Finds paths for all agents from their start locations to their goal locations.
        disjoint - use disjoint splitting or not.
        goal_priority: way of determining priority agent."""

        self.start_time = timer.time()

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):                    
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])                                               # Find initial path for each agent
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)                                                          # Add initial path to root node    

        root['cost'] = get_sum_of_cost(root['paths'])                                           # Calculate cost of root node
        root['collisions'] = detect_collisions(root['paths'])                                   # Detect collisions in root node
        self.push_node(root)                                                                    # Add root node to open list

        # Task 3.2: Testing
        if root['collisions'] is None:
            print('No collision')                                                               # If there are no collisions, return no collision
        else:
            for collision in root['collisions']:
               root['constraints'].append(disjoint_splitting(collision, self.goals, goal_priority))     # Add constraints to root node (of the initial paths)
            print(f'Done with initial search')

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node tFo your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            node = self.pop_node()                                                      # Get the next best node from the open list
            
            if not node['collisions']:
                self.print_results(node, goal_priority)                                 # If there are no collisions, return the solution
                return node['paths']
            
            else:       
                collision = node['collisions'][0]  
                constraints = disjoint_splitting(collision, self.goals, goal_priority)      # Get constraints for the first collision in the node

                for constraint in constraints:              
                    child = {'cost': 0,
                            'constraints': node['constraints'] + [constraint],          
                            'paths': [], 
                            'collisions': []}                                   # Create a child node for each constraint
                    
                    for i in range(self.num_of_agents):
                        path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                    i, child['constraints'])                    # Find a path for each agent in the child node
                        if path is None:
                            raise BaseException('No solutions')     
                        child['paths'].append(path)                                 # Add the path to the child node
                    child['cost'] = get_sum_of_cost(child['paths'])                 # Calculate the cost of the child node
                    child['collisions'] = detect_collisions(child['paths'])	        # Detect collisions in the child node
                    self.push_node(child)                                           # Add the child node to the open list
            
        self.print_results(root, goal_priority)
        return root['paths']                                                        # Return the paths of the root node if there are no solutions

    def print_results(self, node, goal_priority):
        max_makespan = 0                                                            # Initialize makespan as empty variable
        max_makespan_agent = None                                                   # Initialize agent with maximum makespan as empty variable

        # Calculate makespan and find the agent with the maximum makespan
        for agent_number, path in enumerate(node['paths']):                         
            steps = 0
            for i in range(len(path)-1):
                if path[i] != path[i+1]:
                    steps += 1
            if steps > max_makespan:
                max_makespan = steps                                                # Update makespan if the current agent has a higher makespan
                max_makespan_agent = agent_number                                   # Update agent with maximum makespan if the current agent has a higher makespan

        # Calculate CPU time
        CPU_time = timer.time() - self.start_time                                   

        # Print the results
        print(f"\n Found a solution for goal priority: {goal_priority} \n")
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Makespan:        {} (agent {})".format(max_makespan, max_makespan_agent))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))