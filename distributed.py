from cbs import detect_collisions, detect_collision, get_location, disjoint_splitting
from single_agent_planner import a_star, compute_heuristics, get_sum_of_cost
import time as timer
from aircraft import AircraftDistributed

def detect_collision_radar(path1, path2, timestep, radar = False, future_timesteps = 1):
    '''Returns the first collision that occurs between two robot paths Using a radar if desired with an inputted amount of future timesteps.
    Radar does not work!!'''

    if not radar:
        for time in range( min(len(path1), len(path2))):     # timesteps are from timestep to timestep + 2, until the end of the shortest pathlength
            loc1 = get_location(path1, time)
            loc2 = get_location(path2, time)

            """Vertex Collision"""
            if loc1 == loc2: 
                return {'loc': loc1, 'timestep': time}      # return the location of the vertex collision and the timestep it occurs at
            
            """Edge Collision"""
            if time > 0:
                prev_loc1 = get_location(path1, time - 1)
                prev_loc2 = get_location(path2, time - 1)
                if loc1 == prev_loc2 and loc2 == prev_loc1:
                    return {'loc': [prev_loc1, loc1], 'timestep': time}   # return the location of the edge collision and the timestep it occurs at
    
    elif radar:

        for time in range(timestep, min( min(len(path1), len(path2)), timestep + future_timesteps + 1)):     # Timesteps are from timestep to timestep + 2, until the end of the shortest pathlength
            loc1 = get_location(path1, time)                                                                 # Get the location of agent 1 at the current timestep
            loc2 = get_location(path2, time)                                                                 # Get the location of agent 2 at the current timestep

            """Vertex Collision"""
            if loc1 == loc2: 
                return {'loc': loc1, 'timestep': time}                                                       # Return the location of the vertex collision and the timestep it occurs at
            
            """Edge Collision"""
            if time > 0:
                prev_loc1 = get_location(path1, time - 1)
                prev_loc2 = get_location(path2, time - 1)
                if loc1 == prev_loc2 and loc2 == prev_loc1:
                    return {'loc': [prev_loc1, loc1], 'timestep': time}                                      # Return the location of the edge collision and the timestep it occurs at
            
    pass

def detect_collisions_radar(paths, timestep = 0, radar = False, future_timesteps = 0):
    '''Returns a list of collisions that occur between all robot paths Using a radar if desired with an inputted amount of future timesteps.'''

    collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            """Detect vertex collision between two agents"""
            collision = detect_collision_radar(paths[i], paths[j], timestep, radar, future_timesteps)

            if collision:
                collisions.append({'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep']})


    return collisions      

class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        for goal in goals:
            self.heuristics.append(compute_heuristics(my_map, goal))


    def find_solution(self, disjoint=True, goal_priority = 'lower', radar = False, future_timesteps = 1):
        """
        Finds paths for all agents from start to goal locations. 
        - goal_priority: True, False, 'lower', or 'higher' (default = 'lower')
        - use radar: True or False (default = False) (doesnt work)
        - future_timesteps: look ahead time of radar (default = 1 timestep)
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """

        # Initialize constants       
        start_time = timer.time()
        result = []
        self.CPU_time = timer.time() - start_time
        
        # Create agent objects with AircraftDistributed class
        agents = []                                                                         
        for i in range(len(self.starts)):
            agent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)          # Create an agent object for each agent        
            agents.append(agent)

        for agent in agents:
            path = a_star(self.my_map, agent.start, agent.goal, agent.heuristics, agent.id, [])                     # Plan paths for each agent
            if path is None:
                print("Failed to find a path for agent {}".format(agent.id))                                        
                return None                                                                                         # If no path is found, return none
            agent.path = path   
        
        if not radar:       
            max_timestep = 1                                # If not radar, then max_timestep is 1 (iterate only once, not per timestep)
        elif radar:
            max_timestep = max([len(agent.path) for agent in agents])

        for timestep in range(0, max_timestep):
            solve = True
            while solve:
                total_paths = []
                for agent in agents:
                    total_paths.append(agent.path)                                                  # Create a list of all paths
                    if agent.path is None:
                        raise BaseException('No solutions')
 
                collisions = detect_collisions_radar(total_paths, timestep, radar, future_timesteps = 1)        # Detect collisions between all agents
                
                if not collisions:
                    print(f'No collisions at timestep {timestep}')
                    break

                conflicts = {agent.id: [] for agent in agents}                              # Create a dictionary of conflicts consisting of collisions for each agent

                for collision in collisions:
                    agent1 = agents[collision['a1']]
                    agent2 = agents[collision['a2']]
                    goals = [goal for goal in self.goals]                                       # Create a list of goals
                    constraints = disjoint_splitting(collision, goals, goal_priority)           # Split the constraints into disjoint sets

                    for constraint in constraints:
                        if constraint['agent'] == agent1.id:
                            conflicts[agent1.id].append(constraint)                             # Add the constraints to the conflicts dictionary for the corresponding agent
                            agent1.add_constraint(constraints)
                        elif constraint['agent'] == agent2.id:
                            agent2.add_constraint(constraints)   
                            conflicts[agent2.id].append(constraint)                             # Add the constraints to the conflicts dictionary for the corresponding agent

                for agent in agents:
                    if not radar:
                        agent.replan()                                                          # Replan the path for each agent

                    else:                                                                       # If radar is true
                        if conflicts[agent.id]: 
                            agent.add_constraint(conflicts[agent.id])                           # Add the constraints to the agent from conflixts dictionary
                        agent.replan_from(timestep)                                             # Replan the path for each agent from the current timestep
                            
        # Build result list
        for agent in agents:
            path = []
            for i in range(len(agent.path)):
                path.append(agent.path[i])                                                      # Append the path for each agent to the result list
            result.append(path)
        
        max_makespan = 0
        for agent_number, path in enumerate(result):                                            # Find the makespan for each agent
            steps = 0
            for i in range(len(path)-1):
                if path[i] != path[i+1]:
                    steps += 1
                    if steps > max_makespan:
                        max_makespan_agent = agent_number
                    max_makespan = max(max_makespan, steps)                                     # Find the max makespan

        self.CPU_time = timer.time() - start_time
        
        # Print final output
        print(f"\n Found a solution for goal priority: {goal_priority} \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print("Makespan:        {} (agent {})".format(max_makespan, max_makespan_agent))

        return result