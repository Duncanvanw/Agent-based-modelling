import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # Compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        start_time = timer.time()
        result = []
        constraints = []   


        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)                                      # Plan paths for each agent
            if path is None:
                raise BaseException('No solutions')
            result.append(path)                                                 # Append path to result list
            path_dict = {}
            path_curr_agent = []
            edge_path_curr_agent = []

            for time_step in range(len(path)):
                path_dict = {'agent': i, 'time_step': time_step, 'loc': path[time_step]}
                path_curr_agent.append(path_dict)
                if time_step > 0:
                    edge_path_dict = {'agent': i, 'time_step': time_step, 'loc': [path[time_step], path[time_step -1]]}     # Make dictionary for edge paths (from current location to previous location)        
                    edge_path_curr_agent.append(edge_path_dict)
    
            for agent in range(i + 1, self.num_of_agents):
                for path in path_curr_agent:    
                    constraint = {'agent': agent, 'time_step': path['time_step'], 'loc': path['loc']}                       # Make dictionary for constraints with location of agent where the vertex collision will be
                    constraints.append(constraint)

                for edge_path in edge_path_curr_agent:
                    if time_step > 0:
                        constraint = {'agent': agent, 'time_step': edge_path['time_step'], 'loc': edge_path['loc']}         # Make dictionary for constraints (edge path of agent)
                        constraints.append(constraint)         
      
        max_makespan = 0
        for agent_number, path in enumerate(result):                                                                # Find the makespan of the solution
            steps = 0
            for i in range(len(path)-1):
                if path[i] != path[i+1]:    
                    steps += 1                                                                                       # If the location of the agent is not the same as the next location, add 1 to the steps   
                    if steps > max_makespan:
                        max_makespan_agent = agent_number
                    max_makespan = max(max_makespan, steps)                                                          # Find max makespan

        
        self.CPU_time = timer.time() - start_time
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print("Makespan:        {} (agent {})".format(max_makespan, max_makespan_agent))
        #print(f"This is the result variable:{result}")
        
        return result
    
