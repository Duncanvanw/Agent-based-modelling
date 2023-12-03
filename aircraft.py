"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from single_agent_planner import a_star

class AircraftDistributed(object):
    """An aircraft agent for distributed planning"""

    def __init__(self, my_map, start, goal, heuristics, id):
        """my_map      - list of lists specifying obstacle positions
        start       - (x, y) tuple specifying start location
        goal        - (x, y) tuple specifying goal location
        heuristics  - list of heuristic values for each location in the map
        id          - unique identifier for the agent
        """
        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.heuristics = heuristics
        self.id = id
        self.path = []
        self.constraints = []


    def add_constraint(self, constraint):
        """Adds a constraint to the agent's list of constraints"""
        if constraint not in self.constraints:
            self.constraints.append(constraint)

    def replan(self):
        """Replans the agent's path with the current list of constraints"""
        self.path = a_star(self.my_map, self.start, self.goal, self.heuristics, self.id, self.constraints)

    def replan_from(self, timestep):
        """Replans the agent's path with the current list of constraints from a given timestep"""
        path_until_timestep = self.path[:timestep + 1]                                                                  # Slice the path until the given timestep
        self.start = path_until_timestep[-1]                                                                            # Set the start location to the last location of the path until the given timestep
        path_from_timestep = a_star(self.my_map, self.start, self.goal, self.heuristics, self.id, self.constraints)     # Replan the path from the given timestep
        self.path = path_until_timestep + path_from_timestep                                                            # Concatenate the two paths                                    

    def __len__(self):
        """Returns the length of the agent's path"""
        return len(self.path)

    def __getitem__(self, index):
        """Allows the agent's path to be accessed using the subscript operator []"""
        return self.path[index]
    
    def __repr__(self):
        """Returns a string representation of the agent"""
        return "AircraftDistributed({}, {}, {}, {})".format(self.my_map, self.start, self.goal, self.id)
