import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals, sizes):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.sizes = sizes
        self.num_of_agents = len(goals)
        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = [
                        # {'agent':1,'loc':[(1,4)],'timestep': 3},
                        # {'agent':1,'loc':[(1,3),(1,4)],'timestep': 3},
                        # {'agent':1,'loc':[(1,3)],'timestep': 2}
                    ]

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, self.sizes[i], constraints)
            if path is None:
                raise BaseException('No solutions')
            # result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            path_size = self.sizes[i]
            for agent in range(self.num_of_agents):
                # print("Adding constraints for agent " + str(agent) + " based on the path of agent " + str(i))
                agent_size = self.sizes[agent]
                for t in range(len(path)):
                    locations = []
                    top_left = list(path[t])
                    if agent != i:
                        for x in range(0, path_size):
                            for y in range(0, path_size):
                                locations.append(tuple([top_left[0] + x, top_left[1] + y]))
                        # print("Locations to be constrained based on this path, at time " + str(t) + ": " + str(locations))
                        if agent_size == 1:
                            # print("Forming constraints for a 1x1 agent.")
                            # print("At time " + str(t) + " this agent cannot occupy:")
                            for location in locations:
                                # print(location)
                                constraints.append({'agent': agent, 'loc': [location], 'timestep': t, 'positive': False})
                            if t > 0:
                                edge_constraint = {'agent': agent, 'loc': [path[t], path[t - 1]], 'timestep': t, 'positive': False}
                                constraints.append(edge_constraint)
                            if t == len(path) - 1:
                                vertex_constraint = {'agent': agent, 'loc': [path[t]], 'timestep': -1, 'positive': False}
                                constraints.append(vertex_constraint)
                        elif agent_size > 1:
                            # print("Forming constraints for a 2x2 or larger agent.")
                            for location in locations:
                                # print("In order to avoid " + str(location) + ", this agent must not be in")
                                for x in range(0, agent_size):
                                    for y in range(0, agent_size):
                                        coordinates = tuple([max(0, location[0] - x), max(0, location[1] - y)])
                                        # print(coordinates)
                                        constraint = {'agent': agent, 'loc': [coordinates], 'timestep': t, 'positive': False}
                                        constraints.append(constraint)
                                        if t == len(path) - 1:
                                            constraint = {'agent': agent, 'loc': [coordinates], 'timestep': -1,
                                                                 'positive': False}
                                        #print("Adding the following constraint: " + str(constraint))
                                        constraints.append(constraint)



            # recalculating paths based on generated constraints
            # print("Invoking a_star with the following constraints:" + str(constraints))
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, self.sizes[i], constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
