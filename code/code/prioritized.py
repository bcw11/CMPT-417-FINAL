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
            for agent in range(self.num_of_agents):
                print("Adding constraints for agent " + str(agent) + " based on the path of agent " + str(i))
                for t in range(len(path)):
                    if agent != i:
                        if self.sizes[i] > 1:
                            print("Forming constraints based on the path of a large agent.")
                            # for the path of a 2x2 agent, the following must be constrained:
                            # [loc][   ]
                            # [   ][   ]
                            top_left = list(path[t])
                            top_right = [top_left[0], top_left[1] + 1]
                            bot_left = [top_left[0] + 1, top_left[1]]
                            bot_right = [top_left[0] + 1, top_left[1] + 1]
                            cells_of_2x2_agent = [top_left, top_right, bot_left, bot_right]
                            print("Agent " + str(i) + " is big and touches "
                                  + str(top_left) + ", "
                                  + str(top_right) + ", "
                                  + str(bot_left) + ", "
                                  + str(bot_right) + " at time " + str(t) + ".")
                            print("Adding the following constraints for agent " + str(agent) + ":")
                            for coordinates in cells_of_2x2_agent:
                                constraint = {'agent': agent, 'loc': [tuple(coordinates)], 'timestep': t, 'positive': False}
                                print(constraint)
                                constraints.append(constraint)
                                # If we are forming constraints for a 2x2 agent, then it must not be in such a position
                                # that it would touch any constrained cells, i.e.
                                # [loc][   ]            [] = agent 'agent'
                                # [   ]{loc}{   }       {} = agent 'i'
                                #      {   }{   }
                                # or
                                # [loc]{loc}{   }
                                # [   ]{   }{   }
                                if self.sizes[agent] > 1:
                                    print("Forming constraints between two 2x2 agents.")
                                    above = [coordinates[0], max(0, coordinates[1] - 1)]
                                    left = [max(0, coordinates[0] - 1), coordinates[1]]
                                    above_and_left = [max(0, coordinates[0] - 1), max(0, coordinates[1] - 1)]
                                    cells_of_other_2x2_agent = [above, left, above_and_left]
                                    for cell in cells_of_other_2x2_agent:
                                        constraint = {'agent': agent, 'loc': [tuple(cell)], 'timestep': t, 'positive': False}
                                        print(constraint)
                                        constraints.append(constraint)
                            # #print("Constraints after finding a big agent:", constraints)
                        elif self.sizes[i] > 2:
                            # This is where the absolute travesty that handling 3x3 agents would go
                            continue
                        elif self.sizes[i] == 1:
                            vertex_constraint = {'agent': agent, 'loc': [path[t]], 'timestep': t, 'positive': False}
                            if self.sizes[agent] > 1:
                                # If we are forming constraints for a 2x2 agent, then it must not be in such a position
                                # that it would touch any constrained cells, i.e.
                                # [loc][   ]            [] = agent 'agent'
                                # [   ]{loc}            {} = agent 'i'
                                # or
                                # [loc]{loc}
                                # [   ][   ]
                                location = list(path[t])
                                above = [location[0], max(0, location[1] - 1)]
                                left = [max(0, location[0] - 1), location[1]]
                                above_and_left = [max(0, location[0] - 1), max(0, location[1] - 1)]
                                cells_of_other_2x2_agent = [above, left, above_and_left]
                                for cell in cells_of_other_2x2_agent:
                                    constraint = {'agent': agent, 'loc': [tuple(cell)], 'timestep': t,
                                                  'positive': False}
                                    print(constraint)
                                    constraints.append(constraint)
                            constraints.append(vertex_constraint)
                            if t == len(path) - 1:
                                vertex_constraint = {'agent': agent, 'loc': [path[t]], 'timestep': -1, 'positive': False}
                                constraints.append(vertex_constraint)
                            if t > 0:
                                edge_constraint = {'agent': agent, 'loc': [path[t], path[t - 1]], 'timestep': t, 'positive': False}
                                constraints.append(edge_constraint)

            # recalculating paths based on generated constraints
            print("Invoking a_star with the following constraints:" + str(constraints))
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
