from re import A
# from this import d
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    for t in range(max(len(path1),len(path2))):
        node1 = get_location(path1,t)
        node2 = get_location(path2,t)
        # finding vertex collisions 
        if(node1 == node2):
            return {'loc':[node1],'timestep':t}
        # finding edge collisions
        if(t > 0):
            if(node1 == node2_prev and node2 == node1_prev):
                return {'loc':[node2,node1],'timestep':t}
        node1_prev = node1
        node2_prev = node2
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    num_of_paths = len(paths)
    for i in range(num_of_paths-1):
        for j in range(i+1,num_of_paths):
            collision = detect_collision(paths[i], paths[j])
            if collision != None:
                collisions.append({'a1':i,'a2':j,'loc':collision['loc'],'timestep':collision['timestep']})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    # vertex collision 
    if(len(collision['loc']) == 1):
        constraints.append({'agent':collision['a1'],'loc':collision['loc'],'timestep':collision['timestep'],'positive':False})
        constraints.append({'agent':collision['a2'],'loc':collision['loc'],'timestep':collision['timestep'],'positive':False})
    # edge collision 
    else:
        loc1 = collision['loc'][0]
        loc2 = collision['loc'][1]
        constraints.append({'agent':collision['a1'],'loc':[loc1,loc2],'timestep':collision['timestep'],'positive':False})
        constraints.append({'agent':collision['a2'],'loc':[loc2,loc1],'timestep':collision['timestep'],'positive':False})
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    constraints = []
    agent = random.randint(0,1)
    agent = list(collision.values())[agent]
    # vertex collision 
    if(len(collision['loc']) == 1):
        constraints.append({'agent':agent,'loc':collision['loc'],'timestep':collision['timestep'],'positive':False})
        constraints.append({'agent':agent,'loc':collision['loc'],'timestep':collision['timestep'],'positive':True})
    # edge collision 
    else:
        loc1 = collision['loc'][0]
        loc2 = collision['loc'][1]
        if(agent == collision['a1']):
            constraints.append({'agent':agent,'loc':[loc1,loc2],'timestep':collision['timestep'],'positive':False})
            constraints.append({'agent':agent,'loc':[loc1,loc2],'timestep':collision['timestep'],'positive':True})
        else:
            constraints.append({'agent':agent,'loc':[loc2,loc1],'timestep':collision['timestep'],'positive':False})
            constraints.append({'agent':agent,'loc':[loc2,loc1],'timestep':collision['timestep'],'positive':True})
    return constraints

# 4.3 computes list of agents that violate a given positive constraint 
def paths_violate_constraint(constraint,paths):
    violating_agents = []
    for agent in range(len(paths)):
        if agent != constraint['agent']:
            curr = get_location(paths[agent],constraint['timestep'])
            prev = get_location(paths[agent],constraint['timestep']-1)
            # checking vertex constraints
            if(len(constraint['loc']) == 1):
                if curr == constraint['loc'][0]:
                    violating_agents.append(agent)
            # checking edge constraints
            else:
                if prev == constraint['loc'][0] and curr == constraint['loc'][1]:
                    violating_agents.append(agent)
                elif prev == constraint['loc'][1] and curr == constraint['loc'][0]:
                    violating_agents.append(agent)
    return violating_agents


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
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        self.start_time = timer.time()
        # Generate the root node
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        # Find initial path for each agent
        for i in range(self.num_of_agents):  
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        ##############################
        # Task 3.3: High-Level Search
        while(len(self.open_list) > 0):
            P = self.pop_node() 
            # found goal node 
            if len(P['collisions']) == 0:
                self.print_results(P)
                return P['paths']
            # getting list of constraints
            collision = P['collisions'][0]
            if(disjoint):
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            # applying constraints child paths
            for constraint in constraints:
                # creating child node 
                if(constraint in P['constraints']):
                    continue
                Q = {'cost': 0,
                    'constraints': P['constraints'].copy() + [constraint],
                    'paths': P['paths'].copy(),
                    'collisions': []}
                
                # find new path for constraint agent
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                        agent, Q['constraints']) 
                Q['paths'][agent] = path 
                
                # find new path for violating agents
                if(constraint['positive'] == True):
                    violating_agents = paths_violate_constraint(constraint,Q['paths'])
                    for violating_agent in violating_agents:
                        # creating new negative constraint for violating agent
                        new_constraint = constraint.copy()
                        new_constraint['agent'] = violating_agent
                        new_constraint['loc'] = constraint['loc'][::-1]
                        new_constraint['positive'] = False
                        Q['constraints'] = Q['constraints'] + [new_constraint]
                        # calculating new path for violating agent
                        new_path = a_star(self.my_map, self.starts[violating_agent], self.goals[violating_agent], self.heuristics[violating_agent],
                                violating_agent, Q['constraints'])    
                        Q['paths'][violating_agent] = new_path   

                # push child node if all paths exist 
                if(not None in Q['paths']):
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    self.push_node(Q)
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
