from re import A
# from this import d
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, get_coords

# detects a single collision between paths 
def detect_collision(path1,path2,sizes):
    for t in range(max(len(path1),len(path2))):
        node1 = get_location(path1,t,sizes[0])
        node2 = get_location(path2,t,sizes[1])
        # finding vertex collisions 
        for loc in node1:
            if(loc in node2):
                # note: [location of collision, reference to agent 1, reference to agent 2]
                ####### 
                return {'loc':[loc,node1[0],node2[0]],'timestep':t}
        # finding edge collisions
        # note: edge collisions only occur only between 1x1 agents 
        if(t > 0):
            if(node1 == node2_prev and node2 == node1_prev):
                # note: need to return node[0] since get_location returns an array of positions
                return {'loc':[node2[0],node1[0]],'timestep':t}
        node1_prev = node1
        node2_prev = node2
    return None

# detects all collisions between agants 
def detect_collisions(paths,sizes):
    collisions = []
    num_of_paths = len(paths)
    for i in range(num_of_paths-1):
        for j in range(i+1,num_of_paths):
            collision = detect_collision(paths[i],paths[j],[sizes[i],sizes[j]])
            if collision != None:
                collisions.append({'a1':i,'a2':j,'loc':collision['loc'],'timestep':collision['timestep']})
    return collisions


def print_constraints(constraints):
    for set in constraints:
        for constraint in set:
            print("a:",constraint['agent'],"loc:",constraint['loc'],"t:",constraint['timestep'])

def print_constraint_set(constraints):
    for constraint in constraints:
        print("a:",constraint['agent'],"loc:",constraint['loc'],"t:",constraint['timestep'])


def asym_splitting(self,collision,old_constraints):
    if(len(collision['loc']) == 3):
        a1 = collision['a1']
        a2 = collision['a2']
        a1_loc = collision['loc'][1]
        a2_loc = collision['loc'][2]
        a1_constraints = []
        a2_constraints = []
        collision_loc = collision['loc'][0]

        x_bound = collision_loc[0] - (self.sizes[a2])
        y_bound = collision_loc[1] - (self.sizes[a2])

        # constraints for a1
        c = {'agent':a1,'loc':[a1_loc],'timestep':collision['timestep'],'positive':False}
        if c not in old_constraints:
            a1_constraints.append(c)
        # constraints for a2
        #a2 is not allowed to be at any vertex where it can collide with a1
        a1coords = get_coords(a1_loc,self.sizes[a1])

        for coord in a1coords:
            for x in range(coord[0],x_bound,-1):
                for y in range(coord[1],y_bound,-1):
                    c = {'agent':a2,'loc':[(x,y)],'timestep':collision['timestep'],'positive':False}
                    if c not in old_constraints:
                        a2_constraints.append(c)
        # print("\n")
    # edge collision (only for 1x1 agents)
    else:
        loc1 = collision['loc'][0]
        loc2 = collision['loc'][1]
        c = {'agent':collision['a1'],'loc':[loc1,loc2],'timestep':collision['timestep'],'positive':False}
        if c not in old_constraints:
            a1_constraints = [c]
        c = {'agent':collision['a2'],'loc':[loc2,loc1],'timestep':collision['timestep'],'positive':False}
        if c not in old_constraints:
            a2_constraints = [c]
    return [a1_constraints,a2_constraints]


def sym_splitting(self, collision, old_constraints):
    if(len(collision['loc']) == 3):
        a1 = collision['a1']
        a2 = collision['a2']
        a1_constraints = []
        a2_constraints = []
        collision_loc = collision['loc'][0]
        x_bound = collision_loc[0] - (self.sizes[a1])
        y_bound = collision_loc[1] - (self.sizes[a1])

        # constraints for a1
        for x in range(collision_loc[0],x_bound,-1):
            for y in range(collision_loc[1],y_bound,-1):
                c = {'agent':a1,'loc':[(x,y)],'timestep':collision['timestep'],'positive':False}
                if c not in old_constraints:
                    a1_constraints.append(c)
        x_bound = collision_loc[0] - (self.sizes[a2])
        y_bound = collision_loc[1] - (self.sizes[a2])
        # constraints for a2
        for x in range(collision_loc[0],x_bound,-1):
            for y in range(collision_loc[1],y_bound,-1):
                c = {'agent':a2,'loc':[(x,y)],'timestep':collision['timestep'],'positive':False}
                if c not in old_constraints:
                    a2_constraints.append(c)
    # edge collision (only for 1x1 agents)
    else:
        loc1 = collision['loc'][0]
        loc2 = collision['loc'][1]
        c = {'agent':collision['a1'],'loc':[loc1,loc2],'timestep':collision['timestep'],'positive':False}
        if c not in old_constraints:
            a1_constraints = [c]
        c = {'agent':collision['a2'],'loc':[loc2,loc1],'timestep':collision['timestep'],'positive':False}
        if c not in old_constraints:
            a2_constraints = [c]
    return [a1_constraints,a2_constraints]


# computes list of agents that violate a given positive constraint 
def paths_violate_constraint(constraint,paths,sizes):
    violating_agents = []
    for agent in range(len(paths)):
        if agent != constraint['agent']:
            curr = get_location(paths[agent],constraint['timestep'],sizes[agent])
            prev = get_location(paths[agent],constraint['timestep']-1,sizes[agent])
            # checking vertex constraints
            if(len(constraint['loc']) == 1):
                for loc in curr:
                    if loc == constraint['loc'][0]:
                        violating_agents.append(agent)
            # checking edge constraints (only for 1x1 agents)
            else:
                curr = curr[0] # <- 1x1, so remove brakcets
                prev = prev[0]
                if prev == constraint['loc'][0] and curr == constraint['loc'][1]:
                    violating_agents.append(agent)
                elif prev == constraint['loc'][1] and curr == constraint['loc'][0]:
                    violating_agents.append(agent)
    return violating_agents

def print_node(node):
    print("cost: ",node['cost'])
    print("constraints: ",node['constraints'])
    print("paths: ",node['paths'])
    print("collisions: ",node['collisions'])
    print("\n")


class MCCBSSolver(object):
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
        if(self.num_of_generated%1000 == 0):
            print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        if(self.num_of_expanded%1000 == 0):
            print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, splitter):
        if splitter == "asymmetrical":
            print("Using asymmetrical splitting.")
        else:
            print("Using symmetrical splitting.")
        self.start_time = timer.time()
        # Generate the root node
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        # Find initial path for each agent
        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, self.sizes[i], root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'],self.sizes)
        self.push_node(root)

        # High-Level Search
        ##############################
        while(len(self.open_list) > 0):
            # print("\n")
            P = self.pop_node()
            # print("P paths:")
            # for i in range(len(P['paths'])):
            #     print("\t",i,"path",P['paths'][i])
            # print("P collision:",P['collisions'])

            # found goal node 
            if len(P['collisions']) == 0:
                self.print_results(P)
                return P['paths']
            # getting list of constraints
            collision = P['collisions'][0]
            if splitter == "asymmetrical":
                constraints = asym_splitting(self,collision, P['constraints'])
            elif splitter == "symmetrical":
                constraints = sym_splitting(self,collision, P['constraints'])
            elif splitter == "disjoint":
                # constraints = disjoint splitting
                constraints = sym_splitting(self, collision, P['constraints'])
            else:
                # constraints = standard splitting
                constraints = sym_splitting(self,collision, P['constraints'])

            for constraint in constraints:
                if(constraint == []):
                    print("constraint skipped")
                    continue

                # creating child node 
                Q = {'cost': 0,
                    'constraints': P['constraints'].copy() + constraint,
                    'paths': P['paths'].copy(),
                    'collisions': []}

                # find new path for constraint agent
                agent = constraint[0]['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                        agent, self.sizes[agent], Q['constraints'])
                Q['paths'][agent] = path

                # push child node if all paths exist 
                if(not None in Q['paths']):
                    Q['collisions'] = detect_collisions(Q['paths'],self.sizes)
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    self.push_node(Q)
                    # print("Q cost:",Q['cost'])
                    # print("Q constraints:")
                    # print_constraint_set(Q['constraints'])
                    # print("Q paths:")
                    # for i in range(len(Q['paths'])):
                    #     print("\t",i,"path",Q['paths'][i])
                    # print("Q collision:",Q['collisions'])
        print("Root solution")
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))