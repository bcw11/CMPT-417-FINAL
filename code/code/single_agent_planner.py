import heapq

# -- PRE DEFINED FUNCTIONS --
def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

# Use Dijkstra to build a shortest-path tree rooted at the goal location
def compute_heuristics(my_map, goal):
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
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

def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


# -- FINAL PROJECT START --

# Return a table that contains the list of constraints of the given agent for each time step.
def build_constraint_table(constraints, agent):
    constraint_table = {}
    for constraint in constraints:
        if(constraint['agent'] == agent):
            constraint_table.setdefault(constraint['timestep'],[]).append({'loc':constraint['loc'],'positive':constraint['positive']})
    return constraint_table

# Check if a move from curr_loc to next_loc at time step next_time violates any given constraint.
def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    if(next_time in constraint_table):
        for constraint in constraint_table[next_time]:
            # vertex constraints
            if(len(constraint['loc']) == 1):
                if(constraint['loc'][0] == next_loc and constraint['positive'] == False):
                    return True
                elif(constraint['loc'][0] != next_loc and constraint['positive'] == True):
                    return True
            # edge constraints
            else:
                if(constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc and constraint['positive'] == False):
                    return True
                elif((constraint['loc'][0] != curr_loc or constraint['loc'][1] != next_loc) and constraint['positive'] == True):
                    return True
                
    # checking for agents that have reached goal node
    if(-1 in constraint_table):
        for constraint in constraint_table[-1]:
            if next_loc == constraint['loc'][0]:
                return True
    return False        

# finds the earlist goal time step
def find_earlist_goal_timestep(goal_loc,constraint_table):
    earliest_goal_timestep = 0
    for timestep, constraints in constraint_table.items():
        for constraint in constraints:
            if (constraint['positive'] == False and len(constraint['loc']) == 1  \
                    and constraint['loc'][0] == goal_loc and timestep > earliest_goal_timestep):
                earliest_goal_timestep = timestep
    return earliest_goal_timestep

# checking bounds of board 
def out_of_bounds(child_loc,my_map):
    return child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0])

def get_location(path, t, size):
    t = max(0,t)
    if(t >= len(path)):
        t = -1
    return get_coords(path[t],size)

def get_coords(loc, size):
    # referencing agent from top left
    coords = []
    for i in range (size):
        for j in range (size):
            coords.append((loc[0]+i,loc[1]+j))
    return coords

# checking bounds of board for 2x2 agent
def sized_out_of_bounds(loc, my_map, size):
    coords = get_coords(loc,size)

    for coord in coords:
        if my_map[coord[0]][coord[1]] or out_of_bounds(coord,my_map):
            return True
    return False

def a_star(my_map, start_loc, goal_loc, h_values, agent, size, constraints):

    open_list = []
    closed_list = dict()
    
    # building a constraint table
    constraint_table = build_constraint_table(constraints,agent)

    # finding earlist goal time step for goal constraints
    earliest_goal_timestep = find_earlist_goal_timestep(goal_loc,constraint_table)

    # finding upper bound on path length/timestep
    size_of_map =  sum([i.count(False) for i in my_map])
    largest_timestep = size_of_map + len(constraint_table)

    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'],root['timestep'])] = root
    
    while len(open_list) > 0:
        curr = pop_node(open_list)
        
        # returning goal node
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr)
        elif curr['timestep'] > largest_timestep:
            return None

        for dir in range(5):
            child_loc = move(curr['loc'], dir)

            # checking if child location is out of bounds of map
            if out_of_bounds(child_loc,my_map) or my_map[child_loc[0]][child_loc[1]]:
                continue
            # checking if 2x2 agent is out of bounds of map
            if size > 1:
                if sized_out_of_bounds(child_loc,my_map,size):
                    continue
            
            # child node
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            
            # checking if child node is already in CLOSED
            if (child['loc'],child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'],child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'],child['timestep'])] = child
                    push_node(open_list, child)
            # checking if child node is constrained
            elif is_constrained(curr['loc'],child['loc'],child['timestep'],constraint_table):
                continue
            # else push node to OPEN 
            else:
                closed_list[(child['loc'],child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
