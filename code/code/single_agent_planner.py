import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
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


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = {}
    for constraint in constraints:
        if(constraint['agent'] == agent):
            constraint_table.setdefault(constraint['timestep'],[]).append(constraint['loc'],constraint['positive'])
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if(next_time in constraint_table):
        for constraint in constraint_table[next_time]:
            table_len = len(constraint)
            if(table_len == 1):
                if(constraint[0] == next_loc):
                    return True
            elif(table_len == 2):
                if(constraint[0] == curr_loc and constraint[1] == next_loc):
                    return True 
            else:
                print("Error(is_constrained): Too many arguments in table")
    # 2.3 checking for additional constraints (agents that have reached goal node)
    if(-1 in constraint_table):
        for constraint in constraint_table[-1]:
            if next_loc == constraint[0]:
                return True
    return False        


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


# 1.4 finds the earlist goal time step
def find_earlist_goal_timestep(goal_loc,constraint_table):
    earliest_goal_timestep = 0
    for timestep in constraint_table:
        for constraint in constraint_table[timestep]:
            table_len = len(constraint) 
            if(table_len == 1 and constraint[0] == goal_loc and timestep > earliest_goal_timestep):
                earliest_goal_timestep = timestep
    return earliest_goal_timestep

# 3.4 checking bounds of board 
def out_of_bounds(child_loc,my_map):
    return child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0])

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    open_list = []
    closed_list = dict()

    # 1.2 building a constraint table
    constraint_table = build_constraint_table(constraints,agent)
    # 1.4 finding earlist goal time step for goal constraints
    earliest_goal_timestep = find_earlist_goal_timestep(goal_loc,constraint_table)
    # 2.4 finding upper bound on path length/timestep
    size_of_map =  sum([i.count(False) for i in my_map])
    largest_timestep = size_of_map + len(constraint_table)

    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'],root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['timestep'] > earliest_goal_timestep:
                return get_path(curr)
        elif curr['timestep'] > largest_timestep:
            return None
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            # 3.4 checking if child location is out of bounds of map
            if(out_of_bounds(child_loc,my_map)):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            if (child['loc'],child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'],child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'],child['timestep'])] = child
                    push_node(open_list, child)
            # 1.2 checking for vertex constraints
            elif is_constrained(curr['loc'],child['loc'],child['timestep'],constraint_table):
                continue
            else:
                closed_list[(child['loc'],child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
