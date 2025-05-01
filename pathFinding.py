import random
import statistics

maze = []  
barriers = []

#Creation of the maze as 2d list. All the values are 0 inside the maze in the initialization
for i in range(6):
    maze.append([])
    for j in range(6):
        maze[i].append(0)

#Random;y selects the starting node. Has to be within 1 and 11 th block in the maze
def Start_Node_Selection():
    start_col = random.choice([0,1])
    start_row = random.randint(0,5)
    
    maze[start_row][start_col] = "Start"
    return block_number(start_row,start_col)
    
#Random;y selects the goal node. Has to be within 24 and 35 th block in the maze
def Goal_Node_Selection(): 
    Goal_col = random.choice([4,5])
    Goal_row = random.randint(0,5)

    maze[Goal_row][Goal_col] = "Goal"
    return block_number(Goal_row, Goal_col)

#Randomly assigns barriers
def barrier_Selection():
    barrier_count = 0
    while barrier_count < 4:   
        barr_col = random.randint(0,5)
        barr_row = random.randint(0,5)
    
        if maze[barr_row][barr_col] == 0:  #Checks whether the node is already occupied
            maze[barr_row][barr_col] = "Barrier"
            barrier_count += 1
            barriers.append(block_number(barr_row,barr_col))
        else:
            continue

def maze_setup():
    global start_node 
    global goal_node 

    start_node = Start_Node_Selection()
    goal_node = Goal_Node_Selection()
    barrier_Selection()

    print("Goal",goal_node)
    print("Start",start_node)
 

#Used to get the block number of the node when row and column ara given    
def block_number(row,col):      
    if col==0:
        return row
    else:
        return col*6+row

#Used to get the row and column numebers back from the block number
def get_row_col(node): 
    row = node % 6
    col = node // 6
    return row, col

    
#Function to get the available neighbors of a node
def get_neighbors(current_block):
    current_row, current_col = get_row_col(current_block)
    neighbors_list = []

    moves = [-1, 0,+1]     #All posible moves(vertically, horizontally and diagonally)

    for col in moves:
        for row in moves:
            if (col == 0 and row== 0) :
                continue

            neighbor_row = current_row+row
            neighbor_col = current_col+col

            if neighbor_row < 0 or neighbor_row >= 6 or neighbor_col < 0 or neighbor_col >= 6:    #Checks whether the block is inside the maze           
                continue

            neighbor_block = block_number(neighbor_row, neighbor_col)

            if neighbor_block in barriers :      #Ommited if teh neighbor node is a barrier
                continue

            if abs(row) == 1 and abs(col) == 1:  
                cost = 1.5   #Added to the path cost. This is for a diagoal neighbor.Indicates the distance gap with vertical or horizonatal moves
            else:  
                cost = 1     #path cost of vertical or horizontal neighbor

            neighbors_list.append((neighbor_block,cost))

    neighbors_list.sort(key=lambda x: x[0])     #Sorted as the ascending block numbers
    return neighbors_list




import heapq

def uniform_cost(start_node, goal_node):

    open_list = [(0, start_node)]    #To store all the available nodes. Stored with the corresponding path cost.
    visited_list = []             #To record all the visited nodes taken from the openset.

    time = 0

    parents = {start_node:None}       #To keep record of the parent node of each node. Used in backtracking the final path from goal to start node.

    path_costs = {start_node:0}       #Records all the path costs at a moment for every node in the open list

    while open_list:               #Runs until the open list is empty
        current_cost, current_node = heapq.heappop(open_list)     #Removes the node with the lowest path cost from the open list

        if current_node in visited_list:             
            continue
            
        visited_list.append(current_node)            #Added to the visited list to stop revisiting to the node.
        time += 1             #Time to explore a single node is taken as 1

        if current_node == goal_node:    #If the current node is the goal, loop stops and path is backtracked
            path = []
            node = current_node
            while node is not None:
                path.append(node)
                node = parents[node]
            path.reverse()              #Path is reversed before returning for better clarity
            return visited_list, time, path
        
        neighbors = get_neighbors(current_node)

        for neighbor,cost  in neighbors:

            neighbor_path_cost = path_costs[current_node] + cost           #The total path cost from start to the neighbor node.

            if neighbor not in path_costs or path_costs[neighbor]>neighbor_path_cost:     #Checks whether if the current neighbor node is recorded before or have a smaller path than the current path 
                path_costs[neighbor] = neighbor_path_cost                   # Added to the path cost dictionary as the smallest path cost to the current neighbor node
                parents[neighbor] = current_node
                heapq.heappush(open_list,(neighbor_path_cost,neighbor))        #Added to the open list with the total path cost
        
    return visited_list, time, None




def heuristic_distance(current_node, goal_node):              # Calculates the distance between a node and the goal node.
    current_node_row, current_node_col = get_row_col(current_node)
    goal_node_row, goal_node_col = get_row_col(goal_node)
    return max(abs(current_node_row - goal_node_row), abs(current_node_col- goal_node_col))


#Same method is used as the uniform cost search, but insted of just path costs, summation of path costs and heuristic distance is calculated for each node and used in comparisons for choosing the path 
def A_star_traversal(start_node, goal_node):

    open_list = [(heuristic_distance(start_node,goal_node)+0, start_node)]    #Summation of path cost and heuristic distance is stored
    visited_list = []

    time = 0

    parents = {start_node:None}

    path_costs = {start_node:0}
    Total_distance = {start_node:heuristic_distance(start_node,goal_node)+0}

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)          #  Node with the total distance is taken first

        if current_node in visited_list:
            continue
            
        visited_list.append(current_node)
        time += 1

        if current_node == goal_node:
            path = []
            node = current_node
            while node is not None:
                path.append(node)
                node = parents[node]
            path.reverse()
            return visited_list, time, path
        
        neighbors = get_neighbors(current_node)

        for neighbor,cost  in neighbors:

            neighbor_path_total_distance = path_costs[current_node] + cost + heuristic_distance(neighbor,goal_node)

            if neighbor not in Total_distance or Total_distance[neighbor]>neighbor_path_total_distance:
                Total_distance[neighbor] = neighbor_path_total_distance
                path_costs[neighbor] = path_costs[current_node] + cost
                parents[neighbor] = current_node
                heapq.heappush(open_list,(neighbor_path_total_distance,neighbor))
        
    return visited_list, time, None






ucs_times = []
ucs_path_lengths = []
astar_times = []
astar_path_lengths = []
start_node = Start_Node_Selection()
goal_node = Goal_Node_Selection()

for i in range(3):
    print(f"\n Maze {i+1}")
    maze_setup()
    
    # Uniform Cost Search
    visited, total_time, path = uniform_cost(start_node, goal_node)
    print("\nUniform Cost Search")
    print("Visited nodes:", visited)
    print("Time:", total_time, "minutes")
    print("Final path:", path)
    ucs_times.append(total_time)
    ucs_path_lengths.append(len(path) if path else 0)
    
    # A* Search
    visited_a_star, total_time_a_star, path_a_star = A_star_traversal(start_node, goal_node)
    print("\nA* Search")
    print("Visited nodes:", visited_a_star)
    print("Time:", total_time_a_star, "minutes")
    print("Final path:", path_a_star)
    astar_times.append(total_time_a_star)
    astar_path_lengths.append(len(path_a_star) if path_a_star else 0)

# Analysis
print("\nSummary")
print("Uniform Cost Search:")
print("Mean Time:", statistics.mean(ucs_times), "minutes")
print("Variance Time:", statistics.variance(ucs_times) if len(ucs_times) > 1 else 0)
print("Mean Path Length:", statistics.mean(ucs_path_lengths))
print("Variance Path Length:", statistics.variance(ucs_path_lengths) if len(ucs_path_lengths) > 1 else 0)

print("\nA* Search:")
print("Mean Time:", statistics.mean(astar_times), "minutes")
print("Variance Time:", statistics.variance(astar_times) if len(astar_times) > 1 else 0)
print("Mean Path Length:", statistics.mean(astar_path_lengths))
print("Variance Path Length:", statistics.variance(astar_path_lengths) if len(astar_path_lengths) > 1 else 0)
             





