# Assignment 1 COMP472 Summer 2020
# Seyedhossein Noorbakhsh
# 40036604
import geopandas as gp
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import math
from Node import Node
from Cell import Cell
import time

# Reading the db file
df = gp.read_file('./crime_dt.dbf')
x = df['geometry'].x
y = df['geometry'].y

# Grid setup
# You can choose different sizes of grid when prompted: (for example 0.003 means you have a 0.003*0.003 grid)
grid_size = float(input("Enter a grid size (for example 0.002 or 0.003): "))
bin_number = math.ceil(0.04 / grid_size)  # 0.04 is the distance between corner coordinates of the area under study
num_of_grid_cells = bin_number * bin_number
print("Bin_number is: ", bin_number, '\nNumber of grid cells is: ', num_of_grid_cells)

# Getting the 2d array of number of points
hist, xedges, yedges, _ = plt.hist2d(x, y, bins=[bin_number, bin_number])

# Creating a list so that we can have the points in grid cells as sorted values
num_points_in_cells = []

for first_dimension in hist:
    for cell in first_dimension:
        num_points_in_cells.append(cell)

num_points_in_cells_sorted = []
num_points_in_cells_sorted = num_points_in_cells.copy()
print(
    '----------------------------------------------------------------\n',
    'Here is the unsorted list of number of crimes in grid cells: \n',
    num_points_in_cells)
print(num_points_in_cells)
# Descending sort of the list of numbers in each grid cell
num_points_in_cells_sorted.sort(reverse=True)
print(
    '----------------------------------------------------------------\n',
    'Here is the sorted list of number of crimes in grid cells: \n',
    num_points_in_cells_sorted)

# Calculation of total crimes:
total_crimes_count = 0
for crimes in num_points_in_cells_sorted:
    total_crimes_count += crimes

# Calculation of average of crimes
average_crimes = total_crimes_count / num_of_grid_cells

print('----------------------------------------------------------------\nTotal number of crimes is: ',
      total_crimes_count, '\nAverage number of crimes is: ', average_crimes,
      '\n----------------------------------------------------------------')

# Defining a mutable threshold
threshold = float(input("\n Please enter a threshold (for example enter 0.8 for 80%): "))
threshold_index = int(num_of_grid_cells * (1 - threshold))
median_num_of_points = num_points_in_cells_sorted[threshold_index]
print('Threshold selected to be equal to: ', threshold * 100, '%\nThreshold index is: ', threshold_index,
      ' and the amount for that element is: ',
      median_num_of_points)

# Color definition for graph
cmap = colors.ListedColormap(['purple', 'yellow'])
bounds = [0, num_points_in_cells_sorted[threshold_index], num_points_in_cells_sorted[0]]
norm = colors.BoundaryNorm(bounds, cmap.N)

hist, _, _, _ = plt.hist2d(x, y, bins=[bin_number, bin_number], cmap=cmap, norm=norm)

# --------------------------------------------------------------------------------------------------
# here I created the nodes, assigned them numbers, and defined a boolean to mark all the border nodes.
nodes = []
node_number = 0  # this will be increased inside the for loop below
# (to assign node numbers to each node at build time)
for x in xedges:
    for y in yedges:
        if node_number <= bin_number or node_number >= ((bin_number + 1) ** 2) - 1 - bin_number or (
                node_number % (bin_number + 1)).__eq__(0) or ((node_number + 1) % (bin_number + 1)).__eq__(0):
            nodes.append(Node(None, x, y, node_number, border_node=True, hits_count=1))
        else:
            nodes.append(Node(None, x, y, node_number, border_node=False, hits_count=0))
            # for example if we have a 5 by 5 grid, then we'll have (6*6)-1 = 35 nodes. border nodes are 0 to 5,
            # multiples of 6 and (multiples of 6)-1
        node_number = node_number + 1
# for node in nodes:
#      print('node_number: ', node.node_number, ' border_node? -> ', node.border_node)

# --------------------------------------------------------------------------------------------------
# An array of cells, assigned them numbers, and defined the nodes that exists in that cell (corner nodes)

cells = []
j = 0
for i in range(num_of_grid_cells):
    list_of_nodes_in_cell = []
    list_of_nodes_in_cell.extend([nodes[j],  # lower left  node
                                  nodes[j + bin_number + 1],  # lower right node
                                  nodes[j + 1],  # upper left node
                                  nodes[j + bin_number + 1 + 1]])  # upper right node
    # list.extend is almost the same as append, the difference is that extend can take multiple args
    cell = Cell(i, list_of_nodes_in_cell)
    if num_points_in_cells[i] >= median_num_of_points:
        cell.high_risk = True
    cells.append(cell)
    # Test:
    # print(i, ' -> Nodes[', j, ' , ', j + bin_number + 1, ' , ', j + 1, ' , ',
    #       j + bin_number + 1 + 1, ']')
    if ((i + 1) % bin_number).__eq__(0):
        j = j + 2  # This is to increment the index number for nodes when we reach border
    else:
        j = j + 1  # we increment j here for traversing anyways!

# --------------------------------------------------------------------------------------------------
# Here I traverse inside all cells, if a cell is a high_risk cell, I increment the number of its node_hits by 1
# note that I already incremented the hits for border nodes by 1 at the time of creation
for cell in cells:
    for node in cell.list_of_nodes:
        if cell.high_risk.__eq__(True):
            node.hits_count += 1
            # Test:
            # print('\t Hits for node number ', node.node_number, ' incremented by 1 as it is in a high_risk cell.')
            # print('\t Hits for node number ', node.node_number, ' incremented by 1 as it is in a high_risk cell.')


    # --------------------------------------------------------------------------------------------------
    # to check if two nodes are diagonal to each other (and inside a common cell)
    def cell_index_finder_for_2_diagonal_nodes(node1: Node, node2: Node) -> int:
        for cell in cells:
            a_counter = 0
            for node in cell.list_of_nodes:
                if node.node_number == node1.node_number or node.node_number == node2.node_number:
                    a_counter = a_counter + 1
            if a_counter == 2:
                return cell.cell_number


    # to find two cells that are beside each other and have the same nodes on the border edge.
    def cells_finder_for_2_non_diagonal_nodes(node1: Node, node2: Node) -> []:
        list_of_2_cell_index_having_those_2_nodes = []
        for cell in cells:
            a_counter = 0
            for node in cell.list_of_nodes:
                if node.node_number == node1.node_number or node.node_number == node2.node_number:
                    a_counter = a_counter + 1
                if a_counter == 2:
                    list_of_2_cell_index_having_those_2_nodes.append(cell.cell_number)
        list_of_2_cell_index_having_those_2_nodes.sort()
        index_list = []
        index_list.extend(
            [list_of_2_cell_index_having_those_2_nodes[0], list_of_2_cell_index_having_those_2_nodes[1]])
        return index_list


    # defining a function to calculate cost of going from one node to other
    def cost_calculator(start_node: Node, end_node: Node):
        if start_node.__eq__(end_node):
            return 0
        else:
            # moving horizontally or vertically
            if start_node.x == end_node.x or start_node.y == end_node.y:
                if start_node.border_node == True and end_node.border_node == True:  # if both nodes are border nodes
                    if (start_node.hits_count + end_node.hits_count) < 4 and cells[
                        cell_index_finder_for_2_diagonal_nodes(start_node, end_node)].high_risk == False:
                        return 1.3
                    else:
                        return 1000
                # if at least one of the nodes is not a border node
                else:
                    if (start_node.hits_count + end_node.hits_count) < 2:
                        return 1
                    if (start_node.hits_count + end_node.hits_count) >= 2 and (
                            start_node.hits_count + end_node.hits_count) < 6 and not (
                            cells[cells_finder_for_2_non_diagonal_nodes(start_node, end_node)[0]].high_risk == True and
                            cells[cells_finder_for_2_non_diagonal_nodes(start_node, end_node)[1]].high_risk == True):
                        return 1.3
                    else:
                        return 1000
            else:
                # moving diagonally
                if start_node.border_node == True or end_node.border_node == True:
                    if (start_node.hits_count + end_node.hits_count) < 4:
                        return 1.5
                    else:
                        return 10000
                else:  # moving diagonally between two non-border nodes
                    if cells[cell_index_finder_for_2_diagonal_nodes(start_node, end_node)].high_risk:
                        return 10000
                    else:
                        return 1.5

# adding the node_numbers on the plot
for node in nodes:
    plt.annotate(node.node_number, xy=(node.x, node.y))


# The following functions will be used to determine the correct index of children nodes
def all_possible_children_of_a_node(node_index):  # This function return all possible children index of a node
    # index 0 is top, then it goes clockwise, so that index 7 will be
    # the node at top left.
    possible_indices_for_children = []
    possible_indices_for_children.extend([
        node_index + 1,  # top
        node_index + bin_number + 2,  # top_right (diagonal)
        node_index + bin_number + 1,  # right
        node_index + bin_number,  # lower_right (diagonal)
        node_index - 1,  # bottom
        node_index - bin_number - 2,  # lower left (diagonal)
        node_index - bin_number - 1,  # left
        node_index - bin_number  # top_left
    ])

    return possible_indices_for_children


def list_of_children_of_a_node(a_node: Node) -> []:
    # This function choose the correct indices for children nodes and remove the invalid ones
    # based on the position of the node (which border it is on)
    node_index = a_node.node_number
    correct_children = []
    possible_children_index = all_possible_children_of_a_node(node_index)
    if a_node.border_node:
        if node_index <= bin_number:  # Node is on the left borderline
            if node_index == 0:  # Node is the origin point  of the grid
                correct_children.extend(
                    [
                        nodes[possible_children_index[0]],
                        nodes[possible_children_index[1]],
                        nodes[possible_children_index[2]]
                    ])
            return correct_children
            if node_index == bin_number:  # Node is the upper left corner point  of the grid
                correct_children.extend(
                    [nodes[possible_children_index[3]]])
                return correct_children
            else:  # Node is between the origin point and the upper left corner point  of the grid
                correct_children.extend(
                    [nodes[possible_children_index[1]],
                     nodes[possible_children_index[2]],
                     nodes[possible_children_index[3]],
                     ])
                return correct_children
        if node_index >= (len(nodes) - 1 - bin_number):  # Node is on the right borderline
            if node_index == (bin_number + 1) ** 2 - 1:  # Top right corner point (last node!)
                correct_children.extend(
                    [nodes[possible_children_index[5]]]
                )
                return correct_children
            if node_index == len(nodes) - 1 - bin_number:  # Node is the lower right corner point  of the grid
                correct_children.extend([
                    nodes[possible_children_index[7]]])
                return correct_children
            else:  # Node is between the last node and the lower right corner node
                correct_children.extend(
                    [nodes[possible_children_index[5]],
                     nodes[possible_children_index[6]],
                     nodes[possible_children_index[7]]])
                return correct_children
        else:  # Top and bottom border lines:
            # node on the bottom border line
            if node_index % (bin_number + 1) == 0:
                correct_children.extend(
                    [nodes[possible_children_index[0]],
                     nodes[possible_children_index[1]],
                     nodes[possible_children_index[2]],
                     nodes[possible_children_index[6]],
                     nodes[possible_children_index[7]]])
                return correct_children
            # Node on the top borderline
            if (node_index + 1) % (bin_number + 1) == 0:
                correct_children.extend(
                    [nodes[possible_children_index[3]],
                     nodes[possible_children_index[4]],
                     nodes[possible_children_index[5]]
                     ])
                return correct_children
    else:  # Node is not a border node (all possible nodes are good!)
        correct_children.extend(
            [nodes[possible_children_index[0]], nodes[possible_children_index[1]],
             nodes[possible_children_index[2]], nodes[possible_children_index[3]],
             nodes[possible_children_index[4]], nodes[possible_children_index[5]],
             nodes[possible_children_index[6]], nodes[possible_children_index[7]]])
        return correct_children
    # # Test:
    # test_list = []
    # test_list = list_of_children_of_a_node(nodes[435])
    # results = []
    # for node in test_list:
    #     results.append(node.node_number)
    # print('These are the correct children indices for that node: ', results)


#############################################################################################################
#                                 A* algorithm and heuristic Function                                       #
#############################################################################################################
# Definition of the heuristic function which returns a scaled distance between the origin node to the goal
def heuristic(origin: Node, goal: Node):
    scale = bin_number / 0.04
    return scale * math.sqrt((origin.x - goal.x) ** 2 + (origin.y - goal.y) ** 2)


# creating 2 lists for open and closed lists
open_list = []
closed_list = []

# asking user for start and end nodes
upper_range_for_nodes = str(len(nodes) - 1)

selected_start = int(
    input("\nEnter a node number for the start node: (can be chosen between 0 to " + upper_range_for_nodes + ' ):'))
selected_end = int(
    input("Enter a node number for the start node: (can be chosen between 0 to " + upper_range_for_nodes + ' ):'))

# Runtime will be calculated from this point on:
start_time = time.time()

start_node = nodes[selected_start]
end_node = nodes[selected_end]

start_node.parent = None
start_node.f = start_node.g = start_node.h = 0
end_node.f = end_node.g = end_node.h = 0

# adding start node to open_list
open_list.append(start_node)
current_node = open_list[0]
total_cost = 0
path = []

while len(open_list) > 0:
    # Get the current node
    current_node = open_list[0]

    current_index = 0
    for index, node in enumerate(open_list):
        if node.f < current_node.f:
            current_node = node
            current_index = index

    open_list.pop(current_index)
    closed_list.append(current_node)

    children = list_of_children_of_a_node(current_node)

    if current_node.node_number == end_node.node_number:
        print('A path has been found from node_number: ', start_node.node_number, ' to node_number: ',
              end_node.node_number)

        x_values_of_path = []
        y_values_of_path = []
        current = current_node
        start_node.parent = None
        while current is not None:
            path.append(current)
            current = current.parent

        path.reverse()
        plt.scatter(start_node.x, start_node.y, marker='s')
        plt.scatter(end_node.x, end_node.y, marker='*')

        print("The path would be: ", end='')

        for node in path:
            if node.__eq__(end_node):
                print(node.node_number, '.')
            else:
                print(node.node_number, end=''), print(' -> ', end='')
            total_cost = total_cost + node.g
            x_values_of_path.append(node.x)
            y_values_of_path.append(node.y)

        print("Total time spent for generating the path is: ", time.time() - start_time, ' seconds.')
        print("The total cost calculated for this path would be: ", total_cost)
        plt.plot(x_values_of_path, y_values_of_path, '-', color='white')
        plt.show()
        exit()

    for child in children:
        breaking_for_loop = False

        for node in closed_list:
            if node.node_number == child.node_number:
                breaking_for_loop = True
                open_list[:] = (item for item in open_list if item.node_number != node.node_number)
                continue
        if breaking_for_loop:
            continue

        child.g = current_node.g + cost_calculator(current_node, child)
        if child.g >= 1000:  # if cost >= 1000 that means the node is inaccessible
            continue
        child.h = heuristic(child, end_node)
        child.f = child.g + child.h
        child.parent = current_node

        child_already_in_open_list = False
        for node in open_list:
            if child.node_number == node.node_number:
                child_already_in_open_list = True
                if child.g < node.g:
                    open_list[:] = (item for item in open_list if item.node_number != node.node_number)
                    open_list.append(child)
        # if child does not exist in open list, or if it is closer to end_node should be added to open list
        if not child_already_in_open_list:
            open_list.append(child)

    if time.time() - start_time > 10:
        print(time.time() - start_time)
        print('Time is up. The optimal path is not found.')
        exit()

plt.show()
