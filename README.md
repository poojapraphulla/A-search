# A-search
import numpy as np 

class NodeA:
    """
        A class for a node is created that will contain all
        the attributes associated with the node like the parent
        of the node, position of the node, and all three costs(g,h & f)
        for the node.
        
        parentNode is the parent of the current node n 
        
        currentPosition is the current position of the node n in maze
        
        g(n) = g is the cumulative cost to reach from start node to current node n 
        
        h(n) = h is the estimated cost of cheapest path from current node n to a goal node
    """
    
    def __init__(a, parentNode=None, currentPosition=None):
        
        #Initializing a node 
        a.parentNode = parentNode
        a.currentPosition = currentPosition
        
        a.g = 0
        a.h = 0
        a.f = 0
        
    def __equality__(a, another):
        
        #Building a method for checking the equality of the node with another node 
        return a.currentPosition == another.currentPosition
        
    def return_path(n, mazegame):
    
        #This function returns the path of the search
        path = []
        no_of_rows, no_of_columns = np.shape(mazegame)
        
        #The initialized result maze is created with -1 in every position
        resultMaze = [[-1 for i in range(no_of_columns)] for j in range(no_of_rows)]
        current = n
        while current is not None:
            path.append(current.currentPosition)
            current = current.parentNode
            
        #The reversed path is returned in order to show the path from the start node to the goal node
        path = path[::-1]
        starting_value = 0
        
        #The path from the start node to the goal node obtained through A* search is updated with every step
        #incremented by -1
        for i in range(len(path)):
            resultMaze[path[i][0]][path[i][1]] = starting_value
            starting_value += 1 
        return resultMaze
        
    def search(mazegame, cost, start, end):
        """
            Returns a list of tuples as a path from the given start node to the given goal node in the given mazegame
            :parameter mazegame:
            :parameter cost
            :parameter start:
            :parameter end:
            :return:
        """
        
        #The start node is created with initialized values for g, h and f
        start_node = Node(None, tuple(start))
        start_node.g = start_node.h = start_node.f = 0
        #The end node is created with initialized values for g, h and f
        end_node = Node(None, tuple(end))
        end_node.g = end_node.h = end_node.f = 0
        
        #The lowest cost node is to be expanded next
        #Initializing frontier list in which nodes which are yet to be explored are put 
        frontier_list = []
        #Initializing closed list in which nodes already explored are put
        closed_list = []
        #The start node is added
        frontier_list.append(start_node)
        
        #To avoid infinte loops, a stop condition is added
        outer_iterations = 0
        max_iterations = (len(mazegame) // 2) ** 10
        
        #There exists four movements for each position
        move = [[-1, 0 ], # movement to go up
                [ 0, -1], # movement to go left 
                [ 1, 0 ], # movement to go down
                [ 0, 1 ]] # movement to go right
                
        #find mazegame has got how many rows and columns 
        no_of_rows, no_of_columns = np.shape(mazegame)
        
        #loop
        while len(frontier_list) > 0:
            #counter of limit operation is incremented for every time any node is referred
            outer_iterations += 1 
            
            #get the current node
            n = frontier_list[0]
            current_index = 0 
            for index, item in enumerate(frontier_list):
                if item.f < n.f:
                    n = item
                    current_index = index
                    
            #Return the path as computation cost is too high or it does not have any solution
            if outer_iterations > max_iterations:
                print (" Not able to find the path as there are too many iterations")
                return return_path(n, mazegame)
        
        #Maintaining and updating the frontier and closed lists
        #Pop out current node out off the frontier list
        frontier_list.pop(current_index)
        #Add to the closed list
        closed_list.append(n)
        
        #Testing for goal node
        if n == end_node:
            return return_path(n, mazegame)
            
        #Generating children from all adjacent squares
        children = []
        
        for new_position in move:
            #Get node position
            node_position = (n.currentPosition[0] + new_position[0],
                             n.currentPosition[1] + new_position[1])
            
            #Make sure that it is within the range of the maze boundary
            if (node_position[0] > (no_of_rows - 1) or
                node_position[0] < 0 or
                node_position[1] < (no_of_columns - 1) or
                node_position[1] < 0):
                    continue
                
            if mazegame[node_position[0]][node_position[1]] != 0:
                continue
            
            #Create new node
            new_node = Node(n, node_position)
            
            #Appending
            children.append(new_node)
            
            #Loop through children
            for child in children:
                
                #Child is in the closed_list, so search the entire list
                if len([closed_list for closed_list in closed_list if closed_list == child]) > 0:
                    continue
                
                #creating the f, g and h values
                child.g = n.g + cost
                #Heuristic costs are calculated using euclidean distance
                child.h = (((child.currentPosition[0] - end_node.currentPosition[0]) ** 2) + 
                           ((child.currentPosition[1] - end_node.currentPosition[1]) ** 2))
                           
                child.f = child.g + child.h 
                
                #Child is already in frontier list and g cost is already lower
                if len([i for i in frontier_list if child == i and child.g > i.g]) > 0:
                    continue
                
                #Add the child to the frontier list
                frontier_list.append(child) 

def main():
  mazegame = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
                        
  start = [0,0] #starting position
  end = [7,6] #ending position
  cost = 1 #cost per movement
            
  path = search(mazegame, cost, start, end)
  print(path)

if __name__ == '__main__':
  main()              
