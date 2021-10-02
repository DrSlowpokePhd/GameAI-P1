from heapq import heappop, heappush

# Helper function for calculating distance
def Euclidean_Distance(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    
# Helper for finding detail point in a destination box.
def Find_Detail_Point(source_point, source_box, destination_box):
    x_range = (max(source_box[0],destination_box[0]), min(source_box[1],destination_box[1]))
    y_range = (max(source_box[2],destination_box[2]), min(source_box[3],destination_box[3]))
    
    closest_point = (-1,-1)
    for x in range(x_range[0], x_range[1]+1):
        for y in range(y_range[0], y_range[1]+1):
            d1 = Euclidean_Distance(closest_point, source_point)
            d2 = Euclidean_Distance((x, y), source_point)
            if closest_point == (-1,-1) or d1 > d2:
                closest_point = (x,y)
    
    return closest_point

def find_path (source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point through the mesh
    Args: 
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to
    Returns: 
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    # Trying to implement bidirectional A*

    path = []
    forward_waypoints = {}  # dictonary containing the detail point in each box visited from src to dest.
    backward_waypoints = {} # dictonary containing the detail point in each box visited from dest to src.
    forward_boxes = {}      # previous box and cost to get to box for each box visited from src to dest.
    backward_boxes = {}     # previous box and cost to get to box for each box visited from src to dest.
    queue = []              # queue for Breadth First Search
    #print ("source point: ", source_point)
    #print ("destination point: ", destination_point)
    
    # 0 and 0 are placeholder values for the purpose of initializing source and destination box
    source_box, destination_box = 0, 0
    # find the source and destination boxes
    for i in mesh['boxes']:
        if source_point[0] >= i[0] and source_point[0] <= i[1] and source_point[1] >= i[2] and source_point[1] <= i[3]:
            #print("source box", i)
            source_box = i
            heappush(queue, (0, i, 'destination')) # Switched to the heap like are being used in the Dijkstra file because it keeps the list sorted.
            forward_boxes[source_box] = (source_box, 0)
            forward_waypoints[source_box] = (source_point)
        if destination_point[0] >= i[0] and destination_point[0] <= i[1] and destination_point[1] >= i[2] and destination_point[1] <= i[3]:
            #print("destination box", i)
            destination_box = i
            heappush(queue, (0, i, 'source'))
            backward_boxes[destination_box] = (destination_box, 0)
            backward_waypoints[destination_box] = (destination_point)
    
    # check for invalid source or destination.
    if source_box == 0 or destination_box == 0:
        print("No Path! Invalid source and/or destination!")
        return path, {}
    
    # bidriectional A* implementation
    final_box = 0
    while True:
        if not queue:
            print("No path!")
            for box in forward_boxes:
                boxes[box] = forward_boxes[box]
            for box in backward_boxes:
                boxes[box] = backward_boxes[box]
            return path, boxes.keys()
    
        priority, current_box, current_goal = heappop(queue)
        # print (current_box)
        # stop as soon as both searches overlap.
        if current_goal == 'destination':
            if current_box in backward_boxes:
                final_box = current_box
                break
        else:
            if current_box in forward_boxes:
                final_box = current_box
                break
        #print(mesh['adj'][current_box])
        for box in mesh['adj'][current_box]:
            if current_goal == 'destination':
                # Search from source to destination.
                # need to calculate total cost so far including next point.
                p1 = forward_waypoints[current_box]
                p2 = Find_Detail_Point(p1, current_box, box)
                cost = forward_boxes[current_box][1] + Euclidean_Distance(p1, p2)
                # estimated remaining cost.
                heuristic = Euclidean_Distance(p2, destination_point)
                if box not in forward_boxes or cost < forward_boxes[box][1]:
                    # make the priority cost so far + estimated remaining cost to make it A*.
                    heappush(queue, (cost + heuristic, box, 'destination'))
                    forward_boxes[box] = (current_box, cost)
                    forward_waypoints[box] = p2
            else:
                # Search from destination to source.
                # need to calculate total cost so far including next point.
                p1 = backward_waypoints[current_box]
                p2 = Find_Detail_Point(p1, current_box, box)
                cost = backward_boxes[current_box][1] + Euclidean_Distance(p1, p2)
                # estimated remaining cost.
                heuristic = Euclidean_Distance(p2, source_point)
                if box not in backward_boxes or cost < backward_boxes[box][1]:
                    # make the priority cost so far + estimated remaining cost to make it A*.
                    heappush(queue, (cost + heuristic, box, 'source'))
                    backward_boxes[box] = (current_box, cost)
                    backward_waypoints[box] = p2
    
    # construct path.
    forward_box = forward_boxes[final_box][0]
    backward_box = backward_boxes[final_box][0]

    path.append(forward_waypoints[final_box])
    path.append(backward_waypoints[final_box])
        
    while True:
        # exit loop when source and destination have been reached.
        if forward_box == source_box and backward_box == destination_box:
            if path[0] != source_point:
                path.insert(0, source_point)
            if path[-1] != destination_point:
                path.append(destination_point)
            break
        # reached source but not destination
        elif forward_box == source_box:
            if path[0] != source_point:
                path.insert(0, source_point)
            
            path.append(backward_waypoints[backward_box])
            backward_box = backward_boxes[backward_box][0]
        # reached destination but not source
        elif backward_box == destination_box:
            if path[-1] != destination_point:
                path.append(destination_point)
            
            path.insert(0, forward_waypoints[forward_box])
            forward_box = forward_boxes[forward_box][0]
        # reached neither
        else:
            path.insert(0, forward_waypoints[forward_box])
            forward_box = forward_boxes[forward_box][0]
            
            path.append(backward_waypoints[backward_box])
            backward_box = backward_boxes[backward_box][0]
    
    # print(mesh['adj'])
    boxes = {}
    
    # had issues trying to return both sets of boxes so I am combining them into one.
    for box in forward_boxes:
        boxes[box] = forward_boxes[box]
    for box in backward_boxes:
        boxes[box] = backward_boxes[box]
    
    return path, boxes.keys()
