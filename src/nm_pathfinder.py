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

    # part 1: Breadth first search

    path = []
    boxes = {} # changed back to dictionary
    queue = [] # queue for Breadth First Search
    print ("source point: ", source_point)
    print ("destination point: ", destination_point)
    
    # 0 and 0 are placeholder values for the purpose of initializing source and destination box
    source_box, destination_box = 0, 0
    # find the source and destination boxes
    for i in mesh['boxes']:
        if source_point[0] >= i[0] and source_point[0] <= i[1] and source_point[1] >= i[2] and source_point[1] <= i[3]:
            print("source box", i)
            #path.append(source_point)
            source_box = i
            queue.append(i)
            boxes[source_box] = source_box
        if destination_point[0] >= i[0] and destination_point[0] <= i[1] and destination_point[1] >= i[2] and destination_point[1] <= i[3]:
            print("destination box", i)
            #path.append(destination_point)
            destination_box = i
    
    # check for invalid source or destination.
    if source_box == 0 or destination_box == 0:
        print("No Path! Invalid source and/or destination!")
        return path, boxes
    
    # the ACTUAL BFS implementation, box to box for now
    while True:
        if not queue:
            print("No path!")
            return path, boxes
    
        current_box = queue.pop()
        # print (current_box)
        if current_box is destination_box:
            break
        #print(mesh['adj'][current_box])
        for box in mesh['adj'][current_box]:
            if box not in boxes and box not in queue:
                queue.insert(0, box)
                boxes[box] = current_box
            
        #boxes.append(current_box)
    
    # construct path. Case 1: Src and Dest in same box. Case 2: Src and Dest in different boxes. TEST VERSION, DOESN'T MEET REQUIREMENTS (draws lines between midpoints)
    path.append(destination_point)
    box = destination_box
    while True:
        if boxes[box] == source_box:
            path.append(source_point)
            break
        else:
            box = boxes[box]
            path.append((box[0] + (box[1] - box[0])/2, box[2] + (box[3] - box[2])/2))
    
    # print(mesh['adj'])
    return path, boxes.keys()
