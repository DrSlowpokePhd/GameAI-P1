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
    boxes = [] # {} # had to change the dict to a list due to issues in nm_interactive.py
    queue = [] # queue for Breadth First Search
    print ("source point: ", source_point)
    print ("destination point: ", destination_point)
    
    # 0 and 0 are placeholder values for the purpose of initializing source and destination box
    source_box, destination_box = 0, 0
    # find the source and destination boxes
    for i in mesh['boxes']:
        if source_point[0] > i[0] and source_point[0] < i[1] and source_point[1] > i[2] and source_point[1] < i[3]:
            print("source box", i)
            path.append(source_point)
            destination_box = i
        if destination_point[0] > i[0] and destination_point[0] < i[1] and destination_point[1] > i[2] and destination_point[1] < i[3]:
            print("source box", i)
            path.append(destination_point)
            source_box = i
            queue.append(i)
    # the ACTUAL BFS implementation, box to box for now
    
    while queue:
        current_box = queue.pop()
        # print (current_box)
        if current_box is destination_box:
            break
        print(mesh['adj'][current_box])
        for box in mesh['adj'][current_box]:
            if box not in boxes:
                queue.insert(0, box)
            
        boxes.append(current_box)

    # print(mesh['adj'])
    return path, boxes
