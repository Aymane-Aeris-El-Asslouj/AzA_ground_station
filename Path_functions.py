import Path_objects
import Parameters

PATHS_PER_WAYPOINTS = Parameters.PATHS_PER_WAYPOINTS
MAX_ATTEMPTS_PER_WAYPOINTS = Parameters.MAX_ATTEMPTS_PER_WAYPOINTS


# from a set of 2d waypoints, returns a list of 2d straight paths that connect them while avoiding obstacles
# this list contains, for each two waypoints given, a list of up to 10 paths connecting those 2 waypoints
def Straight_path_finder(Start_position, Mission_waypoint_list, Obstacles, Border):
    # Create the total number of nodes to connect by adding the starting position as a node to the mission waypoints
    True_waypoint_list = [Start_position] + Mission_waypoint_list

    # Each two waypoints from the true waypoint list will be connected by up to PATHS_PER_WAYPOINTS straight line
    # paths. These paths get grouped into a list that corresponds to each two waypoints. Then these lists of paths
    # get grouped into the "Straight_line_paths" list for the entire mission.
    Straight_line_paths = list()

    # Nodes creatable on border
    Border_nodes = Border.Create_vertex_nodes()

    # loop through each waypoint in the list except the last one, and try to connect it to the following waypoint
    for Waypoint_index in range(len(True_waypoint_list) - 1):
        List_of_paths_found = list()
        Current_waypoint = True_waypoint_list[Waypoint_index]
        Next_waypoint = True_waypoint_list[Waypoint_index + 1]
        Paths_being_searched = [Path_objects.Path([Current_waypoint])]

        # avoid endless search
        Attempts = 0
        # keep looking for paths till running out of paths to explore or find 10 paths connecting the two waypoints
        while len(Paths_being_searched) > 0 and len(
                List_of_paths_found) < PATHS_PER_WAYPOINTS:
            Attempts += 1
            # generate extensions of first part in list of paths begin searched by adding nodes to it
            # extensions are done by either connecting the path to the next waypoint, to nodes creates on obstacles
            # or nodes created on the border
            Path_extensions = list()

            # Fine the path that is the shortest while adding remaining distance to next waypoint
            Best_path = min(Paths_being_searched, key=lambda Path_i: Path_i.Path_distance_to_waypoint(Next_waypoint))

            # get the last node of first best path
            Last_node = Best_path.Waypoint_list[len(Best_path.Waypoint_list) - 1]

            # check if we can connect the Last_node to the next waypoint
            if Last_node.Is_connectable_to(Next_waypoint, Obstacles, Border):
                Path_extensions.append(Path_objects.Path(Best_path.Waypoint_list + [Next_waypoint]))

            # extend path by connecting it to nodes created on obstacles
            for Obstacle_i in Obstacles:
                # create two nodes on each side of each obstacle
                Node_1, Node_2 = Obstacle_i.Create_tangent_nodes(Last_node)
                if Node_1 is not None:
                    if Last_node.Is_connectable_to(Node_1, Obstacles, Border):
                        Path_extensions.append(Path_objects.Path(Best_path.Waypoint_list + [Node_1]))
                    if Last_node.Is_connectable_to(Node_2, Obstacles, Border):
                        Path_extensions.append(Path_objects.Path(Best_path.Waypoint_list + [Node_2]))

            # extend path by connecting it to nodes created on border
            for Node_i in Border_nodes:
                if Last_node.Is_connectable_to(Node_i, Obstacles, Border):
                    Path_extensions.append(Path_objects.Path(Best_path.Waypoint_list + [Node_i]))

            # add extensions of best path that did not reach the next waypoint yet to the list of paths
            # being searched and the others to list of paths found
            for New_path in Path_extensions:
                # get last node of path
                Last_node_of_path = New_path.Waypoint_list[len(New_path.Waypoint_list) - 1]
                # Check if new node is not already in the path
                if not Best_path.Contains_waypoint(Last_node_of_path):
                    # check if path is useful (not bouncing off obstacle/border vertex or going backwards)
                    Is_useful = True
                    # cannot bounce without 3 nodes
                    if len(New_path.Waypoint_list) > 2:

                        # found before_last_node
                        Before2_last_node = New_path.Waypoint_list[len(New_path.Waypoint_list) - 3]
                        Before_last_node = New_path.Waypoint_list[len(New_path.Waypoint_list) - 2]
                        Last_node = New_path.Waypoint_list[len(New_path.Waypoint_list) - 1]

                        # Check If path is going backwards
                        if Before_last_node.Is_going_backwards(Before2_last_node, Last_node):
                            Is_useful = False

                        # Check if path is bouncing
                        elif Before_last_node.Is_bouncing(Before2_last_node, Last_node):
                            Is_useful = False

                    if Is_useful:
                        # check if last node of new found path is the next waypoint
                        if Last_node_of_path.Coincides_with(Next_waypoint):
                            List_of_paths_found.append(New_path)
                        else:
                            Paths_being_searched.append(New_path)

            # delete the path that we're done exploring
            Paths_being_searched.remove(Best_path)

            # If too many attempts return no path found
            if Attempts >= MAX_ATTEMPTS_PER_WAYPOINTS:
                return list()

        # If no path found for specific waypoint, return no path found
        if len(List_of_paths_found) == 0:
            return list()

        # add the found paths to the list of mission paths
        Straight_line_paths.append(List_of_paths_found)
    return Straight_line_paths


# from a list of 2d straight paths, gets a list of 2d alleviated paths with alleviation waypoints added
def Alleviator(Straight_path_list, Obstacles, Border):
    return Straight_path_list


# from a list of 2d alleviated paths, get 3d alleviated paths with altitude set for the waypoints missing it
def Altitude_analysis(Alleviated_path_list):
    return Alleviated_path_list


# from a list of 3d alleviated paths, get a final chosen stitched path
def Path_selector(Full_path_list):
    # This code is temporary ##########################################
    Stitched_paths = list()
    # if no path found, return none
    if len(Full_path_list) == 0:
        print("Error: No valid path found for mission waypoints")
        return Stitched_paths
    # currently creating as many paths as possible with a different path for each two waypoints each path
    for Path_list_index in range(max(len(Path_group_i) for Path_group_i in Full_path_list)):
        Chosen_paths = list(Path_list_i[min(Path_list_index, len(Path_list_i) - 1)] for Path_list_i in Full_path_list)
        Stitched_paths.append(Path_objects.Path([Full_path_list[0][0].Waypoint_list[0]] + sum(list(
            Path_i.Waypoint_list[1:] for Path_i in Chosen_paths), [])))

    return Stitched_paths
