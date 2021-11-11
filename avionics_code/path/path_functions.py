import path.path_objects as p_o
import helpers.parameters as para
import copy
import helpers.geometrical_functions as g_f

PATHS_PER_WAYPOINTS = para.PATHS_PER_WAYPOINTS
MAX_ATTEMPTS_PER_WAYPOINTS = para.MAX_ATTEMPTS_PER_WAYPOINTS
VARIATIONS_FOR_PATH_BUILDING = para.VARIATIONS_FOR_PATH_BUILDING
MAX_CLIMBING_RATIO = para.MAX_CLIMBING_RATIO


def Compute_path(self):
    if len(self.Mission_waypoint_list) > 0:
        self.Straight_2D_paths_list = p_f.Straight_2D_path_finder(self)
        self.Curved_2D_paths_list = p_f.Curving_2D(self, self.Straight_2D_paths_list)
        self.Curved_3D_paths_list = p_f.Altitude_extension(self.Curved_2D_paths_list)
        self.Chosen_3D_path = p_f.Path_selector(self.Curved_3D_paths_list)
    else:
        self.Straight_2D_paths_list = list()
        self.Curved_2D_paths_list = list()
        self.Curved_3D_paths_list = list()
        self.Chosen_3D_path = list()

"""from a set of 2d waypoints, returns a list of 2d
straight paths that connect them while avoiding obstacles
this list contains, for each two waypoints given,
a list of up to 10 paths connecting those 2 waypoints"""
def Straight_2D_path_finder(Mission_profile):

    Start_position = Mission_profile.Current_position
    Mission_waypoint_list = Mission_profile.Mission_waypoint_list
    Obstacles = Mission_profile.Obstacles
    Border = Mission_profile.Border

    """Create the total number of nodes to connect by adding the
    starting position as a node to the mission waypoints"""
    True_waypoint_list = [Start_position] + Mission_waypoint_list

    """Each two waypoints from the true waypoint list will be connected by up to PATHS_PER_WAYPOINTS straight line
    paths. These paths get grouped into a list that corresponds to each two waypoints, a path group. Then these
    path groups get grouped into the "Straight_paths_list" list for the entire mission."""
    Straight_paths_list = list()

    # Nodes creatable on border
    Border_nodes = Border.Create_vertex_nodes()

    # loop through each waypoint in the list except the last
    # one, and try to connect it to the following waypoint
    for Waypoint_index in range(len(True_waypoint_list) - 1):
        List_of_paths_found = list()
        Current_waypoint = True_waypoint_list[Waypoint_index]
        Next_waypoint = True_waypoint_list[Waypoint_index + 1]
        Paths_being_searched = [p_o.Path([Current_waypoint])]

        # avoid endless search
        Attempts = 0
        # keep looking for paths till running out of paths to
        # explore or find 10 paths connecting the two waypoints
        while len(Paths_being_searched) > 0 and len(List_of_paths_found) < PATHS_PER_WAYPOINTS:
            Attempts += 1
            """generate extensions of first part in list of paths
            begin searched by adding nodes to it
            extensions are done by either connecting the path to
            the next waypoint, to nodes creates on obstacles
            or nodes created on the border"""
            Path_extensions = list()

            # Fine the path that is the shortest while adding remaining distance to next waypoint
            def Distance_filter(Path_i):
                return Path_i.Path_distance_to_waypoint(Next_waypoint)
            Best_path = min(Paths_being_searched, key=lambda Path_i: Distance_filter(Path_i))

            # get the last node of first best path
            Last_node = Best_path.Waypoint_list[len(Best_path.Waypoint_list) - 1]

            # check if we can connect the Last_node to the next waypoint
            if Last_node.Is_connectable_to(Next_waypoint, Mission_profile):
                Path_extensions.append(p_o.Path(Best_path.Waypoint_list + [Next_waypoint]))

            # extend path by connecting it to nodes created on obstacles
            for Obstacle_i in Obstacles:
                # create two nodes on each side of each obstacle
                Node_1, Node_2 = Obstacle_i.Create_tangent_nodes(Last_node)
                if Node_1 is not None:
                    if Last_node.Is_connectable_to(Node_1, Mission_profile):
                        Path_extensions.append(p_o.Path(Best_path.Waypoint_list + [Node_1]))
                    if Last_node.Is_connectable_to(Node_2, Mission_profile):
                        Path_extensions.append(p_o.Path(Best_path.Waypoint_list + [Node_2]))

            # extend path by connecting it to nodes created on border
            for Node_i in Border_nodes:
                if Last_node.Is_connectable_to(Node_i, Mission_profile):
                    Path_extensions.append(p_o.Path(Best_path.Waypoint_list + [Node_i]))

            """add extensions of best path that did not reach
            the next waypoint yet to the list of paths
            being searched and the others to list of paths found"""
            for New_path in Path_extensions:
                # get last node of path
                Last_node_of_path = New_path.Waypoint_list[len(New_path.Waypoint_list) - 1]
                # Check if new node is not already in the path
                if not Best_path.Contains_waypoint(Last_node_of_path):
                    # check if path is useful (not bouncing off obstacle/border
                    # vertex or going backwards)
                    Is_useful = True
                    # cannot bounce without 3 nodes
                    if len(New_path.Waypoint_list) > 2:

                        # found before_last_node
                        Before2_last_node = New_path.Waypoint_list[len(New_path.Waypoint_list) - 3]
                        Before_last_node = New_path.Waypoint_list[len(New_path.Waypoint_list) - 2]
                        Last_node = New_path.Waypoint_list[len(New_path.Waypoint_list) - 1]

                        # Check If path is going backwards
                        if Before_last_node.Parent_obstacle is None and Before_last_node.Parent_Vertex is None:
                            if Before_last_node.Is_going_backwards(Before2_last_node, Last_node):
                                Is_useful = False

                        # Check if path is bouncing
                        elif Before_last_node.Is_bouncing(Before2_last_node, Last_node):
                            Is_useful = False

                        # Check if path is going through a node in a straight line
                        elif Before_last_node.Is_across_viable_edge(Before2_last_node, Last_node, Mission_profile):
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
                break

        # add the found paths to the list of mission paths
        Straight_paths_list.append(List_of_paths_found)
    return Straight_paths_list


"""from a list of 2d straight paths, gets a list of 2d alleviated
paths with alleviation waypoints added"""
def Curving_2D(Mission_profile, Straight_2D_paths_list):

    # only keep parts of the mission for which a path was found
    Paths_list = copy.deepcopy(list(filter(lambda x: len(x) > 0, Straight_2D_paths_list)))
    """A curved path can be built by selecting one straight path per waypoint and curving it
    So a curved path from a list of straight paths can be described using a vector with its
    elements being the index of which path of each path group from the straight path list
    was selected to make the curved path, -1 meaning no path chosen"""
    Max_per_group = list(len(Path_group_i) for Path_group_i in Paths_list)
    Curved_paths_found = list()

    """This process stitches paths selected from different path groups, curves them,
    then check if they're valid. Path selection is done by picking a certain number
    of alterations to do to the default shortest path, then choosing which parts of
    the paths to apply them to, then run through all possible alterations in each case"""
    for Variation_num in range(VARIATIONS_FOR_PATH_BUILDING+1):

        # if not enough paths to make N variations, break
        if Variation_num > len(Paths_list):
            break
        # Distribute the variations on the Curved_path_vector
        Distribution = [1]*Variation_num+[0]*(len(Paths_list)-Variation_num)
        while len(Distribution) > 0:
            # run over all possible variations with Variation_num changes
            Path_vector = [0]*(len(Paths_list))
            if sum(Distribution) != 0:
                g_f.Increment_bound_var(Path_vector, Distribution, Max_per_group, Variation_num)

            while len(Path_vector) > 0:
                # Select the paths that were picked
                Path_to_stitch = list()
                for Index, Value in enumerate(Path_vector):
                    Path_to_stitch.append(Paths_list[Index][Value])

                # Stitch them
                W_list = sum(list(Path_i.Waypoint_list[1:] for Path_i in Path_to_stitch), [])
                Stitched_waypoint_list = copy.deepcopy([Path_to_stitch[0].Waypoint_list[0]] + W_list)
                Stitched_path = p_o.Path(Stitched_waypoint_list)

                # Curve, generate off shoot points, and check if path is valid
                Stitched_path.Curve(Mission_profile)
                Stitched_path.Off_shoot()
                if Stitched_path.Is_valid(Mission_profile):
                    Curved_paths_found.append(Stitched_path)
                g_f.Increment_bound_var(Path_vector, Distribution, Max_per_group, Variation_num)
            g_f.Distribution_increment(Distribution, Variation_num)

    return Curved_paths_found


"""from a list of 2d alleviated paths, get 3d alleviated paths with
altitude set for the waypoints missing it"""
def Altitude_extension(Curved_2D_paths_list):
    Curved_2D_paths_list_new = copy.deepcopy(Curved_2D_paths_list)

    """Go over each path in the list, and select all waypoints
    between each two mission waypoints for which altitude is known.
    If the climb between the two waypoints is too steep,
    throw the path away. If not, linearly distribute the climb"""
    Validity_of_paths = [True]*len(Curved_2D_paths_list_new)
    for index, Path_i in enumerate(Curved_2D_paths_list_new):
        W_list = Curved_2D_paths_list_new[index].Waypoint_list
        Selected_waypoints = [W_list[0]]
        for Waypoint_i in W_list:
            if Waypoint_i.z is None:
                Selected_waypoints.append(Waypoint_i)
            else:
                Selected_waypoints.append(Waypoint_i)
                Selected_sub_path = p_o.Path(Selected_waypoints)
                Altitude_change = Waypoint_i.z - Selected_waypoints[0].z
                Distance_traveled = Selected_sub_path.Compute_simple_distance_2d()
                if Distance_traveled != 0:
                    Altitude_ratio = Altitude_change/Distance_traveled
                    if abs(Altitude_ratio) > MAX_CLIMBING_RATIO:
                        Validity_of_paths[index] = False
                        break
                    for index_1, W_i in enumerate(Selected_waypoints):
                        if W_i.Alleviation_waypoint is not None:
                            W_i.Alleviation_waypoint.z = 100
                        if W_i.z is None:
                            P_w = Selected_waypoints[index_1-1]
                            W_i.z = P_w.z + Altitude_ratio * W_i.Distance_2d_to(P_w)

                Selected_waypoints.clear()
                Selected_waypoints.append(Waypoint_i)
    Curved_3D_paths_list = list()
    for index, Valid_i in enumerate(Validity_of_paths):
        if Valid_i:
            Curved_3D_paths_list.append(Curved_2D_paths_list_new[index])
    return Curved_3D_paths_list


"""from a list of 3d alleviated paths, get a final chosen stitched path"""
def Path_selector(Curved_3D_paths_list):
    return min(Curved_3D_paths_list, key=lambda Path_i: Path_i.Compute_simple_distance_2d())
