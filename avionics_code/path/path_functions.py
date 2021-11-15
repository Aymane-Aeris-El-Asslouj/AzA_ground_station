from avionics_code.path import path_objects as p_o
from avionics_code.helpers import parameters as para, geometrical_functions as g_f

import copy

PATHS_PER_WAYPOINTS = para.PATHS_PER_WAYPOINTS
MAX_ATTEMPTS_PER_WAYPOINTS = para.MAX_ATTEMPTS_PER_WAYPOINTS
VARIATIONS_FOR_PATH_BUILDING = para.VARIATIONS_FOR_PATH_BUILDING
MAX_CLIMBING_RATIO = para.MAX_CLIMBING_RATIO


def Compute_path(self):
    if len(self.waypoint_list) > 0:
        self.straight_2D_paths_list = p_f.straight_2D_path_finder(self)
        self.Curved_2D_paths_list = p_f.Curving_2D(self, self.straight_2D_paths_list)
        self.Curved_3D_paths_list = p_f.Altitude_extension(self.Curved_2D_paths_list)
        self.Chosen_3D_path = p_f.path_selector(self.Curved_3D_paths_list)
    else:
        self.straight_2D_paths_list = list()
        self.Curved_2D_paths_list = list()
        self.Curved_3D_paths_list = list()
        self.Chosen_3D_path = list()


def single_path(way_1, way_2, profile):
    """find path between two waypoints"""

    obstacles = profile.obstacles
    border = profile.border

    # nodes creatable on border
    border_nodes = border.border_waypoints

    # loop through each waypoint in the list except the last
    # one, and try to connect it to the following waypoint
    paths_being_searched = [p_o.Path([way_1])]

    # avoid endless search
    attempts = 0
    # keep looking for paths till running out of paths to
    # explore or find 10 paths connecting the two waypoints
    while len(paths_being_searched) > 0:
        attempts += 1
        """generate extensions of first part in list of paths
        begin searched by adding nodes to it
        extensions are done by either connecting the path to
        the next waypoint, to nodes creates on obstacles
        or nodes created on the border"""
        path_extensions = list()

        # Fine the path that is the shortest while adding remaining distance to next waypoint
        def distance_filter(path_i):
            return path_i.path_distance_to_waypoint(way_2)

        best_path = min(paths_being_searched, key=lambda path_i: distance_filter(path_i))

        # get the last node of first best path
        last_node = best_path.waypoint_list[len(best_path.waypoint_list) - 1]

        # check if we can connect the last_node to the next waypoint
        if last_node.is_connectable_to(way_2, profile):
            return p_o.Path(best_path.waypoint_list + [way_2])

        # extend path by connecting it to nodes created on obstacles
        for obstacle_i in obstacles:
            # create two nodes on each side of each obstacle
            node_1, node_2 = obstacle_i.create_tangent_nodes(last_node)
            if node_1 is not None:
                if last_node.is_connectable_to(node_1, profile):
                    path_extensions.append(p_o.Path(best_path.waypoint_list + [node_1]))
                if last_node.is_connectable_to(node_2, profile):
                    path_extensions.append(p_o.Path(best_path.waypoint_list + [node_2]))

        # extend path by connecting it to nodes created on border
        for node_i in border_nodes:
            if last_node.is_connectable_to(node_i, profile):
                path_extensions.append(p_o.Path(best_path.waypoint_list + [node_i]))

        """add extensions of best path that did not reach
        the next waypoint yet to the list of paths
        being searched and the others to list of paths found"""
        for new_path in path_extensions:
            # get last node of path
            last_node_of_path = new_path.waypoint_list[len(new_path.waypoint_list) - 1]
            # Check if new node is not already in the path
            if not best_path.contains_waypoint(last_node_of_path):
                # check if path is useful (not bouncing off obstacle/border
                # vertex or going backwards)
                is_useful = True
                # cannot bounce without 3 nodes
                if len(new_path.waypoint_list) > 2:

                    # found before_last_node
                    before2_last_node = new_path.waypoint_list[len(new_path.waypoint_list) - 3]
                    before_last_node = new_path.waypoint_list[len(new_path.waypoint_list) - 2]
                    last_node = new_path.waypoint_list[len(new_path.waypoint_list) - 1]

                    # Check If path is going backwards
                    if before_last_node.parent_obstacle is None and before_last_node.parent_vertex is None:
                        if before_last_node.is_going_backwards(before2_last_node, last_node):
                            is_useful = False

                    # Check if path is bouncing
                    elif before_last_node.is_bouncing(before2_last_node, last_node):
                        is_useful = False

                    # Check if path is going through a node in a straight line
                    elif before_last_node.is_across_viable_edge(before2_last_node, last_node, profile):
                        is_useful = False

                if is_useful:
                    # check if last node of new found path is the next waypoint
                    if last_node_of_path.coincides_with(way_2):
                        return new_path
                    else:
                        paths_being_searched.append(new_path)

        # delete the path that we're done exploring
        paths_being_searched.remove(best_path)

        # If too many attempts return no path found
        if attempts >= MAX_ATTEMPTS_PER_WAYPOINTS:
            return None

def straight_2d_path_finder(start_position, waypoint_list, profile):
    """from a set of 2d waypoints, returns a list of 2d
    straight paths that connect them while avoiding obstacles
    this list contains, for each two waypoints given,
    a list of up to 10 paths connecting those 2 waypoints"""

    obstacles = profile.obstacles
    border = profile.border

    """create the total number of nodes to connect by adding the
    starting position as a node to the mission waypoints"""
    true_waypoint_list = [start_position] + waypoint_list

    """Each two waypoints from the true waypoint list will be connected by up to PATHS_PER_WAYPOINTS straight line
    paths. These paths get grouped into a list that corresponds to each two waypoints, a path group. Then these
    path groups get grouped into the "straight_paths_list" list for the entire mission."""
    straight_paths_list = list()

    # nodes creatable on border
    border_nodes = border.border_waypoints

    # loop through each waypoint in the list except the last
    # one, and try to connect it to the following waypoint
    for waypoint_index in range(len(true_waypoint_list) - 1):
        list_of_paths_found = list()
        current_waypoint = true_waypoint_list[waypoint_index]
        next_waypoint = true_waypoint_list[waypoint_index + 1]
        paths_being_searched = [p_o.Path([current_waypoint])]

        # avoid endless search
        attempts = 0
        # keep looking for paths till running out of paths to
        # explore or find 10 paths connecting the two waypoints
        while len(paths_being_searched) > 0 and len(list_of_paths_found) < PATHS_PER_WAYPOINTS:
            attempts += 1
            """generate extensions of first part in list of paths
            begin searched by adding nodes to it
            extensions are done by either connecting the path to
            the next waypoint, to nodes creates on obstacles
            or nodes created on the border"""
            path_extensions = list()

            # Fine the path that is the shortest while adding remaining distance to next waypoint
            def distance_filter(path_i):
                return path_i.path_distance_to_waypoint(next_waypoint)
            
            best_path = min(paths_being_searched, key=lambda path_i: distance_filter(path_i))

            # get the last node of first best path
            last_node = best_path.waypoint_list[len(best_path.waypoint_list) - 1]

            # check if we can connect the last_node to the next waypoint
            if last_node.is_connectable_to(next_waypoint, profile):
                path_extensions.append(p_o.Path(best_path.waypoint_list + [next_waypoint]))

            # extend path by connecting it to nodes created on obstacles
            for obstacle_i in obstacles:
                # create two nodes on each side of each obstacle
                node_1, node_2 = obstacle_i.create_tangent_nodes(last_node)
                if node_1 is not None:
                    if last_node.is_connectable_to(node_1, profile):
                        path_extensions.append(p_o.Path(best_path.waypoint_list + [node_1]))
                    if last_node.is_connectable_to(node_2, profile):
                        path_extensions.append(p_o.Path(best_path.waypoint_list + [node_2]))

            # extend path by connecting it to nodes created on border
            for node_i in border_nodes:
                if last_node.is_connectable_to(node_i, profile):
                    path_extensions.append(p_o.Path(best_path.waypoint_list + [node_i]))

            """add extensions of best path that did not reach
            the next waypoint yet to the list of paths
            being searched and the others to list of paths found"""
            for new_path in path_extensions:
                # get last node of path
                last_node_of_path = new_path.waypoint_list[len(new_path.waypoint_list) - 1]
                # Check if new node is not already in the path
                if not best_path.contains_waypoint(last_node_of_path):
                    # check if path is useful (not bouncing off obstacle/border
                    # vertex or going backwards)
                    is_useful = True
                    # cannot bounce without 3 nodes
                    if len(new_path.waypoint_list) > 2:

                        # found before_last_node
                        before2_last_node = new_path.waypoint_list[len(new_path.waypoint_list) - 3]
                        before_last_node = new_path.waypoint_list[len(new_path.waypoint_list) - 2]
                        last_node = new_path.waypoint_list[len(new_path.waypoint_list) - 1]

                        # Check If path is going backwards
                        if before_last_node.parent_obstacle is None and before_last_node.parent_vertex is None:
                            if before_last_node.is_going_backwards(before2_last_node, last_node):
                                is_useful = False

                        # Check if path is bouncing
                        elif before_last_node.is_bouncing(before2_last_node, last_node):
                            is_useful = False

                        # Check if path is going through a node in a straight line
                        elif before_last_node.is_across_viable_edge(before2_last_node, last_node, profile):
                            is_useful = False

                    if is_useful:
                        # check if last node of new found path is the next waypoint
                        if last_node_of_path.coincides_with(next_waypoint):
                            list_of_paths_found.append(new_path)
                        else:
                            paths_being_searched.append(new_path)

            # delete the path that we're done exploring
            paths_being_searched.remove(best_path)

            # If too many attempts return no path found
            if attempts >= MAX_ATTEMPTS_PER_WAYPOINTS:
                break

        # add the found paths to the list of mission paths
        straight_paths_list.append(list_of_paths_found)
    return straight_paths_list


"""from a list of 2d straight paths, gets a list of 2d alleviated
paths with alleviation waypoints added"""
def Curving_2D(profile, straight_2D_paths_list):

    # only keep parts of the mission for which a path was found
    paths_list = copy.deepcopy(list(filter(lambda x: len(x) > 0, straight_2D_paths_list)))
    """A curved path can be built by selecting one straight path per waypoint and curving it
    So a curved path from a list of straight paths can be described using a vector with its
    elements being the index of which path of each path group from the straight path list
    was selected to make the curved path, -1 meaning no path chosen"""
    Max_per_group = list(len(path_group_i) for path_group_i in paths_list)
    Curved_paths_found = list()

    """This process stitches paths selected from different path groups, curves them,
    then check if they're valid. path selection is done by picking a certain number
    of alterations to do to the default shortest path, then choosing which parts of
    the paths to apply them to, then run through all possible alterations in each case"""
    for Variation_num in range(VARIATIONS_FOR_PATH_BUILDING+1):

        # if not enough paths to make N variations, break
        if Variation_num > len(paths_list):
            break
        # Distribute the variations on the Curved_path_vector
        Distribution = [1]*Variation_num+[0]*(len(paths_list)-Variation_num)
        while len(Distribution) > 0:
            # run over all possible variations with Variation_num changes
            path_vector = [0]*(len(paths_list))
            if sum(Distribution) != 0:
                g_f.Increment_bound_var(path_vector, Distribution, Max_per_group, Variation_num)

            while len(path_vector) > 0:
                # Select the paths that were picked
                path_to_stitch = list()
                for Index, Value in enumerate(path_vector):
                    path_to_stitch.append(paths_list[Index][Value])

                # Stitch them
                W_list = sum(list(path_i.waypoint_list[1:] for path_i in path_to_stitch), [])
                Stitched_waypoint_list = copy.deepcopy([path_to_stitch[0].waypoint_list[0]] + W_list)
                Stitched_path = p_o.Path(Stitched_waypoint_list)

                # Curve, generate off shoot points, and check if path is valid
                Stitched_path.Curve(profile)
                Stitched_path.Off_shoot()
                if Stitched_path.is_valid(profile):
                    Curved_paths_found.append(Stitched_path)
                g_f.Increment_bound_var(path_vector, Distribution, Max_per_group, Variation_num)
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
    for index, path_i in enumerate(Curved_2D_paths_list_new):
        W_list = Curved_2D_paths_list_new[index].waypoint_list
        Selected_waypoints = [W_list[0]]
        for waypoint_i in W_list:
            if waypoint_i.z is None:
                Selected_waypoints.append(waypoint_i)
            else:
                Selected_waypoints.append(waypoint_i)
                Selected_sub_path = p_o.Path(Selected_waypoints)
                Altitude_change = waypoint_i.z - Selected_waypoints[0].z
                distance_traveled = Selected_sub_path.Compute_simple_distance_2d()
                if distance_traveled != 0:
                    Altitude_ratio = Altitude_change/distance_traveled
                    if abs(Altitude_ratio) > MAX_CLIMBING_RATIO:
                        Validity_of_paths[index] = False
                        break
                    for index_1, W_i in enumerate(Selected_waypoints):
                        if W_i.Alleviation_waypoint is not None:
                            W_i.Alleviation_waypoint.z = 100
                        if W_i.z is None:
                            P_w = Selected_waypoints[index_1-1]
                            W_i.z = P_w.z + Altitude_ratio * W_i.distance_2d_to(P_w)

                Selected_waypoints.clear()
                Selected_waypoints.append(waypoint_i)
    Curved_3D_paths_list = list()
    for index, Valid_i in enumerate(Validity_of_paths):
        if Valid_i:
            Curved_3D_paths_list.append(Curved_2D_paths_list_new[index])
    return Curved_3D_paths_list


"""from a list of 3d alleviated paths, get a final chosen stitched path"""
def path_selector(Curved_3D_paths_list):
    return min(Curved_3D_paths_list, key=lambda path_i: path_i.Compute_simple_distance_2d())
