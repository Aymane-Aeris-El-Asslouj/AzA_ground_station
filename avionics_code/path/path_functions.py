from avionics_code.path import path_objects as p_o
from avionics_code.helpers import parameters as para, algebraic_functions as a_f
from avionics_code.helpers import geometrical_functions as g_f

import copy

PATHS_PER_WAYPOINTS = para.PATHS_PER_WAYPOINTS
MAX_ATTEMPTS_PER_WAYPOINTS = para.MAX_ATTEMPTS_PER_WAYPOINTS
VARIATIONS_FOR_PATH_BUILDING = para.VARIATIONS_FOR_PATH_BUILDING
MAX_CLIMBING_RATIO = para.MAX_CLIMBING_RATIO


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
        last_node = best_path.waypoint_list[ - 1]

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
                node_new = copy.deepcopy(node_i)
                node_new.mission_index = last_node.mission_index
                path_extensions.append(p_o.Path(best_path.waypoint_list + [node_new]))

        """add extensions of best path that did not reach
        the next waypoint yet to the list of paths
        being searched and the others to list of paths found"""
        for new_path in path_extensions:
            # get last node of path
            last_node_of_path = new_path.waypoint_list[-1]
            # Check if new node is not already in the path
            if not g_f.float_eq_2d(last_node_of_path.pos, new_path.waypoint_list[-2].pos):
                # check if path is useful (not bouncing off obstacle/border
                # vertex or going backwards)
                is_useful = True
                # cannot bounce without 3 nodes
                if len(new_path.waypoint_list) > 2:

                    # found before_last_node
                    before2_last_node = new_path.waypoint_list[ - 3]
                    before_last_node = new_path.waypoint_list[- 2]
                    last_node = new_path.waypoint_list[- 1]

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
                    if last_node_of_path is way_2:
                        return new_path
                    else:
                        paths_being_searched.append(new_path)

        # delete the path that we're done exploring
        paths_being_searched.remove(best_path)

        # If too many attempts return no path found
        if attempts >= MAX_ATTEMPTS_PER_WAYPOINTS:
            return None

def straight_2d_path_finder(plane_obj, waypoint_list, profile):
    """from a set of 2d waypoints, returns a list of 2d
    straight paths that connect them while avoiding obstacles
    this list contains, for each two waypoints given,
    a list of up to 10 paths connecting those 2 waypoints"""

    obstacles = profile.obstacles
    border = profile.border

    """create the total number of nodes to connect by adding the
    starting position as a node to the mission waypoints"""

    if len(waypoint_list) > 0:
        mission_index = waypoint_list[0].mission_index
    else:
        mission_index = 0

    start_point = p_o.Waypoint(mission_index, plane_obj.pos, plane_obj.z)
    true_waypoint_list = [start_point] + waypoint_list

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
            last_node = best_path.waypoint_list[- 1]

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
                    node_new = copy.deepcopy(node_i)
                    node_new.mission_index = last_node.mission_index
                    path_extensions.append(p_o.Path(best_path.waypoint_list + [node_new]))

            """add extensions of best path that did not reach
            the next waypoint yet to the list of paths
            being searched and the others to list of paths found"""
            for new_path in path_extensions:
                # get last node of path
                last_node_of_path = new_path.waypoint_list[-1]
                # Check if new node is not already in the path
                if not g_f.float_eq_2d(last_node_of_path.pos, new_path.waypoint_list[-2].pos):
                    # check if path is useful (not bouncing off obstacle/border
                    # vertex or going backwards)
                    is_useful = True
                    # cannot bounce without 3 nodes
                    if len(new_path.waypoint_list) > 2:

                        # found before_last_node
                        before2_last_node = new_path.waypoint_list[- 3]
                        before_last_node = new_path.waypoint_list[- 2]
                        last_node = new_path.waypoint_list[- 1]

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
                        if last_node_of_path is next_waypoint:
                            list_of_paths_found.append(new_path)
                        else:
                            paths_being_searched.append(new_path)

            # delete the path that we're done exploring
            paths_being_searched.remove(best_path)

            # If too many attempts return no path found
            if attempts >= MAX_ATTEMPTS_PER_WAYPOINTS:
                break

        # add the found paths to the list of mission paths
        if len(list_of_paths_found) > 0:
            straight_paths_list.append(list_of_paths_found)
        else:
            straight_paths_list.append(None)
    return straight_paths_list


"""from a list of 2d straight paths, gets a list of 2d alleviated
paths with alleviation waypoints added"""
def curving_2d(straight_2D_paths_list, profile):

    # only keep parts of the mission for which a path was found
    paths_list = copy.deepcopy(list(filter(lambda x: len(x) > 0, straight_2D_paths_list)))
    """A curved path can be built by selecting one straight path per waypoint and curving it
    So a curved path from a list of straight paths can be described using a vector with its
    elements being the index of which path of each path group from the straight path list
    was selected to make the curved path, -1 meaning no path chosen"""
    max_per_group = list(len(path_group_i) for path_group_i in paths_list)
    curved_2d_paths_list = list()

    """This process stitches paths selected from different path groups, curves them,
    then check if they're valid. path selection is done by picking a certain number
    of alterations to do to the default shortest path, then choosing which parts of
    the paths to apply them to, then run through all possible alterations in each case"""
    for variation_num in range(VARIATIONS_FOR_PATH_BUILDING+1):

        # if not enough paths to make N variations, break
        if variation_num > len(paths_list):
            break
        # distribute the variations on the curved_path_vector
        distribution = [1]*variation_num+[0]*(len(paths_list)-variation_num)
        while len(distribution) > 0:
            # run over all possible variations with variation_num changes
            path_vector = [0]*(len(paths_list))
            if sum(distribution) != 0:
                a_f.increment_bound_var(path_vector, distribution, max_per_group, variation_num)

            while len(path_vector) > 0:
                # Select the paths that were picked
                path_to_stitch = list()
                for index, value in enumerate(path_vector):
                    path_to_stitch.append(paths_list[index][value])

                # stitch them
                w_list = sum(list(path_i.waypoint_list[1:] for path_i in path_to_stitch), [])
                stitched_waypoint_list = copy.deepcopy([path_to_stitch[0].waypoint_list[0]] + w_list)
                stitched_path = p_o.Path(stitched_waypoint_list)

                # curve, generate off shoot points, and check if path is valid
                stitched_path.curve(profile)
                stitched_path.off_shoot()
                ####if stitched_path.is_valid(profile):
                curved_2d_paths_list.append(stitched_path)
                a_f.increment_bound_var(path_vector, distribution, max_per_group, variation_num)
            ####a_f.distribution_increment(distribution, variation_num)
            #### following break should not be here
            break

    return curved_2d_paths_list


"""from a list of 2d alleviated paths, get 3d alleviated paths with
altitude set for the waypoints missing it"""
def Altitude_extension(curved_2D_paths_list):
    curved_2D_paths_list_new = copy.deepcopy(curved_2D_paths_list)

    """Go over each path in the list, and select all waypoints
    between each two mission waypoints for which altitude is known.
    If the climb between the two waypoints is too steep,
    throw the path away. If not, linearly distribute the climb"""
    validity_of_paths = [True]*len(curved_2D_paths_list_new)
    for index, path_i in enumerate(curved_2D_paths_list_new):
        w_list = curved_2D_paths_list_new[index].waypoint_list
        Selected_waypoints = [w_list[0]]
        for waypoint_i in w_list:
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
                        validity_of_paths[index] = False
                        break
                    for index_1, w_i in enumerate(Selected_waypoints):
                        if w_i.Alleviation_waypoint is not None:
                            w_i.Alleviation_waypoint.z = 100
                        if w_i.z is None:
                            P_w = Selected_waypoints[index_1-1]
                            w_i.z = P_w.z + Altitude_ratio * w_i.distance_2d_to(P_w)

                Selected_waypoints.clear()
                Selected_waypoints.append(waypoint_i)
    curved_3D_paths_list = list()
    for index, valid_i in enumerate(validity_of_paths):
        if valid_i:
            curved_3D_paths_list.append(curved_2D_paths_list_new[index])
    return curved_3D_paths_list


"""from a list of 3d alleviated paths, get a final chosen stitched path"""
def path_selector(curved_3D_paths_list):
    return min(curved_3D_paths_list, key=lambda path_i: path_i.Compute_simple_distance_2d())
