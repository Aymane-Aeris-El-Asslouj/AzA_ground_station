from avionics_code.path import path_objects as p_o
from avionics_code.references import parameters as para
from avionics_code.references import global_variables as g_v
from avionics_code.utility_functions import geometrical_functions as g_f

import time

PATHS_PER_WAYPOINTS = para.PATHS_PER_WAYPOINTS
MAX_ATTEMPTS_PER_WAYPOINTS = para.MAX_ATTEMPTS_PER_WAYPOINTS
BACK_TRACKING_DEPTH = para.BACK_TRACKING_DEPTH
FRAMES_PER_SECOND = para.FRAMES_PER_SECOND

SLEEP_THREADS = para.SLEEP_THREADS


def recursive_path_search(path_to_now, waypoint_list,
                          profile, depth, full_list):
    """extends a path, so it can reach the first waypoint
    of a waypoint list then calls itself to continue the
    search till the waypoint list is empty. At that point,
    it returns the found path. the path is curved and checked
    in altitude to make sure it is valid, if not, switch to
    another found path or return that the previously found
    path needs to be rebuilt"""

    if SLEEP_THREADS:
        # sleep to allow other threads to run
        if (time.time() - g_v.mc.start_time) > 0.4 / FRAMES_PER_SECOND:
            time.sleep(1 / FRAMES_PER_SECOND)
        g_v.mc.start_time = time.time()

    completion = 100 * (1 - len(waypoint_list)/len(full_list))
    g_v.gui.layers["system status"].path_search_percentage = completion

    # max depth reached in the search
    local_depth = depth
    # if all waypoints were reached, return the found path
    if len(waypoint_list) == 0:
        return path_to_now, local_depth
    else:
        # get the last waypoint of the found path
        last_node = path_to_now.waypoint_list[-1]

        # find n path that connect it to
        # the next waypoint in the waypoint list
        next_paths = single_path_search(last_node, waypoint_list[0],
                                        profile, PATHS_PER_WAYPOINTS)

        # go through all paths found and explore their future paths
        for picked_next_path in next_paths:

            # merge the previous path and new selected path
            new_path_to_now = p_o.Path(path_to_now.waypoint_list
                                       + picked_next_path.waypoint_list[1:])

            # calls itself and get the path found and
            # max depth that it reached
            r_search = recursive_path_search
            r_1, r_2 = r_search(new_path_to_now, waypoint_list[1:],
                                profile, depth+1, full_list)
            explore_attempt = r_1
            max_depth = r_2

            # check if the limits of back tracing were breached
            if explore_attempt is None:
                if max_depth - local_depth > BACK_TRACKING_DEPTH:
                    return None, max(local_depth, max_depth)

            # update local depth
            local_depth = max(local_depth, max_depth)

            # if there was no problem, return the found path
            if explore_attempt is not None:
                return explore_attempt, local_depth

        return None, local_depth


def single_path_search(way_1, way_2, profile, path_number):
    """finds n path between two waypoints while avoiding obstacles
    and staying in the border. This is done by graph exploration
    where a set of paths created made out of waypoints. At each round,
    the closest path is extended by creating waypoints tangent
    to obstacles or the border till the paths reach the next waypoints.
    The paths are also curved and set in altitude"""

    obstacles = profile.obstacles
    border = profile.border

    mission_type = way_2.mission_type

    # search from the post turn waypoint if there is one
    if way_1.post_turn_waypoint is not None:
        starting_node = way_1.post_turn_waypoint
    else:
        starting_node = way_1

    # nodes creatable on border
    border_nodes = border.border_waypoints

    # loop through each waypoint in the list except the last
    # one, and try to connect it to the following waypoint
    num_paths_found = 0
    paths_being_searched = [p_o.Path([starting_node])]

    # avoid endless search
    attempts = 0
    # keep looking for paths till running out of paths to
    # explore or find 10 paths connecting the two waypoints
    while len(paths_being_searched) > 0 and num_paths_found < path_number:
        attempts += 1
        """generate extensions of first part in list of paths
        begin searched by adding nodes to it
        extensions are done by either connecting the path to
        the next waypoint, to nodes creates on obstacles
        or nodes created on the border"""
        path_extensions = list()

        # Fine the path that is the shortest while adding
        # remaining distance to next waypoint
        def distance_filter(path_i):
            return path_i.path_distance_to_waypoint(way_2)

        best_path = min(paths_being_searched,
                        key=lambda path_i: distance_filter(path_i))

        # get the last node of first best path
        last_node = best_path.waypoint_list[- 1]

        # check if we can connect the last_node to the next waypoint
        if last_node.is_connectable_to(way_2, profile):
            new_path = p_o.Path(best_path.waypoint_list + [way_2])
            path_extensions.append(new_path)

        # extend path by connecting it to nodes created on obstacles
        for obs_i in obstacles:
            # create two nodes on each side of each obstacle
            tangent = obs_i.create_tangent_nodes
            node_1, node_2 = tangent(last_node, profile, mission_type)
            if node_1 is not None:
                if last_node.is_connectable_to(node_1, profile):
                    new_path = p_o.Path(best_path.waypoint_list + [node_1])
                    path_extensions.append(new_path)
            if node_2 is not None:
                if last_node.is_connectable_to(node_2, profile):
                    new_path = p_o.Path(best_path.waypoint_list + [node_2])
                    path_extensions.append(new_path)

        # extend path by connecting it to nodes created on border
        for node_i in border_nodes:
            if last_node.is_connectable_to(node_i, profile):
                node_new = p_o.Waypoint(mission_type, node_i.pos,
                                        parent_vertex=node_i.parent_vertex)
                new_path = p_o.Path(best_path.waypoint_list + [node_new])
                path_extensions.append(new_path)

        """add extensions of best path that did not reach
        the next waypoint yet to the list of paths
        being searched and the others to list of paths found"""
        for new_path in path_extensions:
            # get last node of path
            last_node_of_path = new_path.waypoint_list[-1]
            # Check if new node is not already in the path
            if not g_f.float_eq_2d(last_node_of_path.pos,
                                   new_path.waypoint_list[-2].pos):
                # check if path is useful (not bouncing off obstacle/border
                # vertex or going backwards)
                is_useful = True
                # cannot bounce without 3 nodes
                if len(new_path.waypoint_list) > 2:

                    # found before_last_node
                    before2_last_node = new_path.waypoint_list[-3]
                    before_last_node = new_path.waypoint_list[-2]
                    last_node = new_path.waypoint_list[-1]

                    b_node = before_last_node
                    b2_node = before2_last_node

                    # Check If path is going backwards
                    if b_node.parent_obstacle is None:
                        if b_node.parent_vertex is None:
                            if b_node.is_going_backwards(b2_node, last_node):
                                is_useful = False

                    # Check if path is bouncing
                    elif b_node.is_bouncing(b2_node, last_node):
                        is_useful = False

                    # Check if path is going through a node in a straight line
                    elif b_node.is_across_viable_edge(b2_node, last_node, profile):
                        is_useful = False

                if is_useful:
                    # check if last node of new-found path is the next waypoint
                    if last_node_of_path is way_2:
                        # replace the initial post turn waypoint by way 1
                        found_path = p_o.Path([way_1] + new_path.waypoint_list[1:])

                        # check if curving and altitude setting of path works
                        curving_attempt = found_path.curve(profile)
                        altitude_attempt = found_path.altitude_set()

                        if curving_attempt and altitude_attempt:
                            yield found_path
                            num_paths_found += 1
                            attempts = 0
                    else:
                        paths_being_searched.append(new_path)

        # delete the path that we're done exploring
        paths_being_searched.remove(best_path)

        # If too many attempts return no path found
        if attempts >= MAX_ATTEMPTS_PER_WAYPOINTS:
            break
