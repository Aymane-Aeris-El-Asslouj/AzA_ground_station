import Path_functions


# from a set of 3d waypoints to go through and a map, returns a set of 3d waypoints that avoid obstacles
def Final_path_finder(Start_position, Mission_waypoint_list, Obstacles, Border):
    if len(Mission_waypoint_list) == 0:
        return list()

    # from a set of 2d waypoints, returns a list of 2d straight paths that connect them while avoiding obstacles
    # this list contains, for each two waypoints given, a list of up to 10 paths connecting those 2 waypoints
    Straight_path_list = Path_functions.Straight_path_finder(Start_position, Mission_waypoint_list, Obstacles, Border)

    # from a list of 2d straight paths, gets a list of 2d alleviated paths with alleviation waypoints added
    Alleviated_path_list = Path_functions.Alleviator(Straight_path_list, Obstacles, Border)

    # from a list of 2d alleviated paths, get 3d alleviated paths with altitude set for the waypoints missing it
    Full_path_list = Path_functions.Altitude_analysis(Alleviated_path_list)

    # from a list of 3d alleviated paths, get a final chosen stitched path
    Final_path = Path_functions.Path_selector(Full_path_list)

    return Final_path
