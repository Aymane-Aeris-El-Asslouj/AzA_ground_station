from avionics_code.helpers import geometrical_functions as g_f, parameters as para

WEIGHT_OF_ANGLE = para.WEIGHT_OF_ANGLE
BIAS_OF_ANGLE = para.BIAS_OF_ANGLE
WEIGHT_OF_DISTANCE = para.WEIGHT_OF_DISTANCE
BIAS_OF_DISTANCE = para.BIAS_OF_DISTANCE
OFF_AXIS_WEIGHT_OF_ANGLE = para.OFF_AXIS_WEIGHT_OF_ANGLE


def cover(waypoint_array, plane_pos, off_axis=False):
    """Finds a path that covers all waypoints in list on a map"""

    path = list()
    cell_array = [way.pos for way in waypoint_array]
    if len(waypoint_array) == 0:
        return path

    # start with an edge
    if plane_pos is not None:
        last_pos_2 = plane_pos
        edge_cells = list()
        min_x = min(cell.pos[0] for cell in waypoint_array)
        max_x = max(cell.pos[0] for cell in waypoint_array)
        min_y = min(cell.pos[1] for cell in waypoint_array)
        max_y = max(cell.pos[1] for cell in waypoint_array)

        def check_edge(x):
            f_q = g_f.float_eq
            return f_q(x[0], min_x) or f_q(x[0], max_x) or f_q(x[1], min_y) or f_q(x[1], max_y)

        edge_cells.extend(filter(check_edge, cell_array))
        last_pos = min(edge_cells, key=lambda x: g_f.distance_2d(x, plane_pos))

    else:
        last_pos = min(cell_array, key=lambda x: x[0])
        last_pos_2 = last_pos

    # add closest cell to list of cells to travel then delete it from array
    for index in range(len(waypoint_array)):
        next_cell = min(waypoint_array, key=lambda x: cell_cost(last_pos_2, last_pos, x.pos, off_axis))
        path.append(next_cell)
        del waypoint_array[waypoint_array.index(next_cell)]
        last_pos_2 = last_pos
        last_pos = next_cell.pos

    return path


def cell_cost(cell_0, cell_1, cell_2, off_axis):
    """Finds the 'cost' of going into a cell
    considering the two previous cells"""

    if not off_axis:
        a = WEIGHT_OF_ANGLE
    else:
        a = OFF_AXIS_WEIGHT_OF_ANGLE
    b = BIAS_OF_ANGLE
    c = WEIGHT_OF_DISTANCE
    d = BIAS_OF_DISTANCE

    cell_0 = cell_0
    cell_1 = cell_1
    cell_2 = cell_2

    distance_to_cell = g_f.distance_2d(cell_1, cell_2)

    cell_0_to_1 = g_f.sub_vectors(cell_1, cell_0)
    cell_1_to_2 = g_f.sub_vectors(cell_2, cell_1)

    if g_f.float_eq_2d(cell_0_to_1, (0, 0)) or g_f.float_eq_2d(cell_1_to_2, (0, 0)):
        angle_change = 0
    else:
        angle_change = g_f.find_angle(cell_0_to_1, cell_1_to_2)

    return (a * abs(angle_change) + c) * (b * distance_to_cell + d)
