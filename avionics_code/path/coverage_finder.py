from avionics_code.helpers import geometrical_functions as g_f, parameters as para
from avionics_code.path import path_functions as p_f, path_objects as p_o

WEIGHT_OF_ANGLE = para.WEIGHT_OF_ANGLE
BIAS_OF_ANGLE = para.BIAS_OF_ANGLE
WEIGHT_OF_DISTANCE = para.WEIGHT_OF_DISTANCE
BIAS_OF_DISTANCE = para.BIAS_OF_DISTANCE


def cover(cell_array, plane_pos, profile):
    """Finds a path that covers all cells of an array"""

    path = list()

    if len(cell_array) == 0:
        return path

    # start with an edge
    if plane_pos is not None:
        last_pos_2 = plane_pos
        edge_cells = list()
        min_x = min(cell[0] for cell in cell_array)
        max_x = max(cell[0] for cell in cell_array)
        min_y = min(cell[1] for cell in cell_array)
        max_y = max(cell[1] for cell in cell_array)

        def check_edge(x):
            f_q = g_f.float_eq
            return f_q(x[0], min_x) or f_q(x[0], max_x) or f_q(x[1], min_y) or f_q(x[1], max_y)

        edge_cells.extend(filter(check_edge, cell_array))
        last_pos = min(edge_cells, key=lambda x: weighted_distance(x, plane_pos, profile))

    else:
        last_pos = min(cell_array, key=lambda x: x[0])
        last_pos_2 = last_pos

    # add closest cell to list of cells to travel then delete it from array
    for index in range(len(cell_array)):
        next_cell = min(cell_array, key=lambda x: cell_cost(last_pos_2, last_pos, x, profile))
        path.append(next_cell)
        del cell_array[cell_array.index(next_cell)]
        last_pos_2 = last_pos
        last_pos = next_cell

    return path

def cell_cost(cell_0, cell_1, cell_2, profile):
    """Finds the 'cost' of going into a cell
    considering the two previous cells"""

    a = WEIGHT_OF_ANGLE
    b = BIAS_OF_ANGLE
    c = WEIGHT_OF_DISTANCE
    d = BIAS_OF_DISTANCE

    cell_0 = cell_0
    cell_1 = cell_1
    cell_2 = cell_2

    distance_to_cell = weighted_distance(cell_1, cell_2, profile)

    cell_0_to_1 = g_f.sub_vectors(cell_1, cell_0)
    cell_1_to_2 = g_f.sub_vectors(cell_2, cell_1)

    if g_f.float_eq_2d(cell_0_to_1, (0, 0)) or g_f.float_eq_2d(cell_1_to_2, (0, 0)):
        angle_change = 0
    else:
        angle_change = g_f.find_angle(cell_0_to_1, cell_1_to_2)

    return (a * abs(angle_change) + c) * (b * distance_to_cell + d)

def weighted_distance(pos_1, pos_2, profile):
    """Gives true path distance between two
    waypoints while accounting for dodging obstacles"""

    distance = g_f.distance_2d(pos_1, pos_2)

    way_1 = p_o.Waypoint(pos_1)
    way_2 = p_o.Waypoint(pos_2)

    path = p_f.single_path(way_1, way_2, profile)
    if path is None:
        return float('inf')
    path.distance_2d_update()

    return path.simple_distance_2d
