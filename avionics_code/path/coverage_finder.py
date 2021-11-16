from avionics_code.helpers import geometrical_functions as g_f, parameters as para
from avionics_code.path import path_functions as p_f, path_objects as p_o

WEIGHT_OF_ANGLE = para.WEIGHT_OF_ANGLE
BIAS_OF_ANGLE = para.BIAS_OF_ANGLE
WEIGHT_OF_DISTANCE = para.WEIGHT_OF_DISTANCE
BIAS_OF_DISTANCE = para.BIAS_OF_DISTANCE
OFF_AXIS_WEIGHT_OF_ANGLE = para.OFF_AXIS_WEIGHT_OF_ANGLE


def cover(cell_array, plane_pos, profile, off_axis=False):
    """Finds a path that covers all positions on a map"""

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
        last_pos = min(edge_cells, key=lambda x: g_f.distance_2d(x, plane_pos))

    else:
        last_pos = min(cell_array, key=lambda x: x[0])
        last_pos_2 = last_pos

    # add closest cell to list of cells to travel then delete it from array
    for index in range(len(cell_array)):
        next_cell = min(cell_array, key=lambda x: cell_cost(last_pos_2, last_pos, x, profile, off_axis))
        path.append(next_cell)
        del cell_array[cell_array.index(next_cell)]
        last_pos_2 = last_pos
        last_pos = next_cell

    return path

def cell_cost(cell_0, cell_1, cell_2, profile, off_axis):
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