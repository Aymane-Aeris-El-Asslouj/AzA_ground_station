import math

from avionics_code.path import path_objects as p_o, coverage_finder as c_f
from avionics_code.helpers import global_variables as g_v, parameters as para, geometrical_functions as g_f

CV_IMAGE_GROUND_SIZE = para.CV_IMAGE_GROUND_SIZE
OFF_AXIS_IMAGING_DISTANCE = para.OFF_AXIS_IMAGING_DISTANCE
OFF_AXIS_IMAGING_SEPARATION = para.OFF_AXIS_IMAGING_SEPARATION
OFF_AXIS_IMAGING_RANGE = para.OFF_AXIS_IMAGING_RANGE

class MissionState:
    """stores generated true mission and its competition status"""

    def __init__(self):

        # list of waypoints for the whole generated mission
        self.waypoint_list = list()

        # stores the indices of separation between parts
        # of the mission in the waypoint list
        self.separator = list()

    def generate(self):
        """Generates waypoint list for whole mission including
        mission waypoints, airdrop, object search, mapping,
        off axis object, and landing"""

        print("\nGenerating mission state...")
        generated_list = list()
        self.separator.clear()

        # add mission waypoints
        for index, waypoint in enumerate(g_v.mp.mission_waypoints):
            if waypoint.is_valid(g_v.mp):
                generated_list.append(waypoint)
            else:
                print(f'Warning: waypoint {index} is not valid, will not attempt')
        if len(generated_list) == 0:
            print("Warning: Could not make trajectory for waypoints, will not attempt")
        self.separator.append(len(generated_list))

        # add airdrop
        airdrop_waypoint = p_o.Waypoint(g_v.mp.airdrop_object.pos)
        if airdrop_waypoint.is_valid(g_v.mp):
            generated_list.append(airdrop_waypoint)
        else:
            print("Warning: airdrop position is not valid, airdrop will not attempt")
        self.separator.append(len(generated_list))

        # add scouting of search area
        cover, off_axis_group_1 = self.scout_area(g_v.mp.search_area, generated_list)
        if len(cover) > 0:
            generated_list.extend(cover)
        else:
            print("Warning: Could not make trajectory for searching, will not attempt")
        self.separator.append(len(generated_list))

        # add scouting of mapping area
        cover, off_axis_group_2 = self.scout_area(g_v.mp.mapping_area, generated_list)
        if len(cover) > 0:
            generated_list.extend(cover)
        else:
            print("Warning: Could not make trajectory for mapping, will not attempt")
        self.separator.append(len(generated_list))

        # add off-axis object imaging
        off_axis_object_trajectory = self.border_off_axis_imaging(g_v.mp.off_axis_object)
        if len(off_axis_object_trajectory) > 0:
            generated_list.extend(off_axis_object_trajectory)
        else:
            print("Warning: Could not make trajectory for off-axis object, will not attempt")
        self.separator.append(len(generated_list))

        # add off-axis search imaging
        off_axis_trajectory = self.bulk_off_axis(off_axis_group_1)
        generated_list.extend(off_axis_trajectory)
        self.separator.append(len(generated_list))

        # add off-axis mapping imaging
        off_axis_trajectory = self.bulk_off_axis(off_axis_group_2)
        generated_list.extend(off_axis_trajectory)
        self.separator.append(len(generated_list))

        # add landing
        generated_list.extend([])
        self.separator.append(len(generated_list))

        self.waypoint_list = generated_list

    @staticmethod
    def scout_area(area, generated_list):
        """return list of waypoints that cover an polygonal
        area with squares for picture purposes, and a list of
        positions that are not valid and need to be taken
        care through off-axis imaging"""

        cover = list()
        off_axis_group = list()

        # need at least 3 points to have a true polygon
        if len(area.vertices) < 3:
            return cover, off_axis_group

        vertices_pos = [vertex.pos for vertex in area.vertices]

        # determine square edges of area
        min_area_x = min(pos[0] for pos in vertices_pos)
        max_area_x = max(pos[0] for pos in vertices_pos)
        min_area_y = min(pos[1] for pos in vertices_pos)
        max_area_y = max(pos[1] for pos in vertices_pos)
        width = max_area_x - min_area_x
        height = max_area_y - min_area_y

        """Create a grid of cells that cover the area,
        then check if the area covered by each cell covers
        part area, then check if the cell can be accessed
        without going outside the border or entering an
        obstacle"""
        SIZE = CV_IMAGE_GROUND_SIZE
        cell_array = list()
        pairs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
        for row_index in range(math.ceil(height/SIZE)):
            for column_index in range(math.ceil(width/SIZE)):
                cell_x = min_area_x + (column_index + 1/2) * SIZE
                cell_y = min_area_y + (row_index + 1/2) * SIZE
                cell = (cell_x, cell_y)

                # get corners of area covered by search point
                S = SIZE / 2
                corners = [(cell_x + pair[0] * S, cell_y + pair[1] * S) for pair in pairs]
                inside = [g_f.point_inside_polygon(pos, vertices_pos) for pos in corners]

                """check if the area covers part of the search area"""
                if any(inside):
                    if p_o.Waypoint(cell).is_valid(g_v.mp):
                        cell_array.append((cell_x, cell_y))
                    else:
                        off_axis_group.append(p_o.Waypoint((cell_x, cell_y)))
                else:
                    intersection = False

                    for index in range(len(vertices_pos)):
                        vertex_1 = vertices_pos[index]
                        vertex_2 = vertices_pos[(index+1) % len(vertices_pos)]
                        for index_2 in range(len(corners)):
                            corner_1 = corners[index_2]
                            corner_2 = corners[(index_2 + 1) % len(corners)]
                            inter = g_f.seg_to_seg_intersection
                            if inter(vertex_1, vertex_2, corner_1, corner_2):
                                intersection = True
                                break
                        if intersection:
                            break

                    if intersection:
                        if p_o.Waypoint(cell).is_valid(g_v.mp):
                            cell_array.append((cell_x, cell_y))
                        else:
                            off_axis_group.append(p_o.Waypoint((cell_x, cell_y)))

        # last waypoint of the plane
        if len(generated_list) > 0:
            last_pos = generated_list[len(generated_list)-1].pos
        else:
            last_pos = None
        # use the coverage finder to find a path
        # to get the list of cells to go through
        cover = c_f.cover(cell_array, last_pos, g_v.mp)
        cover = [p_o.Waypoint(pos) for pos in cover]

        return cover, off_axis_group

    @staticmethod
    def border_off_axis_imaging(map_object, inside=False):
        """returns a pair of waypoints on the edge
        of the border to allow for off-axis imagine"""

        pos = map_object.pos
        profile = g_v.mp
        vertices = profile.border.vertices

        # find vectors to the closest point inside the polygon for each edge
        waypoint_pairs = list()
        for index, vertex in enumerate(vertices):
            cur_vertex_pos = vertices[index].pos
            next_vertex_pos = vertices[(index+1) % len(vertices)].pos

            proj = g_f.point_to_seg_projection(pos, cur_vertex_pos, next_vertex_pos)

            vec = g_f.sub_vectors(proj, pos)
            vec_norm = g_f.norm(vec)
            if not inside:
                vec_scaled = g_f.scale_vector(vec, 1 + OFF_AXIS_IMAGING_DISTANCE / vec_norm)
            else:
                vec_scaled = g_f.scale_vector(vec, 1 - OFF_AXIS_IMAGING_DISTANCE / vec_norm)

            closest_point = g_f.add_vectors(vec_scaled, pos)

            n1, n2 = g_f.unit_normal_vectors_to_line(pos, closest_point)

            n1_scaled = g_f.scale_vector(n1, OFF_AXIS_IMAGING_SEPARATION)
            n2_scaled = g_f.scale_vector(n2, OFF_AXIS_IMAGING_SEPARATION)

            way_1 = p_o.Waypoint(g_f.add_vectors(closest_point, n1_scaled))
            way_2 = p_o.Waypoint(g_f.add_vectors(closest_point, n2_scaled))

            if way_1.is_connectable_to(way_2, g_v.mp):
                if way_1.is_valid(g_v.mp) and way_2.is_valid(g_v.mp):
                    waypoint_pairs.append((way_1, way_2, vec_norm))

        if len(waypoint_pairs) > 0:
            waypoint_pair = min(waypoint_pairs, key=lambda x: x[2])
        else:
            return []

        if waypoint_pair[2] <= OFF_AXIS_IMAGING_RANGE:
            return [waypoint_pair[0], waypoint_pair[1]]
        else:
            return []

    def bulk_off_axis(self, off_axis_group):
        """returns trajectory for taking off-axis images
        for a group of points"""

        # split them into inside border and outside border
        outside_border = list()
        inside_border = list()
        for way in off_axis_group:
            if g_v.mp.border.waypoint_in_area(way):
                inside_border.append(way)
            else:
                outside_border.append(way)

        # do border off axis imaging for those outside the border
        outside_border_pairs = [self.border_off_axis_imaging(way) for way in outside_border]

        # split inside border into close to border and close to obstacle
        close_to_border = list()
        close_to_obstacle = list()
        for way in inside_border:
            if g_v.mp.border.waypoint_is_too_close(way):
                close_to_border.append(way)
            else:
                close_to_obstacle.append(way)

        # do border off axis imaging for those outside the border
        inside_border_pairs_1 = [self.border_off_axis_imaging(way, inside=True) for way in outside_border]

        full_pairs = outside_border_pairs + inside_border_pairs_1

        return sum(full_pairs, [])
