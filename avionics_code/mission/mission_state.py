from avionics_code.path import path_objects as p_o
from avionics_code.path import coverage_finder as c_f
from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.utility_functions import geometrical_functions as g_f
from avionics_code.utility_functions import geography_functions as geo_f

import math
import threading
import time

OBSTACLE_DISTANCE_FOR_VALID_NODE = para.OBSTACLE_DISTANCE_FOR_VALID_NODE
OBSTACLE_DISTANCE_FOR_ORBIT = para.OBSTACLE_DISTANCE_FOR_ORBIT
BORDER_DISTANCE_FOR_VALID_NODE = para.BORDER_DISTANCE_FOR_VALID_NODE
CV_IMAGE_GROUND_SIZE = para.CV_IMAGE_GROUND_SIZE
OFF_AXIS_IMAGING_RANGE = para.OFF_AXIS_IMAGING_RANGE
AIR_DROP_ALTITUDE = para.AIR_DROP_ALTITUDE
IMAGING_ALTITUDE = para.IMAGING_ALTITUDE
OFF_AXIS_IMAGING_ALTITUDE = para.OFF_AXIS_IMAGING_ALTITUDE
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES
LANDING_COORDINATES = para.LANDING_COORDINATES
TAKEOFF_COORDINATES = para.TAKEOFF_COORDINATES
FEET_PER_METER = para.FEET_PER_METER
OFF_AXIS_IMAGING_DISTANCE = para.OFF_AXIS_IMAGING_DISTANCE

FRAMES_PER_SECOND = para.FRAMES_PER_SECOND

SLEEP_THREADS = para.SLEEP_THREADS

MT = g_v.MessageType
SS = g_v.StandardStatus


class MissionState:
    """stores generated true mission and its competition status"""

    def __init__(self):

        # list of waypoints for the whole generated mission
        self.waypoint_list = list()

        # generation status of the mission state
        self.generation_status = SS.NONE

    def launch_generate(self):
        """starts path computation if it's not
        already running, returns thread"""

        # check that path computation is not ongoing
        threading.Thread(target=self.generate).start()

    def generate(self):
        """Generates waypoint list for whole mission including
        mission waypoints, airdrop, scouting, off axis scouting,
        and landing"""

        self.generation_status = SS.STARTED

        gui_sys = g_v.gui.layers["system status"]
        if g_v.rf.upload_status == SS.SUCCESS:
            g_v.rf.upload_status = SS.EXPIRED
        if g_v.mc.path_computation_status == SS.SUCCESS:
            g_v.mc.path_computation_status = SS.EXPIRED
            gui_sys.path_search_percentage = 0

        generated_list = list()
        g_v.gui.to_draw("mission state")

        # add takeoff waypoint
        geo_5 = geo_f.geographic_to_cartesian_center_5
        takeoff_tuple = geo_5(TAKEOFF_COORDINATES)
        take_off_alt = TAKEOFF_COORDINATES["altitude"]
        take_off_way = p_o.Waypoint(g_v.MissionType.TAKEOFF,
                                    takeoff_tuple, take_off_alt,
                                    is_mission=g_v.Activity.ACTIVE)
        
        # check if takeoff is valid
        O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
        B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
        if take_off_way.is_valid(g_v.mp, O_V_N, B_V_N):
            generated_list.append(take_off_way)
        else:
            self.generation_status = SS.FAILED
            g_v.gui.display_message("ERROR, TAKEOFF NOT VALID",
                                    "CHANGE PARAMETERS!!!", MT.ERROR)
            return

        # add mission waypoints
        for index, waypoint in enumerate(g_v.mp.mission_waypoints):
            O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            if waypoint.is_valid(g_v.mp, O_V_N, B_V_N):
                generated_list.append(waypoint)
            else:
                g_v.gui.display_message(f"waypoint {index} unreachable",
                                        "will not attempt", MT.MISSION_STATE)

        # add airdrop
        drop_way = p_o.Waypoint(g_v.MissionType.WAYPOINTS,
                                g_v.mp.airdrop_object.pos,
                                AIR_DROP_ALTITUDE,
                                is_mission=g_v.Activity.ACTIVE)
        O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
        B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
        if drop_way.is_valid(g_v.mp, O_V_N, B_V_N):
            generated_list.append(drop_way)
        else:
            g_v.gui.display_message("airdrop waypoint unreachable",
                                    "will not attempt", MT.MISSION_STATE)
            
        # add scouting of search area
        cover, off_axis_group = self.scout_2area(g_v.mp.search_area,
                                                 g_v.mp.mapping_area,
                                                 generated_list,
                                                 g_v.MissionType.IMAGING)
        generated_list.extend(cover)

        # add off-axis search imaging
        o_a_o = g_v.mp.off_axis_object
        off_axis_grp = self.bulk_off_axis(off_axis_group,
                                          generated_list,
                                          o_a_o, g_v.MissionType.OFF_AXIS)
        generated_list.extend(off_axis_grp)

        # add landing
        loiter_tuple = geo_5(LANDING_LOITER_COORDINATES)
        loiter_altitude = LANDING_LOITER_COORDINATES["altitude"]
        loiter_radius = abs(LANDING_LOITER_COORDINATES["radius"])
        loiter_way = p_o.Waypoint(g_v.MissionType.LANDING,
                                  loiter_tuple, loiter_altitude,
                                  is_mission=g_v.Activity.ACTIVE)

        O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
        B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
        valid_1 = loiter_way.is_valid(g_v.mp, O_V_N, B_V_N)
        valid_2 = p_o.area_is_valid(loiter_tuple, loiter_radius,
                                    g_v.mp, True)
        if valid_1 and valid_2:
            generated_list.append(loiter_way)
        else:
            self.generation_status = SS.FAILED
            g_v.gui.display_message("ERROR, LANDING LOITER NOT VALID",
                                    "CHANGE PARAMETERS!!!", MT.ERROR)
            return

        # check landing point
        landing_tuple = geo_5(LANDING_COORDINATES)
        landing_altitude = LANDING_COORDINATES["altitude"]
        landing_waypoint = p_o.Waypoint(g_v.MissionType.LANDING,
                                        landing_tuple, landing_altitude,
                                        is_mission=g_v.Activity.ACTIVE)

        O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
        B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
        if not landing_waypoint.is_valid(g_v.mp, O_V_N, B_V_N):
            self.generation_status = SS.FAILED
            g_v.gui.display_message("ERROR, LANDING POINT NOT VALID",
                                    "CHANGE PARAMETERS!!!", MT.ERROR)
            return

        self.waypoint_list = generated_list

        self.generation_status = SS.SUCCESS
        g_v.gui.to_draw("mission state")

    def active_index(self):
        """returns index of first active waypoint"""

        active_index = None
        for index, way_i in enumerate(self.waypoint_list):
            if way_i.is_mission == g_v.Activity.ACTIVE:
                active_index = index
                break
        return active_index

    def active_waypoint(self):
        """returns first active waypoint"""

        if self.active_index() is None:
            return None
        else:
            return self.waypoint_list[self.active_index()]

    def land(self):
        """empties mission state waypoint list to only have landing loiter"""

        self.generation_status = SS.STARTED
        g_v.gui.to_draw("mission state")

        # get loiter position for landing add landing
        geo_5 = geo_f.geographic_to_cartesian_center_5
        loiter_tuple = geo_5(LANDING_LOITER_COORDINATES)
        loiter_altitude = LANDING_LOITER_COORDINATES["altitude"]
        loiter_way = p_o.Waypoint(g_v.MissionType.LANDING,
                                  loiter_tuple, loiter_altitude,
                                  is_mission=g_v.Activity.ACTIVE)
        self.waypoint_list = [loiter_way]

        self.generation_status = SS.SUCCESS
        g_v.gui.to_draw("mission state")

    @staticmethod
    def scout_2area(area1, area2, generated_list, mission_type):
        """return list of waypoints that cover a polygonal
        area with squares for picture purposes, and a list of
        positions that are not valid and need to be taken
        care through off-axis imaging"""

        cover = list()
        off_axis_group = list()

        # need at least 3 points to have a true polygon
        if len(area1.vertices) < 3 and len(area2.vertices) < 3:
            return cover

        # consider vertices if they are in group of more than 2
        if len(area1.vertices) > 2:
            vertices1_pos = [vertex.pos for vertex in area1.vertices]
        else:
            vertices1_pos = list()

        if len(area2.vertices) > 2:
            vertices2_pos = [vertex.pos for vertex in area2.vertices]
        else:
            vertices2_pos = list()

        vertices_all_pos = vertices1_pos + vertices2_pos

        # determine square edges of area
        min_area_x = min(pos[0] for pos in vertices_all_pos)
        max_area_x = max(pos[0] for pos in vertices_all_pos)
        min_area_y = min(pos[1] for pos in vertices_all_pos)
        max_area_y = max(pos[1] for pos in vertices_all_pos)
        width = max_area_x - min_area_x
        height = max_area_y - min_area_y

        """Create a grid of cells that cover the area,
        then check if the area covered by each cell covers
        part area, then check if the cell can be accessed
        without going outside the border or entering an
        obstacle"""
        SIZE = CV_IMAGE_GROUND_SIZE
        way_array = list()

        start_time = time.time()

        for row_index in range(math.ceil(height/SIZE)):
            for column_index in range(math.ceil(width/SIZE)):

                if SLEEP_THREADS:
                    # sleep to allow other threads to run
                    if (time.time() - start_time) > 0.4 / FRAMES_PER_SECOND:
                        time.sleep(1 / FRAMES_PER_SECOND)
                    start_time = time.time()

                cell_x = min_area_x + (column_index + 1/2) * SIZE
                cell_y = min_area_y + (row_index + 1/2) * SIZE
                cell = (cell_x, cell_y)

                # get corners of area covered by search point
                corners = g_f.rect_from_cen_size(cell, SIZE, SIZE)
                inside = g_f.point_inside_polygon
                if len(vertices1_pos) != 0:
                    inside1 = [inside(pos, vertices1_pos) for pos in corners]
                else:
                    inside1 = list()
                if len(vertices2_pos) != 0:
                    inside2 = [inside(pos, vertices2_pos) for pos in corners]
                else:
                    inside2 = list()

                """check if the area covers part of the search area"""
                if any(inside1) or any(inside2):
                    O_D_O = OBSTACLE_DISTANCE_FOR_ORBIT
                    B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
                    act = g_v.Activity.ACTIVE
                    cell_waypoint = p_o.Waypoint(mission_type, cell,
                                                 IMAGING_ALTITUDE,
                                                 is_mission=act)
                    if cell_waypoint.is_valid(g_v.mp, O_D_O, B_V_N):
                        way_array.append(cell_waypoint)
                    else:
                        off_axis_group.append(cell)
                else:
                    intersection1 = False
                    # check if the edges of the covered
                    # area intersect with search area
                    for index in range(len(vertices1_pos)):
                        vertex_1 = vertices1_pos[index]
                        vertex_2 = vertices1_pos[(index+1) % len(vertices1_pos)]
                        for index_2 in range(len(corners)):
                            corner_1 = corners[index_2]
                            corner_2 = corners[(index_2 + 1) % len(corners)]
                            inter = g_f.seg_to_seg_intersection
                            if inter(vertex_1, vertex_2, corner_1, corner_2):
                                intersection1 = True
                                break
                        if intersection1:
                            break

                    intersection2 = False
                    # check if the edges of the covered area
                    # intersect with search area
                    for index in range(len(vertices2_pos)):
                        vertex_1 = vertices2_pos[index]
                        vertex_2 = vertices2_pos[(index + 1) % len(vertices2_pos)]
                        for index_2 in range(len(corners)):
                            corner_1 = corners[index_2]
                            corner_2 = corners[(index_2 + 1) % len(corners)]
                            inter = g_f.seg_to_seg_intersection
                            if inter(vertex_1, vertex_2, corner_1, corner_2):
                                intersection2 = True
                                break
                        if intersection2:
                            break

                    if intersection1 or intersection2:
                        O_D_O = OBSTACLE_DISTANCE_FOR_ORBIT
                        B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
                        act = g_v.Activity.ACTIVE
                        cell_waypoint = p_o.Waypoint(mission_type, cell,
                                                     IMAGING_ALTITUDE,
                                                     is_mission=act)
                        if cell_waypoint.is_valid(g_v.mp, O_D_O, B_V_N):
                            way_array.append(cell_waypoint)
                        else:
                            off_axis_group.append(cell)

        # last waypoint of the plane
        if len(generated_list) > 0:
            last_pos = generated_list[-1].pos
        else:
            last_pos = None

        # use the coverage finder to find a path
        # to get the list of cells to go through
        cover = c_f.cover(way_array, last_pos)

        start_time = time.time()

        # simplify the cover
        if len(cover) > 4:
            cover_kept = [True, True]
            for index in range(2, len(cover)-2):

                if SLEEP_THREADS:
                    # sleep to allow other threads to run
                    if (time.time() - start_time) > 0.4 / FRAMES_PER_SECOND:
                        time.sleep(1 / FRAMES_PER_SECOND)
                    start_time = time.time()

                way_1 = cover[index-2]
                way_2 = cover[index-1]
                way_3 = cover[index]
                way_4 = cover[index+1]
                way_5 = cover[index+2]

                # check that the waypoints are aligned
                allign_1 = way_2.is_across_viable_edge(way_1, way_3, g_v.mp)
                allign_2 = way_3.is_across_viable_edge(way_2, way_4, g_v.mp)
                allign_3 = way_4.is_across_viable_edge(way_3, way_5, g_v.mp)

                if allign_1 and allign_2 and allign_3:
                    cover_kept.append(False)
                else:
                    cover_kept.append(True)
            cover_kept.extend([True, True])

            # takeoff unnecessary waypoints
            cover = list(filter(lambda x: cover_kept[cover.index(x)], cover))

        return cover, off_axis_group

    @staticmethod
    def border_off_axis_imaging(mission_type, pos, inside=False):
        """returns a waypoint on the edge
        of the border to allow for off-axis imagine"""

        profile = g_v.mp
        vertices = profile.border.vertices

        # find vectors to the closest point inside the polygon for each edge
        new_waypoints = list()
        for index, vertex in enumerate(vertices):
            # get edge of border
            cur_vertex = vertices[index].pos
            next_vertex = vertices[(index+1) % len(vertices)].pos
            # get the closest point on edge to pos
            proj = g_f.point_to_seg_projection(pos, cur_vertex,
                                               next_vertex)

            # get vector from pos to proj and scale it,
            # so it is at off axis imaging distance
            vec = g_f.sub_vectors(proj, pos)
            vec_norm = g_f.norm(vec)
            O_A_I_D = OFF_AXIS_IMAGING_DISTANCE
            if not inside:
                vec_scaled = g_f.scale_vector(vec, 1 + O_A_I_D / vec_norm)
                image_distance = g_f.norm(vec_scaled)
            else:
                vec_scaled = g_f.scale_vector(vec, 1 - O_A_I_D / vec_norm)
                image_distance = g_f.norm(vec_scaled) - CV_IMAGE_GROUND_SIZE

            # get new point for off axis imaging
            closest_point = g_f.add_vectors(vec_scaled, pos)
            O_A = OFF_AXIS_IMAGING_ALTITUDE
            new_way = p_o.Waypoint(mission_type, closest_point, O_A,
                                   is_mission=g_v.Activity.ACTIVE,
                                   target=pos)
            O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            if new_way.is_valid(g_v.mp, O_V_N, B_V_N):
                new_waypoints.append((new_way, image_distance))

        # get the closest valid off axis imaging point
        if len(new_waypoints) > 0:
            new_waypoint_tuple = min(new_waypoints, key=lambda x: x[1])
        else:
            return None

        # check if point is close enough for off axis imaging
        if new_waypoint_tuple[1] <= OFF_AXIS_IMAGING_RANGE:
            return new_waypoint_tuple[0]
        else:
            g_v.gui.display_message("off-axis object unreachable",
                                    "will not attempt", MT.MISSION_STATE)
            return None

    def bulk_off_axis(self, off_axis_group, generated_list,
                      off_axis_obj, mission_type):
        """returns trajectory for taking off-axis images
        for a group of points"""

        # add off-axis object imaging
        off_axis_way = []
        off_axis_obj_way = self.border_off_axis_imaging(mission_type,
                                                        off_axis_obj.pos)
        if off_axis_obj_way is not None:
            off_axis_way.append(off_axis_obj_way)
        else:
            g_v.gui.display_message("off-axis object unreachable",
                                    "will not attempt", MT.MISSION_STATE)

        # split them into inside border and outside border
        outside_border = list()
        inside_border = list()
        for pos_i in off_axis_group:
            waypoint_i = p_o.Waypoint(mission_type, pos_i)
            if g_v.mp.border.waypoint_in_area(waypoint_i):
                inside_border.append(pos_i)
            else:
                outside_border.append(pos_i)

        # do border off axis imaging for those outside the border
        outside_border_new = list()
        for pos_i in outside_border:
            new_way = self.border_off_axis_imaging(mission_type, pos_i)
            if new_way is not None:
                outside_border_new.append(new_way)

        # split inside border into close to border and close to obstacle
        close_to_border = list()
        close_to_obstacle = list()
        for pos_i in inside_border:
            waypoint_i = p_o.Waypoint(mission_type, pos_i)
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            if g_v.mp.border.waypoint_is_too_close(waypoint_i, B_V_N):
                close_to_border.append(pos_i)
            else:
                close_to_obstacle.append(pos_i)

        # do border off axis imaging for those inside the border
        inside_border_new_1 = list()
        for pos_i in close_to_border:
            new_way = self.border_off_axis_imaging(mission_type,
                                                   pos_i, inside=True)
            if new_way is not None:
                inside_border_new_1.append(new_way)

        # find obstacles that the points are too close to and do off-axis
        inside_border_new_2 = list()
        for pos_i in close_to_obstacle:

            # find which obstacle the waypoint is too close to
            obstacle = None
            for obstacle_i in g_v.mp.obstacles:
                # check if the seg connecting the two waypoints
                # intersects with the obstacle
                center = obstacle_i.pos
                safe_radius = obstacle_i.r + OBSTACLE_DISTANCE_FOR_ORBIT
                if g_f.point_inside_circle(pos_i, center, safe_radius):
                    obstacle = obstacle_i
                    break

            o_center = obstacle.pos
            o_radius = obstacle.r

            # get vector from obstacle center to current position and scale
            # it till it is at off axis imaging distance
            vec = g_f.sub_vectors(o_center, pos_i)
            vec_norm = g_f.norm(vec)
            dis_from_obs = o_radius + OFF_AXIS_IMAGING_DISTANCE
            vec_scaled = g_f.scale_vector(vec, 1 - dis_from_obs / vec_norm)

            # get off axis imaging point
            image_distance = g_f.norm(vec_scaled) - CV_IMAGE_GROUND_SIZE
            closest_point = g_f.add_vectors(vec_scaled, pos_i)
            O_A = OFF_AXIS_IMAGING_ALTITUDE
            new_way = p_o.Waypoint(mission_type, closest_point, O_A,
                                   is_mission=g_v.Activity.ACTIVE,
                                   target=pos_i)

            # check that the point is valid
            # and close enough for off axis imaging
            O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            A = new_way.is_valid(g_v.mp, O_V_N, B_V_N)
            B = image_distance <= OFF_AXIS_IMAGING_RANGE
            if A and B:
                inside_border_new_2.append(new_way)

        inside_border = inside_border_new_1 + inside_border_new_2
        new_positions = off_axis_way + outside_border_new + inside_border

        # last waypoint of the plane
        if len(generated_list) > 0:
            last_pos = generated_list[-1].pos
        elif not g_v.th.is_empty():
            last_pos = g_v.th.last_flight_profile().last_flight_profile.pos
        else:
            last_pos = None

        # use the coverage finder to find a path
        # to get the list of new positions
        cover = c_f.cover(new_positions, last_pos, off_axis=True)

        return cover
