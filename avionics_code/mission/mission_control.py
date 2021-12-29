from avionics_code.path import path_functions as p_f
from avionics_code.path import path_objects as p_o
from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.utility_functions import geometrical_functions as g_f

import threading
import time

WAYPOINT_ACCEPTANCE_DISTANCE_1 = para.WAYPOINT_ACCEPTANCE_DISTANCE_1
WAYPOINT_ACCEPTANCE_DISTANCE_2 = para.WAYPOINT_ACCEPTANCE_DISTANCE_2
OBSTACLE_DISTANCE_FOR_VALID_PASS = para.OBSTACLE_DISTANCE_FOR_VALID_PASS
MISSION_STATE_REFRESH_RATE = para.MISSION_STATE_REFRESH_RATE
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES

M_S_R_R = MISSION_STATE_REFRESH_RATE

CS = g_v.ControllerStatus


class MissionControl:
    """stores computed path and manipulates it"""
    
    def __init__(self):

        # current path to be sent/that was sent to the plane
        self.chosen_path = None
        self.stripped_path = None
        self.exportable_chosen_path = None

        # computation status of the path
        self.path_computation_status = g_v.StandardStatus.NONE

        # coms variables
        self.lost_con = False
        self.first_connection = True
        self.rf_lost_comms = False

        # controller state
        self.action = CS.WAIT_CONNECTION
        self.waypoint_list_update = False

        # requests
        self.land_requested = False

        self.pre_existing_flight = False

    def launch_controller(self):
        """starts controller"""

        threading.Thread(target=self.plane_controller_loop).start()

    def plane_controller_loop(self):
        """Refreshes mission state by checking if a new
        waypoint was reached, then checks if the plane path
        needs to be recomputed or if time ran out"""

        last_time = time.time()
        while True:
            # keep the refresh rate under or equal
            # to MISSION_STATE_REFRESH_RATE
            new_time = time.time()
            time.sleep(abs(1 / M_S_R_R - (new_time - last_time)))
            last_time = time.time()

            if g_v.close_request or g_v.ms.generation_status == g_v.StandardStatus.FAILED:
                break

            # update mission waypoints if needed
            if self.waypoint_list_update:
                self.update_waypoint_status_list()

            # check if connection is lost
            if g_v.rf.connection_status == 0:
                self.lost_con = True
                self.action = CS.WAIT_CONNECTION
                self.waypoint_list_update = False

            # check if a new connection has been established
            if self.lost_con and g_v.rf.connection_status == 1:
                if self.first_connection:
                    self.first_connection = False
                else:
                    g_v.rf.launch_connect()
                self.lost_con = False
                self.action = CS.CHECK_AIR

            # do not do anything in periods of lost comms
            if g_v.rf.lost_comms:
                self.rf_lost_comms = True
            else:
                # check in if air status
                if self.action == CS.CHECK_AIR:
                    if g_v.th.in_air.data_received():
                        if g_v.th.in_air.data["in air"]:
                            self.action = CS.WAIT_GENERATION_FOR_DOWNLOAD
                            self.pre_existing_flight = True
                        else:
                            self.action = CS.WAIT_GENERATION_FOR_COMPUTE

                # wait for mission generation before computing path
                if self.action == CS.WAIT_GENERATION_FOR_COMPUTE:
                    if g_v.ms.generation_status == g_v.StandardStatus.SUCCESS:
                        self.waypoint_list_update = True
                        self.action = CS.WAIT_POSITION_TELEMETRY

                # wait for mission generation before computing path
                if self.action == CS.WAIT_GENERATION_FOR_DOWNLOAD:
                    if g_v.ms.generation_status == g_v.StandardStatus.SUCCESS:
                        self.action = CS.DOWNLOAD_MISSION

                # download mission
                if self.action == CS.DOWNLOAD_MISSION:
                    g_v.rf.launch_download_mission()
                    self.action = CS.WAIT_DOWNLOAD_MISSION

                # wait for end of mission download or relaunch it
                if self.action == CS.WAIT_DOWNLOAD_MISSION:
                    if g_v.rf.download_status == g_v.StandardStatus.FAILED:
                        g_v.rf.launch_download_mission()
                    elif g_v.rf.download_status == g_v.StandardStatus.SUCCESS:
                        self.action = CS.WAIT_REQUESTS
                        self.land_requested = False
                        self.waypoint_list_update = True

                # wait for position telemetry
                if self.action == CS.WAIT_POSITION_TELEMETRY:
                    if g_v.th.position.data_received() and g_v.th.heading.data_received():
                        self.launch_compute_path()
                        self.action = CS.WAIT_COMPUTE_PATH

                # wait for end of path computation and upload
                if self.action == CS.WAIT_COMPUTE_PATH:
                    if self.path_computation_status == g_v.StandardStatus.SUCCESS:
                        g_v.rf.launch_upload_mission()
                        self.land_requested = False
                        self.action = CS.WAIT_REQUESTS
                    elif self.path_computation_status == g_v.StandardStatus.FAILED:
                        self.action = CS.WAIT_REQUESTS

                # check if any requests were made
                if self.action == CS.WAIT_REQUESTS:
                    if self.rf_lost_comms:
                        self.action = CS.WAIT_GENERATION_FOR_DOWNLOAD
                    elif self.land_requested:
                        g_v.ms.land()
                        self.action = CS.WAIT_GENERATION_FOR_COMPUTE

                self.rf_lost_comms = False

    def land_request(self):
        """ask the controller to make the plane land"""

        if self.action == CS.WAIT_REQUESTS:
            self.land_requested = True
        else:
            self.busy_message()

    @staticmethod
    def busy_message():
        """displays controller is busy in GUI"""

        g_v.gui.display_message("controller is busy", "retry later", 0)

    @staticmethod
    def update_waypoint_status_list():
        """checks if a waypoint from the mission state was reached"""

        if g_v.th.position.data_received():
            plane_obj = g_v.th.position.data["flight object"]
            plane_3d_pos = (plane_obj.pos[0], plane_obj.pos[1], plane_obj.z)
            active_waypoint = g_v.ms.active_waypoint()

            if active_waypoint is not None:
                a_w = active_waypoint
                active_waypoint_3d_pos = (a_w.pos[0], a_w.pos[1], a_w.z)
                # check if the active waypoint was reached
                m_i = a_w.mission_index
                if m_i == 0 or m_i == 1:
                    acceptance_radius = WAYPOINT_ACCEPTANCE_DISTANCE_1
                elif m_i == 2 or m_i == 3:
                    acceptance_radius = WAYPOINT_ACCEPTANCE_DISTANCE_2
                else:
                    acceptance_radius = abs(LANDING_LOITER_COORDINATES["radius"]) * 1.2
                if g_f.distance_3d(plane_3d_pos, active_waypoint_3d_pos) < acceptance_radius:
                    # deactivate waypoint
                    active_waypoint.is_mission = 0
                    g_v.gui.to_draw("mission state")
                    g_v.gui.to_draw("path")

    def launch_compute_path(self):
        """starts path computation if it's not already running, returns thread"""

        # check that path computation is not ongoing
        if self.path_computation_status != g_v.StandardStatus.STARTED:
            new_thread = threading.Thread(target=self.compute_path)
            new_thread.start()
            return new_thread
        else:
            g_v.gui.display_message("Cannot start path computation", "it is started already", 0)
            return None

    def compute_path(self):
        """Computes path to be sent to the plane
        based on the mission state"""

        self.path_computation_status = g_v.StandardStatus.STARTED

        # get info for path computation
        plane_obj = g_v.th.position.data["flight object"]
        heading = g_v.th.heading.data["heading"]
        plane_pos = plane_obj.pos
        plane_z = plane_obj.z

        waypoint_list = g_v.ms.waypoint_list
        profile = g_v.mp

        # check if the plane is inside an obstacle or too close to one
        obstacles = profile.obstacles
        O_D_V = OBSTACLE_DISTANCE_FOR_VALID_PASS
        for obstacle in obstacles:
            # delete obstacle if it is already inside
            if g_f.distance_2d(obstacle.pos, plane_pos) <= obstacle.r:
                profile.delete_obstacle(obstacles.index(obstacle))
                # represent plane as being further away if it is close to it
            elif g_f.distance_2d(obstacle.pos, plane_pos) <= obstacle.r + O_D_V:

                # get vector from obstacle center to current position and scale
                # it till it is at obstacle pass distance
                vec = g_f.sub_vectors(obstacle.pos, plane_pos)
                vec_norm = g_f.norm(vec)
                distance_from_obstacle = obstacle.r + O_D_V
                vec_scaled = g_f.scale_vector(vec, 1 - distance_from_obstacle / vec_norm)

                # get new plane position
                plane_pos = g_f.add_vectors(vec_scaled, plane_pos)

        w_l = waypoint_list
        found_path, max_depth = self.smart_path_finder(heading, plane_pos, plane_z, w_l, profile)

        if not found_path:
            self.path_computation_status = g_v.StandardStatus.FAILED
            if max_depth > -1:
                g_v.gui.display_message("could not make path", f"max depth: {max_depth}", 0)
            else:
                g_v.gui.display_message("no waypoints to fly to", "no path computed", 0)
            return

        self.chosen_path = found_path

        # strip the plane position, takeoff, and landing loiter
        if self.land_requested:
            self.stripped_path = p_o.Path(self.chosen_path.waypoint_list[1:-1])
        else:
            self.stripped_path = p_o.Path(self.chosen_path.waypoint_list[2:-1])

        # reformat the chosen path to be exportable by
        # adding turning points to the path itself
        flattened_waypoint_list = list()
        for way in self.stripped_path.waypoint_list:

            m_i = way.mission_index
            mission_or_airdrop = m_i == 0 or m_i == 1

            # add pre turn waypoints
            pre_turn_waypoint = way.pre_turn_waypoint
            if pre_turn_waypoint is not None and mission_or_airdrop:
                flattened_waypoint_list.append(pre_turn_waypoint)

            flattened_waypoint_list.append(way)

            # add post turn waypoints
            post_turn_waypoint = way.post_turn_waypoint
            if post_turn_waypoint is not None and mission_or_airdrop:
                flattened_waypoint_list.append(post_turn_waypoint)

        self.exportable_chosen_path = p_o.Path(flattened_waypoint_list)

        self.path_computation_status = g_v.StandardStatus.SUCCESS
        g_v.gui.to_draw("system status")
        g_v.gui.to_draw("path")

    @staticmethod
    def smart_path_finder(heading, plane_pos, plane_z, way_list, profile):
        """finds recursively a path through the waypoints while deleting
        points that it could not reach """

        # Check if there are no points to fly to
        if len(way_list) == 0:
            return False, -1

        start_point = p_o.Waypoint(g_v.MissionType.PLANE, plane_pos, plane_z, is_mission=0)

        # add inertia post turn waypoint
        heading_vec = g_f.rotate_vector((PREFERRED_TURN_RADIUS, 0), heading)
        inertia_pos = g_f.add_vectors(plane_pos, heading_vec)
        start_point.post_turn_waypoint = p_o.Waypoint(0, inertia_pos)

        path_to_now = p_o.Path([start_point])

        found_path, max_depth = p_f.recursive_path_search(path_to_now, way_list, profile, 0, way_list)

        # check if the path could be made
        if found_path is None:
            return False, max_depth
        else:
            return found_path, max_depth
