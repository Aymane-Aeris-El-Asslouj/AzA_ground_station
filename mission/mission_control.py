from path import path_functions as p_f
from path import path_objects as p_o
from references import global_variables as g_v
from references import parameters as para
from utility_functions import geometrical_functions as g_f

import threading
import time

WAYPOINT_ACCEPTANCE_DISTANCE_1 = para.WAYPOINT_ACCEPTANCE_DISTANCE_1
WAYPOINT_ACCEPTANCE_DISTANCE_2 = para.WAYPOINT_ACCEPTANCE_DISTANCE_2
OBSTACLE_DISTANCE_FOR_VALID_PASS = para.OBSTACLE_DISTANCE_FOR_VALID_PASS
MISSION_STATE_REFRESH_RATE = para.MISSION_STATE_REFRESH_RATE
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES

ORBIT_RADIUS_VALIDATION = para.ORBIT_RADIUS_VALIDATION

M_S_R_R = MISSION_STATE_REFRESH_RATE

CS = g_v.ControllerStatus
SS = g_v.StandardStatus
MT = g_v.MessageType
MIT = g_v.MissionType


class MissionControl:
    """stores computed path and manipulates it"""
    
    def __init__(self):

        # current path to be sent/that was sent to the plane
        self.chosen_path = None
        self.stripped_path = None
        self.exportable_chosen_path = None

        # computation status of the path
        self.path_computation_status = SS.NONE
        self.start_time = None

        # controller state
        self.waypoint_list_update = False
        self.queue = []

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

            # update mission waypoints if needed
            if self.waypoint_list_update:
                self.update_waypoint_status_list()

            if len(self.queue) > 0:
                state = self.queue[0]
            else:
                state = None

            # if connection is lost, reset queue, stop waypoint
            # list updates, and wait for connection
            if (g_v.rf.connection_status != SS.SUCCESS
                    and state != CS.AWAITING_CONNECTION):

                gui_sys = g_v.gui.layers["system status"]
                if self.path_computation_status == SS.SUCCESS:
                    self.path_computation_status = SS.EXPIRED
                    gui_sys.path_search_percentage = 0
                if g_v.rf.upload_status == SS.SUCCESS:
                    g_v.rf.upload_status = SS.EXPIRED

                self.queue.insert(0, CS.AWAITING_CONNECTION)
                self.waypoint_list_update = False
                
            # wait for connection to check air status
            if state == CS.AWAITING_CONNECTION:
                if g_v.get_success(g_v.rf, "connection_status"):
                    self.queue[0] = CS.CHECKING_AIR

            # when in air status received, either download mission
            # or start path computation
            elif state == CS.CHECKING_AIR:
                if g_v.th.in_air.data_received():
                    # if in air, download the current mission from it
                    if g_v.th.in_air.data["in air"]:
                        g_v.rf.upload_status = SS.SUCCESS
                        g_v.rf.mission_start = SS.SUCCESS

                        # do a generation if not already done
                        if g_v.ms.generation_status != SS.SUCCESS:
                            self.queue[0] = CS.GENERATE_MISSION
                            self.queue.insert(1, CS.DOWNLOAD_MISSION)
                        else:
                            self.queue[0] = CS.DOWNLOAD_MISSION
                    # otherwise, compute path from current mission
                    else:
                        self.waypoint_list_update = True
                        self.queue.pop(0)

            # start downloading mission when possible
            elif state == CS.DOWNLOAD_MISSION:

                # check that RF comms is free
                if g_v.rf.free_rf():
                    g_v.rf.launch_download_mission()
                    self.queue[0] = CS.DOWNLOADING_MISSION

            # wait download mission from plane
            elif state == CS.DOWNLOADING_MISSION:
                down_status = g_v.rf.download_status

                # keep relaunching download if it fails
                if down_status == SS.FAILED:
                    g_v.rf.launch_download_mission()
                # switch to waiting for requests when done
                elif down_status == SS.SUCCESS:
                    self.waypoint_list_update = True
                    self.queue.pop(0)

            # start generating state when possible
            elif state == CS.GENERATE_MISSION:

                if g_v.ms.generation_status != SS.STARTED:
                    g_v.ms.launch_generate()
                    self.queue[0] = CS.GENERATING_MISSION

            # start generating state when possible
            elif state == CS.GENERATING_MISSION:
                status = g_v.ms.generation_status

                # if failed stop queue
                if status == SS.FAILED or status == SS.SUCCESS:
                    self.queue.pop(0)

            # start computing path when possible
            elif state == CS.COMPUTE_PATH:

                # wait for previous computations to end
                if g_v.ms.generation_status == SS.SUCCESS:
                    if self.path_computation_status != SS.STARTED:
                        self.launch_compute_path()
                        self.queue[0] = CS.COMPUTING_PATH
                else:
                    self.queue.pop(0)

            # wait for end of path computation and upload
            elif state == CS.COMPUTING_PATH:
                status = self.path_computation_status

                # if failed stop queue
                if status == SS.FAILED or status == SS.SUCCESS:
                    self.queue.pop(0)

            # upload mission when possible
            elif state == CS.UPLOAD_MISSION:

                # only start upload with a successful path
                if self.path_computation_status == SS.SUCCESS:
                    # wait for data to be received
                    if g_v.rf.free_rf() and g_v.th.in_air.data_received():
                        g_v.rf.launch_upload_mission()
                        self.queue[0] = CS.UPLOADING_MISSION
                else:
                    self.queue.pop(0)

            # wait for end of path computation and upload
            elif state == CS.UPLOADING_MISSION:
                upload_status = g_v.rf.upload_status

                # if failed redo it
                if upload_status == SS.FAILED:
                    g_v.rf.launch_upload_mission()
                # if succeeds, wait for requests
                elif upload_status == SS.SUCCESS:
                    self.queue.pop(0)

            # start mission when possible
            elif state == CS.START_MISSION:

                # only start upload with a successful path
                if g_v.rf.upload_status == SS.SUCCESS:
                    # wait for data to be received
                    if g_v.rf.free_rf() and g_v.th.armed.data_received():
                        g_v.rf.launch_start_mission()
                        self.queue[0] = CS.STARTING_MISSION
                else:
                    self.queue.pop(0)

            # wait for end of path computation and upload
            elif state == CS.STARTING_MISSION:
                start_status = g_v.rf.mission_start

                # if failed redo it
                if start_status == SS.FAILED:
                    g_v.rf.launch_start_mission()
                # if succeeds, wait for requests
                elif start_status == SS.SUCCESS:
                    self.queue.pop(0)

            # start mission when possible
            elif state == CS.PAUSE_MISSION:

                # only start upload with a successful path
                if g_v.rf.free_rf() and g_v.th.flight_mode.data_received():

                    # wait for data to be received
                    if g_v.th.flight_mode.data["flight mode"].value == 4:
                        g_v.rf.launch_pause_mission()
                        self.queue[0] = CS.PAUSING_MISSION
                    else:
                        self.queue.pop(0)

            # wait for end of path computation and upload
            elif state == CS.PAUSING_MISSION:
                pause_status = g_v.rf.mission_pause

                # if failed redo it
                if pause_status == SS.FAILED:
                    g_v.rf.launch_pause_mission()
                # if succeeds, wait for requests
                elif pause_status == SS.SUCCESS:
                    self.queue.pop(0)

            # start mission when possible
            elif state == CS.GO_TO:

                if g_v.rf.free_rf() and g_v.th.in_air.data_received():

                    # wait for data to be received
                    if g_v.th.in_air.data["in air"]:
                        g_v.rf.launch_go_to(*self.queue[1])
                        self.queue[0] = CS.GO_TOING
                    else:
                        self.queue.pop(0)
                        self.queue.pop(0)

            # wait for end of path computation and upload
            elif state == CS.GO_TOING:
                go_to_status = g_v.rf.go_to_status

                # if failed redo it
                if go_to_status == SS.FAILED:
                    g_v.rf.launch_go_to(*self.queue[1])
                # if succeeds, wait for requests
                elif go_to_status == SS.SUCCESS:

                    # wait for telemetry to validate go to
                    if g_v.th.position.data_received():

                        flight_obj = g_v.th.position.data["flight object"]
                        pos = flight_obj.pos
                        plane_pos = (pos[0], pos[1], flight_obj.z)

                        go_to_tuple = self.queue[1]
                        pos = go_to_tuple[0]
                        go_to_pos = (pos[0], pos[1], go_to_tuple[1])

                        dis = g_f.distance_3d(plane_pos, go_to_pos)
                        if dis < ORBIT_RADIUS_VALIDATION:
                            self.queue.pop(0)
                            self.queue.pop(0)

            # request plane to land
            elif state == CS.LAND:

                # get landing mission state and request
                # path computation then upload
                g_v.ms.land()
                self.queue[0] = CS.COMPUTE_PATH
                self.queue.insert(1, CS.UPLOAD_MISSION)

            # start mission when possible
            elif state == CS.CAMERA_GIMBAL_COMMAND:

                # only start upload with a successful path
                if g_v.rf.free_rf():
                    g_v.rf.launch_camera_gimbal()
                    self.queue[0] = CS.CAMERA_GIMBAL_COMMANDING

            # wait for end of path computation and upload
            elif state == CS.CAMERA_GIMBAL_COMMANDING:
                camera_gimbal_status = g_v.rf.camera_gimbal

                # if failed redo it
                if camera_gimbal_status == SS.FAILED:
                    g_v.rf.launch_camera_gimbal()
                # if succeeds, wait for requests
                elif camera_gimbal_status == SS.SUCCESS:
                    self.queue.pop(0)

    @staticmethod
    def busy_message():
        """displays controller is busy in GUI"""

        g_v.gui.display_message("controller is busy",
                                "retry later", MT.CONTROLLER)

    @staticmethod
    def update_waypoint_status_list():
        """checks if a waypoint from the mission state was reached"""

        # get plane telemetry
        if g_v.th.position.data_received():
            plane_obj = g_v.th.position.data["flight object"]
            pos = plane_obj.pos
            plane_3d_pos = (pos[0], pos[1], plane_obj.z)
            active_waypoint = g_v.ms.active_waypoint()

            if active_waypoint is not None:
                a_w = active_waypoint
                active_way_3d = (a_w.pos[0], a_w.pos[1], a_w.z)
                # check if the active waypoint was reached
                m_i = a_w.mission_type
                if m_i == MIT.WAYPOINTS:
                    acc_rad = WAYPOINT_ACCEPTANCE_DISTANCE_1
                elif m_i == MIT.IMAGING or m_i == MIT.OFF_AXIS:
                    acc_rad = WAYPOINT_ACCEPTANCE_DISTANCE_2
                else:
                    L_L_C = LANDING_LOITER_COORDINATES
                    acc_rad = abs(L_L_C["radius"]) * 1.2
                if g_f.distance_3d(plane_3d_pos, active_way_3d) < acc_rad:
                    # deactivate waypoint
                    active_waypoint.is_mission = g_v.Activity.INACTIVE
                    g_v.gui.to_draw("mission state")
                    g_v.gui.to_draw("path")

    def launch_compute_path(self):
        """starts path computation"""

        # check that path computation is not ongoing
        threading.Thread(target=self.compute_path).start()

    def compute_path(self):
        """Computes path to be sent to the plane
        based on the mission state"""

        self.path_computation_status = SS.STARTED
        if g_v.rf.upload_status == SS.SUCCESS:
            g_v.rf.upload_status = SS.EXPIRED
        g_v.gui.to_draw("path")

        # get info for path computation
        plane_obj = g_v.th.position.data["flight object"]
        heading = g_v.th.heading.data["heading"]
        plane_pos = plane_obj.pos
        plane_z = plane_obj.z

        waypoint_list = g_v.ms.waypoint_list
        profile = g_v.mp

        # check if the plane is inside
        # an obstacle or too close to one
        obstacles = profile.obstacles
        O_D_V = OBSTACLE_DISTANCE_FOR_VALID_PASS
        dis = g_f.distance_2d
        for obs in obstacles:
            # delete obstacle if it is already inside
            if dis(obs.pos, plane_pos) <= obs.r:
                profile.delete_obstacle(obstacles.index(obs))
                # represent plane as being further away
                # if it is close to it
            elif dis(obs.pos, plane_pos) <= obs.r + O_D_V:

                # get vector from obstacle center to current
                # position and scale it till it is at obstacle
                # pass distance
                vec = g_f.sub_vectors(obs.pos, plane_pos)
                vec_norm = g_f.norm(vec)
                dis_from_obs = obs.r + O_D_V
                scale = g_f.scale_vector
                vec_scaled = scale(vec, 1 - dis_from_obs / vec_norm)

                # get new plane position
                plane_pos = g_f.add_vectors(vec_scaled, plane_pos)

        # Check if there are no points to fly to
        if len(waypoint_list) == 0:
            msg = g_v.gui.display_message
            msg("no waypoints to fly to",
                "no path computed", MT.CONTROLLER)
            self.path_computation_status = SS.FAILED
            return

        # make plane starting waypoint
        start_point = p_o.Waypoint(g_v.MissionType.PLANE,
                                   plane_pos, plane_z,
                                   is_mission=g_v.Activity.ACTIVE)

        # add inertia post turn waypoint
        P_R = PREFERRED_TURN_RADIUS
        heading_vec = g_f.rotate_vector((P_R, 0), heading)
        inertia_pos = g_f.add_vectors(plane_pos, heading_vec)
        start_point.post_turn_waypoint = p_o.Waypoint(0, inertia_pos)

        # create path containing plane waypoint
        path_to_now = p_o.Path([start_point])

        # start recursive path search
        self.start_time = time.time()
        search = p_f.recursive_path_search
        found_path, max_depth = search(path_to_now, waypoint_list,
                                       profile, 0, waypoint_list)

        if found_path is None:
            msg = g_v.gui.display_message
            msg("could not make path",
                f"max depth: {max_depth}", MT.CONTROLLER)
            self.path_computation_status = SS.FAILED
            return

        self.chosen_path = found_path

        # strip the plane position, takeoff, and landing loiter
        w_l = self.chosen_path.waypoint_list
        self.stripped_path = p_o.Path(w_l[2:-1])

        # reformat the chosen path to be exportable by
        # adding turning points to the path itself
        flattened_waypoint_list = list()
        for way in self.stripped_path.waypoint_list:

            m_i = way.mission_type
            mission_or_airdrop = m_i == MIT.WAYPOINTS

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

        self.path_computation_status = SS.SUCCESS
        g_v.gui.to_draw("path")
