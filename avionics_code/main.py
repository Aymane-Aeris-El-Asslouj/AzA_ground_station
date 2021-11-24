from avionics_code.communications import rf_communications as rf_c, server_communications as s_c
from avionics_code.mission import mission_control as m_c, mission_state as m_s
from avionics_code.mission import mission_profile as m_p
from avionics_code.gui import graphical_user_interface as g_u_i
import avionics_code.helpers.global_variables as g_v
from avionics_code.flight import flight_profile as f_p

import time
import asyncio


def main():
    """Launches the Avionics code"""

    g_v.init_time = time.time()

    """Graphical User Interface object that allows to visualize
    the progress of the mission along with adding info to the
    mission profile for testing purposes, or issuing drop authorizations
    and emergency landing orders for real missions"""
    g_v.gui = g_u_i.GUI()
    g_v.gui.activate()
    g_v.gui.update_u_i()
    g_v.gui.update_system_status()

    """This Server communications object handles communication with
    the auvsi suas server. It downloads the mission info to make a
    mission profile, then keeps uploading telemetry. In the future,
    it must also upload deliverables of the Computer vision team
    and download other planes' telemetry"""
    g_v.sc = s_c.ServerComs()

    """This RF communications object handles communication with
    the plane's Pixhawk and ground vehicle. It downloads plane
    status information (position, velocity, attitude etc), and
    uploads the mission plan to the Pixhawk. It also has a loop 
    to call for the server comms to upload telemetry and the 
    mission controller to refresh the mission state or land
    if telemetry is not received."""
    g_v.rf = rf_c.RFComs()

    """This Mission profile object stores all the mission info
    provided by the competition's server (border, waypoint,
    obstacles, etc)."""
    g_v.mp = m_p.MissionProfile()

    """Mission state object storing the waypoints that the plane needs
    to go through in order to accomplish the entire mission (mission
    waypoints, UGV drop, scouting for pictures, etc). Has a generation
    function to generate these true mission waypoints along with
    landing functions in case of timeout or emergency that change
    the plan to landing"""
    g_v.ms = m_s.MissionState()

    """telemetry history list that keeps track of all the telemetry packages
    that are received from the pixhawk or plan"""
    g_v.th = f_p.TelemetryHistory()

    """Mission control object that handles refreshing of mission state
    based on new plane status info, path recomputation/export and its
    conditions, along with timeout landing"""
    g_v.mc = m_c.MissionControl()

    # connect to server
    g_v.sc.connect()
    # connect to pixhawk
    asyncio.get_event_loop().run_until_complete(g_v.rf.connect())
    # get mission from server
    g_v.sc.get_mission()
    g_v.gui.update_map()
    # create full mission in mission state
    g_v.ms.generate()
    # start async thread for getting plane status
    asyncio.ensure_future(g_v.rf.fetch_plane_status_loop())
    asyncio.get_event_loop().run_forever()


if __name__ == '__main__':
    # Start the main function
    main()

