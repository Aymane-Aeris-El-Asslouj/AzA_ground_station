from avionics_code.communications import rf_communications as rf_c, server_communications as s_c
from avionics_code.mission import mission_control as m_c, mission_state as m_s
from avionics_code.mission import mission_profile as m_p
from avionics_code.gui import graphical_user_interface as g_u_i
import avionics_code.helpers.global_variables as g_v
from avionics_code.flight import telemetry_objects as t_o

import time
import asyncio
import os
import threading


def main():
    """Launches the Avionics code"""

    g_v.init_time = time.time()

    """Graphical User Interface object that allows to visualize
    the progress of the mission along with adding info to the
    mission profile for testing purposes, or issuing drop authorizations
    and emergency landing orders for real missions"""
    g_v.gui = g_u_i.GUI()
    g_v.gui.activate()

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
    event_loop = asyncio.get_event_loop()
    g_v.rf = rf_c.RFComs(event_loop)

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

    """Mission control object that handles refreshing of mission state
    based on new plane status info, path recomputation/export and its
    conditions, along with timeout landing"""
    g_v.mc = m_c.MissionControl()

    """telemetry history list that keeps track of all the telemetry packages
    that are received from the pixhawk or plan"""
    g_v.th = t_o.TelemetryHistory()

    """initialization commands"""
    # connect to server
    g_v.sc.connect()
    # get mission from server
    g_v.sc.get_mission()
    g_v.gui.to_draw["mission profile"] = True
    # create full mission in mission state
    g_v.ms.launch_generate().join()
    # connect to pixhawk
    asyncio.get_event_loop().run_until_complete(g_v.rf.connect())
    # set/upload plane parameters
    asyncio.get_event_loop().run_until_complete(g_v.rf.set_vehicle_parameters())
    # upload geofence to plane
    asyncio.get_event_loop().run_until_complete(g_v.rf.upload_geofence())
    # subscribe to plane telemetry
    asyncio.ensure_future(g_v.rf.subscribe_to_telemetry())

    """system loops"""
    # start thread for refreshing mission state
    threading.Thread(target=g_v.mc.refresh_mission_state_loop).start()

    try:
        event_loop.run_forever()
        pass
    except KeyboardInterrupt:
        g_v.gui.close_request = True
        g_v.rf.close_request = True
        g_v.mc.close_request = True

        def closing():
            time.sleep(1)
            os._exit(1)

        threading.Thread(target=closing).start()

if __name__ == '__main__':
    # Start the main function
    main()

