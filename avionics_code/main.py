from avionics_code.communications import rf_communications as rf_c, server_communications as s_c
from avionics_code.mission import mission_control as m_c, mission_state as m_s, mission_profile as m_p
from avionics_code.gui import graphical_user_interface as g_u_i, gui_input_manager as g_i_m
import avionics_code.helpers.global_variables as g_v
from avionics_code.flight import flight_profile as f_p

def launch_avionics():
    """Launches the Avionics code"""

    print("")

    """This Server communications object handles communication with
    the auvsi suas server. It downloads the mission info to make a
    mission profile, then keeps uploading telemetry. In the future,
    it must also upload deliverables of the Computer vision team
    and download other planes' telemetry"""
    g_v.sc = s_c.ServerComs()

    """Connect to the interop server"""
    g_v.sc.connect()

    """This RF communications object handles communication with
    the plane's Pixhawk and ground vehicle. It downloads plane
    status information (position, velocity, attitude etc), and
    uploads the mission plan to the Pixhawk. It also has a loop 
    to call for the server comms to upload telemetry and the 
    mission controller to refresh the mission state or land
    if telemetry is not received."""
    g_v.rf = rf_c.RFComs()

    """Connect to the Pixhawk"""
    g_v.rf.connect()

    """This Mission profile object stores all the mission info
    provided by the competition's server (border, waypoint,
    obstacles, etc)."""
    g_v.mp = g_v.sc.get_mission()

    """telemetry history list that keeps track of all the telemetry packages
    that are received from the pixhawk or plan"""
    g_v.th = f_p.TelemetryHistory()

    """Download once the info of the plane to create an initial
    flight plan and add it to the telemetry history list"""
    g_v.rf.fetch_plane_status()

    """Mission state object storing the waypoints that the plane needs
    to go through in order to accomplish the entire mission (mission
    waypoints, UGV drop, scouting for pictures, etc). Has a generation
    function to generate these true mission waypoints along with
    landing functions in case of timeout or emergency that change
    the plan to landing"""
    g_v.ms = m_s.MissionState()

    """Generate complete set of waypoints to accomplish all parts
    of the mission (mission waypoints, UGV drop, map scouting, etc)
    based on the mission profile and the current status of the plane"""
    g_v.ms.generate()

    """Mission control object that handles refreshing of mission state
    based on new plane status info, path recomputation/export and its
    conditions, along with timeout landing"""
    g_v.mc = m_c.MissionControl()

    """Compute selected path object that stores the latest exported trajectory
        waypoint list as a Path object computed from the Mission profile,
        Mission State, and current plane status info. Also calls RF comms
        to convert the path into flight plan and export it to the Pixhawk"""
    g_v.mc.compute_path()

    """Starts the loop of RF comms that keeps fetching for plane
    status info and calling server comms' telemetry upload and
    mission control's mission state refresh"""
    #g_v.rf.loop_fetch_plane_status()

    """Graphical User Interface object that allows to visualize
    the progress of the mission along with adding info to the
    mission profile for testing purposes, or issuing drop authorizations
    and emergency landing orders for real missions"""
    g_v.gui = g_u_i.GUI()
    g_v.gui.display_update()
    g_i_m.gui_input_manager_start()


if __name__ == '__main__':
    launch_avionics()
