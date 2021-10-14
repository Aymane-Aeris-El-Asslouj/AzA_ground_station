from os import path

import Telemetry
import Dashboard_GUI
import Path_objects
import Geometrical_functions

# If Map data file exists, load it onto a dashboard, otherwise, load an empty map dashboard
if path.exists('data.json'):
    Map_data = Telemetry.Map_data_telemetry()

    if len(Map_data.Mission_waypoint_list) > 0:
        Start_position = Path_objects.Waypoint(
            Geometrical_functions.Vector_average(
                [Waypoint_i.Point_2d() for Waypoint_i in Map_data.Mission_waypoint_list]))
    else:
        Start_position = Path_objects.Waypoint((0, 0))

    # create dashboard object with the chosen dashboard size and mission/map info
    Dash = Dashboard_GUI.Dashboard(Map_data.Map_center, Start_position, Map_data.Mission_waypoint_list,
                                   Map_data.Obstacles, Map_data.Border)

else:
    # create a dashboard with empty map
    Dash = Dashboard_GUI.Dashboard((0, 0), Path_objects.Waypoint((0, 0)), list(), list(),
                                   Path_objects.Border([]))

# Initialize Dashboard
Dash.Update()
Dash.Events()
