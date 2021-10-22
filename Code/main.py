import Mission_profile_construction as M_p_c
import Dashboard_GUI as D_g


"""Create mission profile from json file and create dashboard"""
Mission_profile = M_p_c.Json_to_mission_profile(file='data.json')
Mission_profile.Compute_path()
Dash = D_g.Dashboard(Mission_profile)

"""Do First display update of dashboard and activate its GUI"""
Dash.Display_update()
Dash.GUI_input_manager_start()
