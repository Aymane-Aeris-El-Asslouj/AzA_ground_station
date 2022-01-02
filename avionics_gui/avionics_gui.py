from pygui import gui as p_y
from references import parameters as para
from references import global_variables as g_v

from avionics_gui.layers import background_layer
from avionics_gui.layers import mission_state_layer
from avionics_gui.layers import telemetry_layer
from avionics_gui.layers import mission_profile_layer
from avionics_gui.layers import path_layer
from avionics_gui.layers import user_interface_layer
from avionics_gui.layers import system_status_layer

from pathlib import Path

FRAMES_PER_SECOND = para.FRAMES_PER_SECOND
F_P_S = FRAMES_PER_SECOND
REGULAR_UPDATES_PER_SECOND = para.REGULAR_UPDATES_PER_SECOND
R_U_S = REGULAR_UPDATES_PER_SECOND
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE
FONT_SIZE = para.FONT_SIZE
F_S = FONT_SIZE
FONT_TYPE = para.FONT_TYPE
F_T = FONT_TYPE

MT = g_v.MessageType


class AvionicsGUI(p_y.GUI):
    """avionics gui for the AzA avionics team"""

    def __init__(self):
        super().__init__('AzA Avionics', (D_S * 2, D_S), F_P_S, R_U_S)

        # load settings
        dir_path = Path(__file__).parent
        relative_path = 'resources/avionics_gui_settings.json'
        settings_path = (dir_path / relative_path).resolve()
        self.load_settings(settings_path, {})

        # set the font type and size
        self.set_font(F_T, F_S)

        # dictionary of all the window layers
        self.add_layer("background",
                       background_layer.BackgroundLayer(self))
        self.add_layer("system status",
                       system_status_layer.SystemStatusLayer(self))
        self.add_layer("mission profile",
                       mission_profile_layer.MissionProfileLayer(self))
        self.add_layer("mission state",
                       mission_state_layer.MissionStateLayer(self))
        self.add_layer("path",
                       path_layer.PathLayer(self))
        self.add_layer("telemetry",
                       telemetry_layer.TelemetryLayer(self))
        self.add_layer("user interface",
                       user_interface_layer.UserInterfaceLayer(self))

    def display_message(self, msg_1, msg_2, level):
        """display a message on screen with a certain priority level"""

        self.layers["system status"].display_message(msg_1, msg_2, level)

    def is_controller_busy(self):
        """display that the controller is busy"""

        # check if controller is busy
        busy = False
        if g_v.mc is None:
            busy = True
        elif len(g_v.mc.queue) > 0:
            busy = True

        # show message if busy
        if busy:
            self.display_message("controller is busy",
                                 "retry later", MT.CONTROLLER)

        return busy
