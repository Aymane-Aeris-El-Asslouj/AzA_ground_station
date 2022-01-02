from avionics_code.references import global_variables as g_v, parameters as para

import time

RF_TELEMETRY_ALLOWED_DELAY = para.RF_TELEMETRY_ALLOWED_DELAY


class TelemetryHistory:
    """stores history of flight info"""

    def __init__(self):
        self.position = TelemetryObject()
        self.velocity = TelemetryObject()
        self.heading = TelemetryObject()
        self.armed = TelemetryObject()
        self.in_air = TelemetryObject()
        self.landed = TelemetryObject()
        self.flight_mode = TelemetryObject()
        self.status_text = TelemetryObject()
        self.mission_progress = TelemetryObject()


class TelemetryObject:
    """Parent class for telemetry objects"""

    def __init__(self):
        self.telemetry_time = None
        self.telemetry_rate = None
        self.data = None

    def set_data(self, telemetry_time, telemetry_rate, data):
        """store new set of received telemetry data"""

        self.telemetry_rate = telemetry_rate
        self.telemetry_time = telemetry_time
        self.data = data
        g_v.gui.to_draw("telemetry")

    def data_received(self):
        """checks if telemetry data has been received"""

        return self.data is not None

    def data_is_up_to_date(self):
        """checks if telemetry data is up-to-date"""

        time_diff = (time.time() - self.telemetry_time)
        return time_diff < RF_TELEMETRY_ALLOWED_DELAY
