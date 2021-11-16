from avionics_code.helpers import parameters as para
import time

TIME_CUT_OFF_FOR_FLIGHT_STATUS = para.TIME_CUT_OFF_FOR_FLIGHT_STATUS

class TelemetryHistory:
    """stores history of flight info"""

    def __init__(self):
        self.flight_profile_list = list()

    def add_flight_profile(self, plane_pos, plane_z, ugv_pos, ugv_z, time_int):
        """add a flight profile from profile info"""

        fp = FlightProfile()
        fp.plane_obj = FlightObject(plane_pos, plane_z)
        fp.ugv_obj = FlightObject(ugv_pos, ugv_z)
        fp.time_int = time_int
        self.flight_profile_list.append(fp)

        # get rid of all outdated profiles
        self.clean_outdated()

    def last_flight_profile(self):
        """returns last flight profile added to list"""

        if len(self.flight_profile_list) > 0:
            return self.flight_profile_list[-1]
        else:
            return None

    def is_empty(self):
        """checks if flight profile list is empty"""

        return len(self.flight_profile_list) == 0

    def clear(self):
        """clears history of flight profiles"""

        return self.flight_profile_list.clear()

    def clean_outdated(self):
        """Deletes all outdated flight profiles"""

        current_time = time.time()

        def is_not_outdated(profile):
            return current_time - profile.time_int < TIME_CUT_OFF_FOR_FLIGHT_STATUS

        self.flight_profile_list = list(filter(is_not_outdated, self.flight_profile_list))


class FlightProfile:
    """stores all flight info received from the plane"""

    def __init__(self):
        # coordinates of plane (Flight object)
        self.plane_obj = None
        # coordinates of ugv (Flight object)
        self.ugv_obj = None
        # time where data was collected (int)
        self.time_int = None

class FlightObject:
    """Parent class for Flight objects"""

    def __init__(self, position_tuple, z):
        self.pos = position_tuple
        self.z = z
