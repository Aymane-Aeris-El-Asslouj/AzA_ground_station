from avionics_code.helpers import geometrical_functions as g_f, parameters as para

import utm

FEET_PER_METER = para.FEET_PER_METER
MAP_REF = para.MAP_REF


def geographic_to_cartesian_list(points):
    """Perform geographic to cartesian transformation on a list of points
    and give them in the frame of reference of the center of map"""

    center = geographic_to_cartesian(MAP_REF)
    new_points = []
    for point in points:
        new_point = geographic_to_cartesian(point)
        new_points.append(g_f.sub_vectors(new_point, center))
    return new_points


def geographic_to_cartesian(point):
    """Use dictionary with longitude and latitude to get cartesian
    position in feet"""

    (x, y, p, q) = utm.from_latlon(point['latitude'], point['longitude'])
    return x * FEET_PER_METER, y * FEET_PER_METER


def geographic_to_cartesian_center(point):
    """Use dictionary with longitude and latitude to get cartesian
    position in feet with respect to center of map"""

    c_x, c_y = geographic_to_cartesian(MAP_REF)
    (x, y, p, q) = utm.from_latlon(point['latitude'], point['longitude'])
    return x * FEET_PER_METER - c_x, y * FEET_PER_METER - c_y


def cartesian_to_geo(point_2d):
    """Gives (lat,lon) from cartesian coordinates in the map
    center reference frame"""

    # Get cartesian position of center
    (x, y, p, q) = utm.from_latlon(MAP_REF['latitude'], MAP_REF['longitude'])

    # add to it the position of the point
    x, y = x + point_2d[0]/FEET_PER_METER, y + point_2d[1]/FEET_PER_METER

    return utm.to_latlon(x, y, p, q)


def frame_5_to_frame_0(frame_5_coordinates):
    """switches from frame 5 coordinates to frame 0 coordinates"""

    return {
        "latitude": frame_5_coordinates['latitude']/(10**7),
        "longitude": frame_5_coordinates['longitude']/(10**7),
        "altitude": frame_5_coordinates["altitude"]
    }


def geographic_to_cartesian_center_5(point):
    """Use dictionary with longitude and latitude to get cartesian
    position in feet with respect to center of map from frame 5"""

    frame_0 = frame_5_to_frame_0(point)
    return geographic_to_cartesian_center(frame_0)
