import utm
import json
import copy
import sys

sample_item = {
  'AMSLAltAboveTerrain': None,
  'Altitude': 50,
  'AltitudeMode': 1,
  'autoContinue': True,
  'command': 22,
  'doJumpId': 1,
  'frame': 3,
  'params': [2, 0, 0, None, 24.36786542508521, 88.64302358272005, 10],
  'type':'SimpleItem'
}

landing_item = {
                "altitudesAreRelative": False,
                "complexItemType": "fwLandingPattern",
                "landCoordinate": [
                    38.1449043144047,
                    -76.42744191311141,
                    0
                ],
                "loiterClockwise": False,
                "loiterCoordinate": [
                    38.14549760439824,
                    -76.42362494930353,
                    40
                ],
                "loiterRadius": 75,
                "stopTakingPhotos": True,
                "stopVideoPhotos": True,
                "type": "ComplexItem",
                "valueSetIsDistance": False,
                "version": 2
            }

sample_plan = {
  "fileType": "Plan",
    "geoFence": {
        "circles": [
        ],
        "polygons": [
            {
                "inclusion": True,
                "polygon": [
                    [
                        38.14813563556963,
                        -76.4336313640843
                    ],
                    [
                        38.14791083344219,
                        -76.42206560252973
                    ],
                    [
                        38.14232237723878,
                        -76.424614484599
                    ],
                    [
                        38.14202096532991,
                        -76.43510354922833
                    ]
                ],
                "version": 1
            }
        ],
        "version": 2
    },
    "groundStation": "QGroundControl",
    "mission": {
        "cruiseSpeed": 15,
        "firmwareType": 12,
        "hoverSpeed": 5,
        "items": [
            {
                "AMSLAltAboveTerrain": None,
                "Altitude": 50,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": 22,
                "doJumpId": 1,
                "frame": 3,
                "params": [
                    15,
                    0,
                    0,
                    None,
                    38.14508523658602,
                    -76.42685783952264,
                    50
                ],
                "type": "SimpleItem"
            },
            {
                "AMSLAltAboveTerrain": None,
                "Altitude": 50,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": 16,
                "doJumpId": 2,
                "frame": 3,
                "params": [
                    0,
                    0,
                    0,
                    None,
                    38.14442314,
                    -76.42567055,
                    50
                ],
                "type": "SimpleItem"
            },
            {
                "AMSLAltAboveTerrain": None,
                "Altitude": 50,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": 16,
                "doJumpId": 3,
                "frame": 3,
                "params": [
                    0,
                    0,
                    0,
                    None,
                    38.14362380067181,
                    -76.4264175283225,
                    50
                ],
                "type": "SimpleItem"
            },
            {
                "altitudesAreRelative": True,
                "complexItemType": "fwLandingPattern",
                "landCoordinate": [
                    38.1449043144047,
                    -76.42744191311141,
                    0
                ],
                "loiterClockwise": False,
                "loiterCoordinate": [
                    38.14549760439824,
                    -76.42362494930353,
                    40
                ],
                "loiterRadius": 75,
                "stopTakingPhotos": True,
                "stopVideoPhotos": True,
                "type": "ComplexItem",
                "valueSetIsDistance": False,
                "version": 2
            }
        ],
        "plannedHomePosition": [
            38.14469505879297,
            -76.428021188769,
            1
        ],
        "vehicleType": 1,
        "version": 2
    },
    "rallyPoints": {
        "points": [
        ],
        "version": 2
    },
    "version": 1
}

def get_zone(pt):
    (x, y, z, d) = utm.from_latlon(pt['latitude'], pt['longitude'])
    return z

def get_dirc(pt):
    (x, y, z, d) = utm.from_latlon(pt['latitude'], pt['longitude'])
    return d

def get_latlon(x, y, z, d):
  return utm.to_latlon(x, y, z, d)


def generate_plan(boundary, waypoints, center):
  (xc, yc, zc, dc) = utm.from_latlon(center['latitude'], center['longitude'])
  poly = []
  plan = copy.copy(sample_plan)
  for pt in boundary:
    (x, y) = get_latlon(xc + pt[0] / 3.281, yc + pt[1] / 3.281, zc, dc)
    poly.append([x, y])
  plan['geoFence']['polygons'] = [{
    'inclusion': True,
    'polygon': poly,
    'version': 1,
  }]
  wayp = []
  for i in range(len(waypoints)):
    item = copy.copy(sample_item)
    item['command'] = 22 if i == 0 else 16
    item['doJumpId'] = i + 1
    (x, y) = get_latlon(xc + waypoints[i][0][0] / 3.281, yc + waypoints[i][0][1] / 3.281, zc, dc)
    item['params'] = [0, 0, 0, None, x, y, waypoints[i][1] / 3.281]
    item['AltitudeMode'] = 2
    item['Altitude'] = waypoints[i][1] / 3.281
    item['AMSLAltAboveTerrain'] = None
    item['frame'] = 0
    wayp.append(item)
  wayp.append(landing_item)
  plan['mission']['items'] = wayp
  with open('extra files/aza.plan', 'w') as f:
    json.dump(plan, f)
    



