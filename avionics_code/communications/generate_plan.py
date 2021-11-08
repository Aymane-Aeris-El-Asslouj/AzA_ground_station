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
  'params': [15, 0, 0, None, 24.36786542508521, 88.64302358272005, 50],
  'type':'SimpleItem'
}

sample_plan = {
  'fileType': 'Plan',
  'geoFence': {
    'circles': [],
    'polygons': [{
      'inclusion':
      True,
      'polygon': [[24.36911074484618, 88.64152423421103],
            [24.36942038441295, 88.64336865084869],
            [24.36915934201101, 88.64521306818669],
            [24.366467836674573, 88.64461403751278],
            [24.36773143007941, 88.64304403044827],
            [24.366541390999757, 88.64121285606072],
            [24.3679351982147, 88.6405751246221]],
      'version':
      1
    }],
    'version':
    2
  },
  'groundStation': 'QGroundControl',
  'mission': {
    'cruiseSpeed': 15,
    'firmwareType': 12,
    'hoverSpeed': 5,
    'items': [{
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 22,
      'doJumpId': 1,
      'frame': 3,
      'params': [15, 0, 0, None, 24.36786542508521, 88.64302358272005, 50],
      'type':
      'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 2,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36865988, 88.6430055, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 3,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36883733, 88.64409346, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 4,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36854687, 88.64444611, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 5,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36822931, 88.64438607, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 6,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36849969, 88.64339196, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 7,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36822689, 88.64173356, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 8,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36878636, 88.64203935, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 9,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36909845, 88.64306845, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 10,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36792514, 88.64329677, 50],
      'type': 'SimpleItem'
    }, {
      'AMSLAltAboveTerrain': None,
      'Altitude': 50,
      'AltitudeMode': 1,
      'autoContinue': True,
      'command': 16,
      'doJumpId': 11,
      'frame': 3,
      'params': [0, 0, 0, None, 24.36785026, 88.6417443, 50],
      'type': 'SimpleItem'
    }],
    'plannedHomePosition': [24.36786542508521, 88.64302358272005, 24],
    'vehicleType':
    2,
    'version':
    2
  },
  'rallyPoints': {
    'points': [],
    'version': 2
  },
  'version': 1
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
    (x, y) = get_latlon(xc + waypoints[i][0] / 3.281, yc + waypoints[i][1] / 3.281, zc, dc)
    item['params'] = [(15 if i == 0 else 0), 0, 0, None, x, y, 50]
    wayp.append(item)
  plan['mission']['items'] = wayp
  with open('../extra files/aza.plan', 'w') as f:
    json.dump(plan, f)
    



