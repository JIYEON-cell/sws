import math

def deg2rad(deg):
    return (deg * math.pi / 180.0)

def rad2deg(rad):
    return (rad * 180.0 / math.pi)

def distance(lat1, lon1, lat2, lon2, unit):
    theta = lon1 - lon2
    dist = math.sin(deg2rad(lat1)) * math.sin(deg2rad(lat2)) + \
            math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * \
            math.cos(deg2rad(theta))
    
    dist = math.acos(dist)
    dist = rad2deg(dist)
    dist = dist * 60 * 1.1515

    if(unit == "K"):
        dist = dist * 1.609344
    return dist