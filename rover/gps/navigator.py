import math

def get_bearing(curr_lat, curr_lon, target_lat, target_lon):
    """
    Returns compass bearing in degrees from current to target.
    0 = North, 90 = East, 180 = South, 270 = West
    """
    lat1 = math.radians(curr_lat)
    lat2 = math.radians(target_lat)
    dLon = math.radians(target_lon - curr_lon)

    x = math.sin(dLon) * math.cos(lat2)
    y = (math.cos(lat1) * math.sin(lat2) -
         math.sin(lat1) * math.cos(lat2) * math.cos(dLon))

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def get_distance_meters(lat1, lon1, lat2, lon2):
    """
    Returns distance in meters between two GPS coordinates.
    Uses the Haversine formula.
    """
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi    = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)

    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def get_turn_error(current_heading, target_bearing):
    """
    Returns degrees to turn.
    Negative = turn left, Positive = turn right.
    """
    error = target_bearing - current_heading
    if error > 180:
        error -= 360
    if error < -180:
        error += 360
    return error
