import math

#Given drone position, gimbal orientation, and LRF range, return estimated 3D vehicle position.
def triangulate_vehicle(drone_pos, yaw_deg, pitch_deg, range_m):
    """
    drone_pos: (x, y, z) in meters -- We assume it's an array with three elements:
                                      [0] -> x, [1] -> y, [2] -> z
    yaw_deg:   yaw angle in degrees (0° = East, 90° = North) -- Horizontal Rotation
    pitch_deg: pitch angle in degrees (0° = horizontal, -90° = down) -- Vertical Rotation
    range_m:   distance to vehicle in meters
    """
    # Convert angles to radians
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)

    # Direction vector from gimbal orientation
    dx = math.cos(pitch) * math.cos(yaw)
    dy = math.cos(pitch) * math.sin(yaw)
    dz = math.sin(pitch)

    # Vehicle position = drone position + direction * range
    vx = drone_pos[0] + dx * range_m
    vy = drone_pos[1] + dy * range_m
    vz = drone_pos[2] + dz * range_m

    return vx, vy, vz


def euclidean_distance(p1, p2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))


def compute_speed(pos1, pos2, delta_t):
    dist = euclidean_distance(pos1, pos2)
    return dist / delta_t  # meters per second
