import numpy as np
import pymap3d as pm
from scipy.spatial.transform import Rotation as R

# Guidance points the SPHERE (light source) at the ground station, NOT the Pixhawk.
# Pixhawk body frame is FRD (+X nose, +Y right, +Z down). The sphere does not tilt toward the nose: its servo
# swings it from nadir (body +Z) to horizontal in a vertical plane rotated 90 deg anti-clockwise (seen from
# above) from the Pixhawk's forward axis, i.e. toward body LEFT (-Y).
# Offset is a yaw about body +Z (down), positive clockwise seen from above, so 90 deg anti-clockwise = -pi/2.
SPHERE_YAW_OFFSET_RAD = -np.pi / 2

# Sphere frame: +X = sphere's horizontal tilt direction, +Y = 90 deg clockwise of it seen from above, +Z = down.
SPHERE_TO_BODY = R.from_euler("z", SPHERE_YAW_OFFSET_RAD)


def compute_error(
    attitude_q: list[float, float, float, float],
    gps_coords: list[float, float, float],
    gs_coords: list[float, float, float],
):
    # VERY IMPORTANT TO MAKE SURE QUATERNION IS FORMATTED AS [x, y, z, w].
    # GPS must be formatted as [lat, lon, alt]
    # gs_coords must be formatted as [lat, lon, alt].
    #
    # Both errors are in the sphere frame, not the Pixhawk body frame:
    #   azimuth_error_rad: GS bearing from the sphere's tilt direction, positive when the GS is clockwise of it
    #       seen from above (same sign as the old nose-relative error, so the RW gains stay valid).
    #   pitch_error_rad: GS angle below horizontal measured inside the sphere's tilt plane
    #       (0 = horizontal along the tilt direction, +pi/2 = straight down). Equals the GS elevation once
    #       the azimuth error is zero.

    gs_lat, gs_lon, gs_alt = gs_coords
    lat, lon, alt = gps_coords
    n, e, d = pm.geodetic2ned(
        gs_lat,
        gs_lon,
        gs_alt,
        lat,
        lon,
        alt,
    )

    r_err = np.array([n, e, d])
    x = r_err/np.linalg.norm(r_err)

    # PX4's attitude quaternion rotates body (FRD) -> NED. Chain sphere -> body -> NED and invert it to express
    # the NED target vector in sphere axes.
    world_to_sphere = (R.from_quat(attitude_q) * SPHERE_TO_BODY).inv().as_matrix()
    direction = world_to_sphere @ x
    azimuth_error_rad = np.arctan2(direction[1], direction[0])
    # Project onto the sphere's tilt plane (x-z) instead of using the full horizontal distance, so the pitch is
    # the angle the servo can actually reach. GS behind the sphere gives > pi/2 and the servo clamps to nadir.
    pitch_error_rad = np.arctan2(direction[2], direction[0])

    return azimuth_error_rad, pitch_error_rad
