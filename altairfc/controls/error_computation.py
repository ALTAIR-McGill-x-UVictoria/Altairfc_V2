import numpy as np
import pymap3d as pm
import tomllib
from scipy.spatial.transform import Rotation as R

def compute_error(attitude_q: list[float,float,float,float], gps_coords: list[float,float,float]):
    # VERY IMPORTANT TO MAKE SURE QUATERNION IS FORMATTED AS [x, y, z, w].
    with open("settings.toml", "rb") as f:
        settings = tomllib.load(f)
    gs = settings["ground_station"]
    gs_lat = gs["latitude"]
    gs_lon = gs["longitude"]
    gs_alt = gs["altitude"]

    x_gs, y_gs, z_gs = pm.geodetic2ecef(gs_lat, gs_lon, gs_alt)
    r_gs = np.array([x_gs, y_gs, z_gs])
    lat, lon, alt = gps_coords
    x_bal, y_bal, z_bal = pm.geodetic2ecef(lat, lon, alt)
    r_bal = np.array([x_bal, y_bal, z_bal])

    r_err = r_gs - r_bal

    x = r_err/np.linalg.norm(r_err)

    world_to_body = R.from_quat(attitude_q).as_matrix()
    direction = world_to_body @ x
    azimuth_error_rad = np.arctan2(direction[1], direction[0])
    pitch_error_rad = np.arctan2(direction[2], direction[0])

    return azimuth_error_rad, pitch_error_rad
