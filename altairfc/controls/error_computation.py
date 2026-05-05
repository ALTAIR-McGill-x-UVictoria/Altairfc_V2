from pathlib import Path

import numpy as np
import pymap3d as pm
import tomllib
from scipy.spatial.transform import Rotation as R

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "settings.toml"
with CONFIG_PATH.open("rb") as f:
    GROUND_STATION = tomllib.load(f)["ground_station"]

def compute_error(
    attitude_q: list[float, float, float, float],
    gps_coords: list[float, float, float],
    gs_coords: list[float, float, float] | None = None,
):
    # VERY IMPORTANT TO MAKE SURE QUATERNION IS FORMATTED AS [x, y, z, w].
    # GPS must be formatted as [lat, lon, alt]
    # gs_coords overrides the TOML ground station position when provided.

    if gs_coords is not None:
        gs_lat, gs_lon, gs_alt = gs_coords
    else:
        gs_lat = GROUND_STATION["latitude"]
        gs_lon = GROUND_STATION["longitude"]
        gs_alt = GROUND_STATION["altitude"]

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
