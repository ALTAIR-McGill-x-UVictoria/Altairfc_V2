from __future__ import annotations

from core.datastore import DataStore


def ground_station_position(datastore: DataStore, default: list[float]) -> list[float]:
    """Ground station [lat deg, lon deg, alt m MSL] to point at.

    settings.gs_use_hardcoded (seeded from [ground_station].use_hardcoded,
    changeable by UpdateSetting from the GS):
        true  -> settings.gs_lat/lon/alt ([ground_station] coordinates, or a
                 GS UpdateSetting override of them)
        false -> the GS's own position from its GS_GPS command (0xC4), falling
                 back to `default` until one has been received
    """
    use_hardcoded = float(datastore.read("settings.gs_use_hardcoded", default=1.0)) >= 1.0
    if use_hardcoded:
        return [
            float(datastore.read("settings.gs_lat", default=default[0])),
            float(datastore.read("settings.gs_lon", default=default[1])),
            float(datastore.read("settings.gs_alt", default=default[2])),
        ]
    gs_lat = datastore.read("command.gs_lat", default=None)
    gs_lon = datastore.read("command.gs_lon", default=None)
    gs_alt = datastore.read("command.gs_alt", default=None)
    if all(v is not None for v in (gs_lat, gs_lon, gs_alt)):
        return [float(gs_lat), float(gs_lon), float(gs_alt)]
    return list(default)
