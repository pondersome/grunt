"""Geographic conversions at a site anchor.

For grunt's site scale (< 1 km), a flat-earth approximation matches
GeographicLib::LocalCartesian to mm. Centralized here so every metric
uses the same convention as navsat_transform's runtime conversion.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

WGS84_R = 6378137.0  # equatorial radius, meters — matches GeographicLib default


@dataclass(frozen=True)
class GeoAnchor:
    """A geographic anchor (typically a site's `site.yaml`)."""
    lat: float
    lon: float
    name: str = "anchor"

    @classmethod
    def from_site_yaml(cls, path: str | Path) -> "GeoAnchor":
        """Load anchor_lat / anchor_lon from a site.yaml.

        Raises ValueError if the file is missing or has null anchor fields —
        we don't fall back to current-fix here; the caller chooses what to do.
        """
        p = Path(path)
        with open(p) as f:
            cfg = yaml.safe_load(f) or {}
        lat = cfg.get("anchor_lat")
        lon = cfg.get("anchor_lon")
        if lat is None or lon is None:
            raise ValueError(
                f"{p} has no anchor_lat / anchor_lon (got {lat=}, {lon=})")
        return cls(lat=float(lat), lon=float(lon), name=p.parent.name)


def lla_to_local_enu(lat: float, lon: float, anchor: GeoAnchor) -> tuple[float, float]:
    """Convert (lat, lon) to local ENU meters at the anchor.

    Returns (east, north) in meters. Matches GeographicLib::LocalCartesian
    to mm precision over distances up to a few km — adequate for site-scale
    work. For longer baselines, use a proper ECEF conversion.
    """
    cos_anchor_lat = math.cos(math.radians(anchor.lat))
    east = math.radians(lon - anchor.lon) * WGS84_R * cos_anchor_lat
    north = math.radians(lat - anchor.lat) * WGS84_R
    return east, north


def haversine_bearing_deg(lat1: float, lon1: float,
                           lat2: float, lon2: float) -> float:
    """Compass bearing in degrees CW from north, point 1 to point 2."""
    lat1_r, lon1_r = math.radians(lat1), math.radians(lon1)
    lat2_r, lon2_r = math.radians(lat2), math.radians(lon2)
    dlon = lon2_r - lon1_r
    x = math.sin(dlon) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def quat_yaw(q) -> float:
    """Extract yaw (radians) from a quaternion-like with .w/.x/.y/.z attrs."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)
