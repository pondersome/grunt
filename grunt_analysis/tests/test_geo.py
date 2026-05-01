"""Sanity tests for ENU conversion at the Ranchero anchor."""
import math

from grunt_analysis.geo import GeoAnchor, lla_to_local_enu, haversine_bearing_deg


RANCHERO = GeoAnchor(lat=32.6780533, lon=-96.9225051, name="ranchero_test")


def test_anchor_is_origin():
    """The anchor itself converts to (0, 0)."""
    x, y = lla_to_local_enu(RANCHERO.lat, RANCHERO.lon, RANCHERO)
    assert abs(x) < 1e-6
    assert abs(y) < 1e-6


def test_north_displacement_is_positive_y():
    """0.001 deg north → ~111 m positive y."""
    x, y = lla_to_local_enu(RANCHERO.lat + 0.001, RANCHERO.lon, RANCHERO)
    assert abs(x) < 0.01      # essentially zero east
    assert 110 < y < 112      # ~111.13 m at WGS84 R


def test_east_displacement_is_positive_x():
    """0.001 deg east at lat 32.7° → ~93.7 m positive x."""
    x, y = lla_to_local_enu(RANCHERO.lat, RANCHERO.lon + 0.001, RANCHERO)
    assert abs(y) < 0.01
    expected = math.radians(0.001) * 6378137.0 * math.cos(math.radians(RANCHERO.lat))
    assert abs(x - expected) < 0.001


def test_haversine_bearing_north():
    """Point directly north has bearing 0°."""
    b = haversine_bearing_deg(RANCHERO.lat, RANCHERO.lon,
                                RANCHERO.lat + 0.001, RANCHERO.lon)
    assert abs(b - 0) < 0.5 or abs(b - 360) < 0.5


def test_haversine_bearing_east():
    """Point directly east has bearing 90°."""
    b = haversine_bearing_deg(RANCHERO.lat, RANCHERO.lon,
                                RANCHERO.lat, RANCHERO.lon + 0.001)
    assert abs(b - 90) < 0.5
