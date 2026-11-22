"""Single source of truth for the UAV competition route.

Coordinates use ArduPilot local NED: X north/forward, Y east/right and the
third value is altitude above the takeoff point (positive up).
"""

COMPETITION_WAYPOINTS_NED = (
    (0.00, 0.00, 1.00, "TAKEOFF"),
    (2.90, 0.00, 1.00, "APPROACH_FRAME"),
    # ArUco 中心位于 x=4.00；扫描位在门框远侧 x=3.80，留出约0.20m视场
    # 裕量，容忍里程计偏差后仍能可靠看到地面码。
    (3.80, 0.00, 1.00, "SCAN_ZONE"),
    (0.50, 0.00, 1.00, "RETURN_TAKEOFF_ZONE"),
    (0.90, 1.85, 1.00, "UGV_STOP_APPROACH"),
    (0.00, 1.85, 0.80, "STRIKE_ZONE"),
    (0.00, 1.85, 0.45, "BALLOON_ALT"),
    (0.00, 0.00, 0.00, "LAND"),
)
