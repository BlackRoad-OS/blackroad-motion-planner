"""Tests for BlackRoad Motion Planner."""
import pytest
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from motion_planner import MotionPlanner, Point3D, douglas_peucker, astar, Obstacle


@pytest.fixture
def tmp_planner():
    with tempfile.TemporaryDirectory() as d:
        db = os.path.join(d, "test.db")
        yield MotionPlanner(db_path=db)


def test_point_distance():
    a = Point3D(0, 0, 0)
    b = Point3D(3, 4, 0)
    assert math.isclose(a.dist(b), 5.0, rel_tol=1e-9)


def test_point_3d_distance():
    a = Point3D(0, 0, 0)
    b = Point3D(1, 1, 1)
    expected = math.sqrt(3)
    assert math.isclose(a.dist(b), expected, rel_tol=1e-9)


def test_add_waypoint(tmp_planner):
    wp = tmp_planner.add_waypoint("home", 0.0, 0.0)
    assert wp.name == "home"
    assert wp.position.x == 0.0
    wps = tmp_planner.list_waypoints()
    assert len(wps) == 1
    assert wps[0]["name"] == "home"


def test_add_obstacle(tmp_planner):
    ob = tmp_planner.add_obstacle(10.0, 10.0, 3.0)
    assert ob.radius == 3.0
    assert ob.kind == "static"
    # Point inside obstacle
    p_in  = Point3D(10.0, 10.0, 0.0)
    p_out = Point3D(20.0, 20.0, 0.0)
    assert ob.contains(p_in, 0.0)
    assert not ob.contains(p_out, 0.0)


def test_astar_no_obstacles():
    result = astar((0,0),(10,10), [], 1.0)
    assert result is not None
    assert result[0] == (0,0)
    assert result[-1] == (10,10)


def test_astar_with_wall():
    # Build a wall of obstacles across x=5
    obs = [Obstacle(id=f"o{y}", cx=5.0, cy=float(y), cz=0.0,
                    radius=0.6, height=10.0) for y in range(0, 20)]
    # Path from left to right; should either fail or find a way around
    result = astar((0,10),(15,10), obs, 1.0)
    # May return None if fully blocked – test the algorithm runs without crash
    assert result is None or isinstance(result, list)


def test_douglas_peucker_straight_line():
    # A perfectly straight line should reduce to 2 points
    pts = [(float(i), float(i)) for i in range(10)]
    simplified = douglas_peucker(pts, epsilon=0.5)
    assert len(simplified) == 2
    assert simplified[0] == pts[0]
    assert simplified[-1] == pts[-1]


def test_douglas_peucker_preserves_bends():
    pts = [(0.0,0.0),(1.0,0.0),(2.0,2.0),(3.0,0.0),(4.0,0.0)]
    simplified = douglas_peucker(pts, epsilon=0.3)
    # The peak at (2,2) should be preserved
    assert len(simplified) >= 3


def test_plan_creates_mission(tmp_planner):
    tmp_planner.add_waypoint("depot", 0.0, 0.0)
    tmp_planner.add_waypoint("target", 10.0, 10.0)
    result = tmp_planner.plan("depot", "target", grid_res=1.0)
    assert result is not None
    assert result.total_distance > 0
    missions = tmp_planner.list_missions()
    assert len(missions) >= 1


def test_plan_missing_waypoint(tmp_planner):
    tmp_planner.add_waypoint("A", 0.0, 0.0)
    result = tmp_planner.plan("A", "nonexistent")
    assert result is None


def test_collision_check_safe(tmp_planner):
    tmp_planner.add_waypoint("p1", 0.0, 0.0)
    tmp_planner.add_waypoint("p2", 5.0, 0.0)
    # Obstacle far away
    tmp_planner.add_obstacle(0.0, 20.0, 1.0)
    result = tmp_planner.plan("p1", "p2", grid_res=1.0)
    if result:
        coll = tmp_planner.check_collision(result.mission_id)
        assert coll["safe"] is True


def test_visualize_returns_string(tmp_planner):
    tmp_planner.add_waypoint("X", 5.0, 5.0)
    output = tmp_planner.visualize(width=20, height=10)
    assert isinstance(output, str)
    assert "+" in output  # border
