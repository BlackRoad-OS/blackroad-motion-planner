"""
BlackRoad Motion Planner - Robot/drone motion planning with A* pathfinding,
waypoint management, and collision detection.
"""

import math
import heapq
import sqlite3
import json
import time
import argparse
import sys
import os
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict, Any
from datetime import datetime

# ── ANSI colours ──────────────────────────────────────────────────────────────
RED    = '\033[0;31m'; GREEN  = '\033[0;32m'; YELLOW = '\033[1;33m'
CYAN   = '\033[0;36m'; BLUE   = '\033[0;34m'; MAGENTA= '\033[0;35m'
BOLD   = '\033[1m';    DIM    = '\033[2m';    NC     = '\033[0m'

DB_PATH = os.environ.get("MOTION_DB", os.path.expanduser("~/.blackroad/motion_planner.db"))

# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class Point3D:
    x: float
    y: float
    z: float = 0.0

    def dist(self, other: "Point3D") -> float:
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2 + (self.z-other.z)**2)

    def horiz_dist(self, other: "Point3D") -> float:
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def __iter__(self):
        yield self.x; yield self.y; yield self.z


@dataclass
class Waypoint:
    id: str
    name: str
    position: Point3D
    speed: float = 1.0          # m/s approach speed
    hover_time: float = 0.0     # seconds to hover at point
    heading: float = 0.0        # degrees
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    tags: str = ""


@dataclass
class Obstacle:
    id: str
    cx: float
    cy: float
    cz: float
    radius: float
    height: float = 10.0
    kind: str = "static"        # static | dynamic | no-fly-zone

    def contains(self, p: Point3D, clearance: float = 0.0) -> bool:
        horiz = math.sqrt((p.x - self.cx)**2 + (p.y - self.cy)**2)
        return horiz <= self.radius + clearance and p.z <= self.height


@dataclass
class PathResult:
    mission_id: str
    start: str
    goal: str
    algorithm: str
    raw_points: int
    smooth_points: int
    total_distance: float
    estimated_time: float
    planning_ms: float
    safe: bool
    path: List[Tuple[float, float]]


@dataclass
class Mission:
    id: str
    name: str
    start_wp: str
    goal_wp: str
    status: str
    total_distance: float
    estimated_time: float
    created_at: str
    notes: str = ""


# ── Database helpers ───────────────────────────────────────────────────────────

def get_conn(db_path: str = DB_PATH) -> sqlite3.Connection:
    os.makedirs(os.path.dirname(db_path), exist_ok=True)
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    return conn


def init_db(db_path: str = DB_PATH):
    with get_conn(db_path) as c:
        c.executescript("""
        CREATE TABLE IF NOT EXISTS waypoints (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL UNIQUE,
            x REAL NOT NULL, y REAL NOT NULL, z REAL DEFAULT 0.0,
            speed REAL DEFAULT 1.0, hover_time REAL DEFAULT 0.0,
            heading REAL DEFAULT 0.0, tags TEXT DEFAULT '',
            created_at TEXT
        );
        CREATE TABLE IF NOT EXISTS obstacles (
            id TEXT PRIMARY KEY,
            cx REAL NOT NULL, cy REAL NOT NULL, cz REAL DEFAULT 0.0,
            radius REAL NOT NULL, height REAL DEFAULT 10.0,
            kind TEXT DEFAULT 'static'
        );
        CREATE TABLE IF NOT EXISTS missions (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            start_wp TEXT NOT NULL, goal_wp TEXT NOT NULL,
            status TEXT DEFAULT 'planned',
            total_distance REAL, estimated_time REAL,
            path_json TEXT,
            created_at TEXT, notes TEXT DEFAULT ''
        );
        CREATE INDEX IF NOT EXISTS idx_wp_name ON waypoints(name);
        """)


# ── Core algorithms ────────────────────────────────────────────────────────────

def douglas_peucker(pts: List[Tuple[float,float]], eps: float) -> List[Tuple[float,float]]:
    """Ramer-Douglas-Peucker path simplification."""
    if len(pts) <= 2:
        return pts

    def seg_dist(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        if dx == dy == 0:
            return math.hypot(p[0]-a[0], p[1]-a[1])
        t = max(0, min(1, ((p[0]-a[0])*dx + (p[1]-a[1])*dy) / (dx*dx+dy*dy)))
        return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)

    dmax, idx = 0.0, 0
    for i in range(1, len(pts)-1):
        d = seg_dist(pts[i], pts[0], pts[-1])
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        l = douglas_peucker(pts[:idx+1], eps)
        r = douglas_peucker(pts[idx:], eps)
        return l[:-1] + r
    return [pts[0], pts[-1]]


def astar(start: Tuple[int,int], goal: Tuple[int,int],
          obstacles: List[Obstacle], res: float,
          max_iter: int = 50_000) -> Optional[List[Tuple[int,int]]]:
    """A* on discrete grid; returns grid coords or None."""
    def blocked(gx, gy):
        p = Point3D(gx * res, gy * res, 0.0)
        return any(ob.contains(p, clearance=0.3) for ob in obstacles)

    def h(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    DIRS = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
    COSTS = [1.0, 1.0, 1.0, 1.0, 1.4142, 1.4142, 1.4142, 1.4142]

    open_h: List[Tuple[float, Tuple[int,int]]] = []
    heapq.heappush(open_h, (0.0, start))
    g: Dict[Tuple[int,int], float] = {start: 0.0}
    came: Dict[Tuple[int,int], Tuple[int,int]] = {}
    closed: set = set()

    for _ in range(max_iter):
        if not open_h:
            break
        _, cur = heapq.heappop(open_h)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = []
            while cur in came:
                path.append(cur); cur = came[cur]
            path.append(start); path.reverse()
            return path
        for (dx, dy), cost in zip(DIRS, COSTS):
            nb = (cur[0]+dx, cur[1]+dy)
            if nb in closed or blocked(*nb):
                continue
            ng = g[cur] + cost
            if ng < g.get(nb, float('inf')):
                g[nb] = ng; came[nb] = cur
                heapq.heappush(open_h, (ng + h(nb, goal), nb))
    return None


# ── Planner class ──────────────────────────────────────────────────────────────

class MotionPlanner:
    def __init__(self, db_path: str = DB_PATH):
        self.db_path = db_path
        init_db(db_path)

    # ── Waypoint management ──
    def add_waypoint(self, name: str, x: float, y: float, z: float = 0.0,
                     speed: float = 1.0, hover: float = 0.0, tags: str = "") -> Waypoint:
        wid = f"wp_{int(time.time()*1000)}"
        wp = Waypoint(id=wid, name=name, position=Point3D(x, y, z),
                      speed=speed, hover_time=hover, tags=tags)
        with get_conn(self.db_path) as c:
            c.execute("""INSERT INTO waypoints
                (id,name,x,y,z,speed,hover_time,tags,created_at)
                VALUES (?,?,?,?,?,?,?,?,?)""",
                (wid,name,x,y,z,speed,hover,tags,wp.created_at))
        return wp

    def get_waypoint(self, name: str) -> Optional[Waypoint]:
        with get_conn(self.db_path) as c:
            r = c.execute("SELECT * FROM waypoints WHERE name=?", (name,)).fetchone()
        if not r: return None
        return Waypoint(id=r["id"], name=r["name"],
                        position=Point3D(r["x"], r["y"], r["z"]),
                        speed=r["speed"], hover_time=r["hover_time"],
                        tags=r["tags"], created_at=r["created_at"])

    def list_waypoints(self) -> List[Dict]:
        with get_conn(self.db_path) as c:
            return [dict(r) for r in c.execute(
                "SELECT id,name,x,y,z,speed,hover_time,tags FROM waypoints ORDER BY name")]

    # ── Obstacle management ──
    def add_obstacle(self, cx: float, cy: float, radius: float,
                     height: float = 10.0, kind: str = "static") -> Obstacle:
        oid = f"ob_{int(time.time()*1000)}"
        with get_conn(self.db_path) as c:
            c.execute("""INSERT INTO obstacles (id,cx,cy,cz,radius,height,kind)
                VALUES (?,?,?,?,?,?,?)""", (oid,cx,cy,0.0,radius,height,kind))
        return Obstacle(id=oid, cx=cx, cy=cy, cz=0.0, radius=radius, height=height, kind=kind)

    def _obstacles(self) -> List[Obstacle]:
        with get_conn(self.db_path) as c:
            return [Obstacle(**dict(r)) for r in
                    c.execute("SELECT id,cx,cy,cz,radius,height,kind FROM obstacles")]

    # ── Path planning ──
    def plan(self, start_name: str, goal_name: str,
             grid_res: float = 0.5, smooth_eps: float = 0.4) -> Optional[PathResult]:
        t0 = time.time()
        sw = self.get_waypoint(start_name)
        gw = self.get_waypoint(goal_name)
        if not sw or not gw:
            return None

        obstacles = self._obstacles()
        sg = (int(sw.position.x / grid_res), int(sw.position.y / grid_res))
        gg = (int(gw.position.x / grid_res), int(gw.position.y / grid_res))

        raw = astar(sg, gg, obstacles, grid_res)
        if raw is None:
            return None

        raw_pts = [(gx*grid_res, gy*grid_res) for gx, gy in raw]
        smooth = douglas_peucker(raw_pts, smooth_eps)

        dist = sum(math.hypot(smooth[i][0]-smooth[i-1][0],
                              smooth[i][1]-smooth[i-1][1])
                   for i in range(1, len(smooth)))
        avg_spd = (sw.speed + gw.speed) / 2 or 1.0
        eta = dist / avg_spd + sw.hover_time + gw.hover_time

        coll = self._collision_check(smooth, obstacles, 0.4)
        mid = f"mission_{int(time.time()*1000)}"
        with get_conn(self.db_path) as c:
            c.execute("""INSERT INTO missions
                (id,name,start_wp,goal_wp,status,total_distance,estimated_time,path_json,created_at)
                VALUES (?,?,?,?,?,?,?,?,?)""",
                (mid, f"{start_name}→{goal_name}", start_name, goal_name,
                 "planned", round(dist,3), round(eta,2),
                 json.dumps(smooth), datetime.now().isoformat()))

        return PathResult(
            mission_id=mid, start=start_name, goal=goal_name,
            algorithm="A*+DouglasPeucker",
            raw_points=len(raw_pts), smooth_points=len(smooth),
            total_distance=round(dist,3), estimated_time=round(eta,2),
            planning_ms=round((time.time()-t0)*1000,2),
            safe=coll["safe"], path=smooth)

    # ── Collision check ──
    def _collision_check(self, pts: List[Tuple], obstacles: List[Obstacle],
                         clearance: float) -> Dict:
        hits = []
        for i, pt in enumerate(pts):
            p = Point3D(pt[0], pt[1], 0.0)
            for ob in obstacles:
                if ob.contains(p, clearance):
                    hits.append({"point_idx": i, "obstacle_id": ob.id,
                                 "dist": round(math.hypot(p.x-ob.cx, p.y-ob.cy), 3)})
        return {"safe": len(hits) == 0, "collisions": hits}

    def check_collision(self, mission_id: str, clearance: float = 0.4) -> Dict:
        with get_conn(self.db_path) as c:
            row = c.execute("SELECT path_json FROM missions WHERE id=?", (mission_id,)).fetchone()
        if not row: return {"error": "mission not found"}
        pts = json.loads(row["path_json"])
        return self._collision_check(pts, self._obstacles(), clearance)

    # ── Mission listing ──
    def list_missions(self) -> List[Dict]:
        with get_conn(self.db_path) as c:
            return [dict(r) for r in c.execute(
                """SELECT id,name,start_wp,goal_wp,status,
                   total_distance,estimated_time,created_at
                   FROM missions ORDER BY created_at DESC LIMIT 50""")]

    # ── ASCII visualization ──
    def visualize(self, width: int = 50, height: int = 20,
                  mission_id: Optional[str] = None) -> str:
        obstacles = self._obstacles()
        waypoints = self.list_waypoints()
        grid = [['·' for _ in range(width)] for _ in range(height)]

        def to_grid(x, y):
            return int(x / 50 * (width-1)), height-1-int(y / 50 * (height-1))

        for ob in obstacles:
            gx, gy = to_grid(ob.cx, ob.cy)
            r = max(1, int(ob.radius / 50 * width))
            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    if dx*dx+dy*dy <= r*r:
                        nx,ny = gx+dx, gy+dy
                        if 0<=nx<width and 0<=ny<height:
                            grid[ny][nx] = '▓'

        if mission_id:
            with get_conn(self.db_path) as c:
                row = c.execute("SELECT path_json FROM missions WHERE id=?",
                                (mission_id,)).fetchone()
            if row:
                for pt in json.loads(row["path_json"]):
                    gx, gy = to_grid(pt[0], pt[1])
                    if 0<=gx<width and 0<=gy<height:
                        grid[gy][gx] = '◦'

        for wp in waypoints:
            gx, gy = to_grid(wp["x"], wp["y"])
            if 0<=gx<width and 0<=gy<height:
                grid[gy][gx] = wp["name"][0].upper()

        border = "+" + "-"*width + "+"
        rows = [border] + ["|"+''.join(r)+"|" for r in grid] + [border]
        return '\n'.join(rows)


# ── Rich output helpers ────────────────────────────────────────────────────────

def table(headers: List[str], rows: List[List], col_w: Optional[List[int]] = None):
    if not col_w:
        col_w = [max(len(str(h)), max((len(str(r[i])) for r in rows), default=0))
                 for i, h in enumerate(headers)]
    sep = "+" + "+".join("-"*(w+2) for w in col_w) + "+"
    def fmt(vals):
        return "|" + "|".join(f" {str(v):<{col_w[i]}} " for i,v in enumerate(vals)) + "|"
    print(f"{CYAN}{sep}{NC}")
    print(f"{BOLD}{fmt(headers)}{NC}")
    print(f"{CYAN}{sep}{NC}")
    for row in rows:
        print(fmt(row))
    print(f"{CYAN}{sep}{NC}")


def ok(msg): print(f"{GREEN}✔{NC} {msg}")
def err(msg): print(f"{RED}✖{NC} {msg}"); sys.exit(1)
def info(msg): print(f"{CYAN}ℹ{NC} {msg}")


# ── CLI ────────────────────────────────────────────────────────────────────────

def cmd_add_waypoint(args, planner):
    wp = planner.add_waypoint(args.name, args.x, args.y,
                               getattr(args,"z",0.0),
                               getattr(args,"speed",1.0),
                               getattr(args,"hover",0.0),
                               getattr(args,"tags",""))
    ok(f"Waypoint {BOLD}{wp.name}{NC} added ({wp.position.x},{wp.position.y},{wp.position.z})")


def cmd_add_obstacle(args, planner):
    ob = planner.add_obstacle(args.cx, args.cy, args.radius,
                               getattr(args,"height",10.0),
                               getattr(args,"kind","static"))
    ok(f"Obstacle {BOLD}{ob.id}{NC} r={ob.radius} at ({ob.cx},{ob.cy})")


def cmd_plan(args, planner):
    info(f"Planning {BOLD}{args.start}{NC} → {BOLD}{args.goal}{NC} …")
    res = planner.plan(args.start, args.goal)
    if not res:
        err("No path found — check waypoints exist and path is not fully blocked")
    safety = f"{GREEN}SAFE{NC}" if res.safe else f"{RED}COLLISION RISK{NC}"
    print(f"\n{BOLD}  Mission:{NC} {res.mission_id}")
    print(f"  Algorithm  : {res.algorithm}")
    print(f"  Raw points : {res.raw_points}  →  Smoothed: {res.smooth_points}")
    print(f"  Distance   : {YELLOW}{res.total_distance} m{NC}")
    print(f"  Est. time  : {YELLOW}{res.estimated_time} s{NC}")
    print(f"  Planning   : {res.planning_ms} ms")
    print(f"  Safety     : {safety}")


def cmd_list_waypoints(args, planner):
    wps = planner.list_waypoints()
    if not wps: info("No waypoints."); return
    table(["ID","Name","X","Y","Z","Speed","Tags"],
          [[w["id"][:12],w["name"],w["x"],w["y"],w["z"],w["speed"],w["tags"]] for w in wps])


def cmd_list_missions(args, planner):
    ms = planner.list_missions()
    if not ms: info("No missions."); return
    table(["ID","Route","Status","Dist(m)","ETA(s)","Created"],
          [[m["id"][:14],f"{m['start_wp']}→{m['goal_wp']}",
            m["status"],m["total_distance"],m["estimated_time"],
            m["created_at"][:19]] for m in ms])


def cmd_visualize(args, planner):
    mid = getattr(args, "mission_id", None)
    print(f"\n{CYAN}Motion Planning Space{NC}")
    print(f"{DIM}  ▓=obstacle  ◦=path  UPPERCASE=waypoint{NC}\n")
    print(planner.visualize(mission_id=mid))


def cmd_check_collision(args, planner):
    result = planner.check_collision(args.mission_id, getattr(args,"clearance",0.4))
    if "error" in result: err(result["error"])
    if result["safe"]:
        ok(f"Path is SAFE (checked {len(result.get('collisions',[]))} potential conflicts)")
    else:
        print(f"{RED}⚠  COLLISION RISK – {len(result['collisions'])} conflicts{NC}")
        for c in result["collisions"]:
            print(f"   point[{c['point_idx']}] ← obstacle {c['obstacle_id']}  dist={c['dist']}")


def main():
    ap = argparse.ArgumentParser(prog="motion_planner",
        description=f"{BOLD}BlackRoad Motion Planner{NC}")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("add-waypoint", help="Add a waypoint")
    p.add_argument("name"); p.add_argument("x",type=float); p.add_argument("y",type=float)
    p.add_argument("--z",type=float,default=0.0); p.add_argument("--speed",type=float,default=1.0)
    p.add_argument("--hover",type=float,default=0.0); p.add_argument("--tags",default="")

    p = sub.add_parser("add-obstacle", help="Add an obstacle")
    p.add_argument("cx",type=float); p.add_argument("cy",type=float)
    p.add_argument("radius",type=float)
    p.add_argument("--height",type=float,default=10.0)
    p.add_argument("--kind",default="static",choices=["static","dynamic","no-fly-zone"])

    p = sub.add_parser("plan", help="Plan A* path between two waypoints")
    p.add_argument("start"); p.add_argument("goal")

    p = sub.add_parser("check-collision", help="Check a mission for collisions")
    p.add_argument("mission_id"); p.add_argument("--clearance",type=float,default=0.4)

    sub.add_parser("list-waypoints", help="List all waypoints")
    sub.add_parser("list-missions",  help="List all missions")

    p = sub.add_parser("visualize", help="ASCII map of the planning space")
    p.add_argument("--mission-id",dest="mission_id",default=None)

    args = ap.parse_args()
    planner = MotionPlanner()
    dispatch = {
        "add-waypoint":    cmd_add_waypoint,
        "add-obstacle":    cmd_add_obstacle,
        "plan":            cmd_plan,
        "check-collision": cmd_check_collision,
        "list-waypoints":  cmd_list_waypoints,
        "list-missions":   cmd_list_missions,
        "visualize":       cmd_visualize,
    }
    dispatch[args.cmd](args, planner)


if __name__ == "__main__":
    main()
