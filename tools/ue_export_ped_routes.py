"""Export UE spline actors into the pedestrian_routes YAML fragment.

Run this INSIDE the Unreal editor (Python editor scripting), not on Linux:

1. Copy this file into your UE project as Content/Python/ue_export_ped_routes.py
   (create the Python folder if needed; enable Editor Preferences > Plugins >
   Python Editor Script Plugin if it is off).
2. In the level, draw one spline per pedestrian: Place Actors panel -> search
   "Spline" -> drag onto the ground. Select it, Alt-drag an existing point to
   drop the next one, then move that point along the floor. Rename each spline
   PED1, PED2, ... in the World Outliner. One spline is one person's whole path.
3. Open the Output Log, switch its console to Python, and run:
   import ue_export_ped_routes; ue_export_ped_routes.export()
4. The fragment is written to Content/Python/pedestrian_routes.yaml next to
   this script and printed to the log. Replace the pedestrian_routes: block in
   profiles/scenes/air_ground_lab.yaml with that fragment, and set human_count
   to the number of PED splines.

The walker follows the drawn curve: the spline is sampled about every metre,
and the person walks those points in order, then returns to the first point
and repeats. Spline height is ignored. Map XY equals UE world XY divided by
100, so the path on the floor is the path they walk.
"""
import math
import os

STEP_M = 1.0
MAX_POINTS = 64
PREFIX = "PED"


def prepare_route(pts, min_m=1.0, max_n=MAX_POINTS):
    """Drop samples closer than min_m, including the loop back to the start."""
    kept = []
    for p in pts:
        if kept and math.dist(kept[-1], p) < min_m:
            continue
        kept.append((float(p[0]), float(p[1])))
    while len(kept) >= 3 and math.dist(kept[-1], kept[0]) < min_m:
        kept.pop()
    if len(kept) > max_n:
        stride = int(math.ceil((len(kept) - 1) / (max_n - 1)))
        kept = kept[::stride]
        if kept[-1] != pts[-1] and math.dist(kept[-1], pts[-1]) >= min_m:
            kept.append((float(pts[-1][0]), float(pts[-1][1])))
        kept = kept[:max_n]
    return kept


def routes_yaml(routes):
    lines = ["pedestrian_routes:"]
    for pts in routes:
        lines.append("-")
        for x, y in pts:
            lines.append("  - [{:.2f}, {:.2f}]".format(x, y))
    return "\n".join(lines) + "\n"


def export():
    import unreal
    out, world = [], unreal.EditorLevelLibrary.get_all_level_actors()
    splines = []
    for a in world:
        label = a.get_actor_label()
        if not label.startswith(PREFIX):
            continue
        comp = a.get_component_by_class(unreal.SplineComponent)
        if comp:
            splines.append((label, comp))
    splines.sort(key=lambda x: x[0])
    for label, comp in splines:
        length_cm = comp.get_spline_length()
        n = max(2, int(length_cm / (STEP_M * 100.0)) + 1)
        raw = []
        for i in range(n):
            d = length_cm * i / (n - 1)
            loc = comp.get_location_at_distance_along_spline(d, unreal.SplineCoordinateSpace.WORLD)
            raw.append([round(loc.x / 100.0, 2), round(loc.y / 100.0, 2)])
        pts = prepare_route(raw)
        if len(pts) < 2:
            unreal.log_warning("{} is shorter than 1 m; skipped.".format(label))
            continue
        out.append(pts)
        unreal.log("{}: {} points, {:.1f} m".format(label, len(pts), length_cm / 100.0))
    if not out:
        unreal.log_warning("No spline actors named PED* found; rename them in the World Outliner.")
        return
    text = routes_yaml(out)
    path = os.path.join(os.path.dirname(__file__), "pedestrian_routes.yaml")
    with open(path, "w") as f:
        f.write(text)
    unreal.log("Wrote {} routes to {}".format(len(out), path))
    unreal.log(text)
