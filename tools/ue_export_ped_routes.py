"""Export UE spline actors into the pedestrian_routes YAML fragment.

Run this INSIDE the Unreal editor (Python editor scripting), not on Linux:

1. Copy this file into your UE project as Content/Python/ue_export_ped_routes.py
   (create the Python folder if needed; enable Editor Preferences > Plugins >
   Python Editor Script Plugin if it is off).
2. In the level, draw one spline per pedestrian: Place Actors panel -> search
   "Spline" -> drag into the viewport, then Alt+click the spline to add points
   and drag them like a pen tool. Rename each spline actor PED1, PED2, ... in
   the World Outliner. Closed loops are fine; leave the last point wherever
   the walk should end before looping back to the start.
3. Open the Output Log, switch its console to Python, and run:
   import ue_export_ped_routes; ue_export_ped_routes.export()
4. The fragment is written to Content/Python/pedestrian_routes.yaml next to
   this script and printed to the log. Commit it and paste it under
   pedestrian_routes: in profiles/scenes/air_ground_lab.yaml.

Coordinate note: map XY equals UE world XY divided by 100 (the CARLA y-flip
and this project's second flip cancel out), so what you see is what walks.
"""
import unreal
import os

STEP_M = 1.0          # sampling spacing along each spline
PREFIX = "PED"


def export():
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
        pts = []
        for i in range(n):
            d = length_cm * i / (n - 1)
            loc = comp.get_location_at_distance_along_spline(d, unreal.SplineCoordinateSpace.WORLD)
            pts.append([round(loc.x / 100.0, 2), round(loc.y / 100.0, 2)])
        out.append(pts)
        unreal.log("{}: {} points, {:.1f} m".format(label, len(pts), length_cm / 100.0))
    if not out:
        unreal.log_warning("No spline actors named PED* found; rename them in the World Outliner.")
        return
    lines = ["pedestrian_routes:"]
    for pts in out:
        lines.append("- " + " -> ".join("[{}, {}]".format(p[0], p[1]) for p in pts))
    text = "\n".join(lines) + "\n"
    path = os.path.join(os.path.dirname(__file__), "pedestrian_routes.yaml")
    with open(path, "w") as f:
        f.write(text)
    unreal.log("Wrote {} routes to {}".format(len(out), path))
    unreal.log(text)
