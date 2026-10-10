#!/usr/bin/env python3
"""Sample the live run and count occupied voxels inside each robot's own body box.

Run from the repo root: python3 tools/check_self_echo.py [samples]
A persistent nonzero count near a robot means self-echo leaked past the
capture-time body filter; note the offsets to size self_filter_margin_m.
"""
import json, sys, time
from pathlib import Path
import numpy as np, yaml

root = Path(__file__).resolve().parents[1]
samples = int(sys.argv[1]) if len(sys.argv) > 1 else 5
robots = {}
for team in (root/'profiles/teams/go2_fighter.yaml').read_text() and [yaml.safe_load((root/'profiles/teams/go2_fighter.yaml').read_text())]:
    for r in team['robots']:
        y = yaml.safe_load((root/f"profiles/robots/{r['profile']}.yaml").read_text())
        robots[r['robot_id']] = (np.array(y['body_half_extents_m'], float),
                                 np.array(y.get('body_offset_m', [0,0,0]), float),
                                 float(y.get('self_filter_margin_m', .30)))
for i in range(samples):
    run = json.loads((root/'runtime/latest_run.json').read_text())['run_root']
    state = json.loads((Path(run)/'state.json').read_text())
    display = json.loads((Path(run)/'shared/display.json').read_text())
    occ = np.array(display.get('occupied', []), float).reshape(-1, 3)
    line = []
    for r in state['robots']:
        half, off, margin = robots[r['robot_id']]
        rel = occ - (np.array(r['position_xyz']) + off)
        hit = np.all(np.abs(rel) <= half + margin, axis=1)
        line.append(f"{r['robot_id']}: margin={hit.sum()} +0.3={np.all(np.abs(rel)<=half+.3,axis=1).sum()}")
        if hit.sum():
            print('  echo offsets:', np.round(rel[hit][:6], 2).tolist())
    print(f"[{i}]", ' | '.join(line), flush=True)
    time.sleep(4)
