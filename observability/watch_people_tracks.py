"""Watch the live people tracker and summarize how long each id stays.

A track here is one camera keeping the same id. It is not the path drawn on the
map. ``strict`` ends at the first missed frame. ``loose`` keeps the same id
across gaps up to 1.5 s, the same gap that clears a speed estimate.
"""
import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CAMERAS = ('go2_01', 'uav_01')
LOOSE_GAP_S = 1.5


def percentile(values, p):
    if not values:
        return None
    values = sorted(values)
    k = (len(values) - 1) * p / 100
    lo = int(k)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] * (1 - (k - lo)) + values[hi] * (k - lo)


def load_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def episodes(frames, loose_gap_s):
    """frames: time-ordered dicts with t, ids, speed_ids for one camera."""
    strict, loose = [], []
    strict_open, loose_open = {}, {}
    prev_t = None
    for frame in frames:
        t = frame['t']
        ids = set(frame['ids'])
        if prev_t is not None and (t < prev_t or t - prev_t > loose_gap_s):
            strict.extend(strict_open.values())
            loose.extend(loose_open.values())
            strict_open, loose_open = {}, {}
        for key in list(strict_open):
            if key not in ids:
                strict.append(strict_open.pop(key))
        for key in list(loose_open):
            if t - loose_open[key]['last_t'] > loose_gap_s:
                loose.append(loose_open.pop(key))
        for key in ids:
            if key in strict_open:
                item = strict_open[key]
                item['frames'] += 1
                item['last_t'] = t
                item['speed'] = item['speed'] or key in frame['speed_ids']
            else:
                strict_open[key] = dict(id=key, frames=1, first_t=t, last_t=t, speed=key in frame['speed_ids'])
            if key in loose_open and t - loose_open[key]['last_t'] <= loose_gap_s:
                item = loose_open[key]
                item['frames'] += 1
                item['last_t'] = t
                item['speed'] = item['speed'] or key in frame['speed_ids']
            else:
                if key in loose_open:
                    loose.append(loose_open.pop(key))
                loose_open[key] = dict(id=key, frames=1, first_t=t, last_t=t, speed=key in frame['speed_ids'])
        prev_t = t
    strict.extend(strict_open.values())
    loose.extend(loose_open.values())
    return strict, loose


def hold_rate(frames):
    held = changed = 0
    for left, right in zip(frames, frames[1:]):
        if not 0 < right['t'] - left['t'] <= LOOSE_GAP_S:
            continue
        used = set()
        for person in left['obs']:
            best = None
            best_distance = 1.2
            for index, other in enumerate(right['obs']):
                if index in used:
                    continue
                distance = ((person['xy'][0] - other['xy'][0]) ** 2 + (person['xy'][1] - other['xy'][1]) ** 2) ** 0.5
                if distance < best_distance:
                    best, best_distance = index, distance
            if best is None:
                continue
            used.add(best)
            if person['id'] == right['obs'][best]['id']:
                held += 1
            else:
                changed += 1
    return held, changed


def histogram(items):
    buckets = {'1': 0, '2': 0, '3-4': 0, '5-9': 0, '10-19': 0, '20+': 0}
    for item in items:
        n = item['frames']
        key = '1' if n == 1 else '2' if n == 2 else '3-4' if n <= 4 else '5-9' if n <= 9 else '10-19' if n <= 19 else '20+'
        buckets[key] += 1
    return buckets


def weighted(items, minimum):
    total = sum(item['frames'] for item in items)
    kept = sum(item['frames'] for item in items if item['frames'] >= minimum)
    return kept, total


def summarize(by_camera):
    strict_all, loose_all = [], []
    counts, held, changed = [], 0, 0
    projected = with_speed = 0
    for frames in by_camera.values():
        frames.sort(key=lambda item: item['t'])
        strict, loose = episodes(frames, LOOSE_GAP_S)
        strict_all.extend(strict)
        loose_all.extend(loose)
        counts.extend(frame['n'] for frame in frames)
        h, c = hold_rate(frames)
        held += h
        changed += c
        for frame in frames:
            projected += len(frame['obs'])
            with_speed += sum(1 for person in frame['obs'] if person['speed'])
    long_loose = sorted(loose_all, key=lambda item: item['frames'], reverse=True)[:8]
    return dict(
        frames=len(counts),
        mean_people_per_frame=round(sum(counts) / len(counts), 3) if counts else None,
        frames_with_a_person=sum(1 for n in counts if n),
        id_hold=[held, held + changed],
        projected=projected,
        with_speed=with_speed,
        strict=dict(n=len(strict_all), p50=percentile([i['frames'] for i in strict_all], 50),
                    max=max((i['frames'] for i in strict_all), default=None), histogram=histogram(strict_all),
                    detections_on_tracks_at_least_4=weighted(strict_all, 4),
                    detections_on_tracks_at_least_10=weighted(strict_all, 10)),
        loose=dict(n=len(loose_all), p50=percentile([i['frames'] for i in loose_all], 50),
                   max=max((i['frames'] for i in loose_all), default=None), histogram=histogram(loose_all),
                   detections_on_tracks_at_least_4=weighted(loose_all, 4),
                   detections_on_tracks_at_least_10=weighted(loose_all, 10),
                   with_speed=sum(1 for i in loose_all if i['speed']),
                   longest=[dict(id=i['id'], frames=i['frames'], seconds=round(i['last_t'] - i['first_t'], 2), speed=i['speed'])
                            for i in long_loose]),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--minutes', type=float, default=15.)
    parser.add_argument('--out', type=Path, default=ROOT / 'runtime' / 'track_watch')
    args = parser.parse_args()
    folder = args.out / time.strftime('%Y%m%d-%H%M%S')
    folder.mkdir(parents=True, exist_ok=True)
    by_camera = defaultdict(list)
    seen = set()
    started = time.time()
    deadline = started + args.minutes * 60
    next_report = started + 60
    identity = None
    while time.time() < deadline:
        pointer = load_json(ROOT / 'runtime' / 'latest_run.json')
        run = Path(pointer['run_root']) if pointer and pointer.get('run_root') else None
        if run is not None:
            for camera in CAMERAS:
                document = load_json(run / camera / 'yolo26' / 'latest.json')
                if not document or document.get('capture_sim_time') is None:
                    continue
                identity = (document.get('model_identity') or {}).get('tracking_ids') or identity
                token = (camera, document.get('source_frame_token'), document.get('capture_sim_time'))
                if token in seen:
                    continue
                seen.add(token)
                observations = (document.get('observations') or {}).get('tracks') or []
                parsed = []
                speed_ids = set()
                for person in observations:
                    xy = (person.get('position_map') or person.get('center_xyz') or [None, None])[:2]
                    if xy[0] is None:
                        continue
                    speed = bool(person.get('velocity_xy_valid'))
                    parsed.append(dict(id=int(person['track_id']), xy=[float(xy[0]), float(xy[1])], speed=speed))
                    if speed:
                        speed_ids.add(int(person['track_id']))
                by_camera[camera].append(dict(
                    t=float(document['capture_sim_time']),
                    n=int(document.get('tracks_count') or len(document.get('tracks') or [])),
                    ids=[int(track['id']) for track in (document.get('tracks') or [])],
                    obs=parsed,
                    speed_ids=speed_ids,
                ))
        if time.time() >= next_report:
            report = summarize(by_camera)
            report.update(elapsed_s=round(time.time() - started, 1), tracking_ids=identity, run=str(run) if run else None)
            (folder / 'summary.json').write_text(json.dumps(report, indent=2))
            loose = report['loose']
            print(f"{report['elapsed_s']:.0f}s frames={report['frames']} mean={report['mean_people_per_frame']} "
                  f"hold={report['id_hold'][0]}/{report['id_hold'][1]} "
                  f"loose_p50={loose['p50']} loose_max={loose['max']} speed={report['with_speed']}/{report['projected']}",
                  flush=True)
            next_report += 60
        time.sleep(0.02)
    report = summarize(by_camera)
    report.update(elapsed_s=round(time.time() - started, 1), tracking_ids=identity, run=str(run) if run else None)
    (folder / 'summary.json').write_text(json.dumps(report, indent=2))
    print('WATCH_DONE', folder, flush=True)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
