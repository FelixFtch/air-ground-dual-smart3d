"""Latest-frame fusion with bounded rates and fair overdue-deadline selection.

A sensor is eligible only when its period has elapsed. Among eligible sensors,
choose the most overdue *relative to its period*. This gives faster requested
streams more service without categorical priority: an overdue depth source can
out-rank LiDAR even when the mapper cannot meet all requested rates. Queueing
still belongs to the caller; this scheduler never creates or retains old frames.
"""
import math


def choose_candidate(candidates, last_served, now, period_for):
    """Choose a due candidate; return None when every source is rate limited.

    Periods are desired wall-clock start-to-start intervals, not promises under
    overload. The caller records the selected start in ``last_served``. A source
    first appearing starts due; absent sources consume no slots. Previously unseen sources are serviced once before already-serviced sources,
    so their initial deadline cannot reset forever under overload. Equal relative
    deadlines then prefer the longer absolute overdue interval and input order.
    """
    now = float(now)
    if not math.isfinite(now):
        raise ValueError('nonfinite scheduling time')
    best = None
    best_score = None
    for item in candidates:
        _, robot, kind, _, _ = item
        period = float(period_for(robot, kind))
        if not math.isfinite(period) or period <= 0:
            raise ValueError('invalid fusion period')
        unseen = (robot, kind) not in last_served
        previous = float(last_served.get((robot, kind), now - period))
        if not math.isfinite(previous):
            raise ValueError('nonfinite last fusion time')
        overdue = now - previous - period
        if overdue < -1e-9:
            continue
        score = (unseen, overdue / period, overdue)
        if best_score is None or score > best_score:
            best, best_score = item, score
    return best
