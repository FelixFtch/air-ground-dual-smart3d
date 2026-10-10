"""Single-owner, one-apply pipeline for ordered map-cache updates.

While the previous native delta is applied, its caller may prepare and request
one next sensor frame. submit() first drains the previous future: there is no
queue of cache jobs or older sensor frames. The map/execution locks and revision
checks remain owned by the caller's apply function.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass


@dataclass(frozen=True)
class FusionFrame:
    robot_id: str
    sensor_id: str
    frame: int
    capture_sim_time: float
    arrival_wall: float
    selected_wall: float
    request_started_wall: float
    response_wall: float
    prepare_s: float
    pose_lock_s: float
    rays_s: float
    points: int
    free_rays: int
    ray_coverage: dict
    calibration_id: str
    sensor_to_world: list


class OrderedApplyPipeline:
    """One producer, one worker, at most one submitted cache apply.

    Exceptions are observed at poll(), the next submit(), or close(). Context
    exit drains any pending apply even when sensor preparation/RPC failed; an
    existing producer exception takes precedence over a concurrent apply error.
    """
    def __init__(self):
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='map-cache-apply')
        self._future = None
        self._closed = False

    def drain(self):
        future, self._future = self._future, None
        if future is not None:
            return future.result()

    def poll(self):
        if self._future is not None and self._future.done():
            return self.drain()

    def submit(self, function, *args, **kwargs):
        if self._closed:
            raise RuntimeError('map_apply_pipeline_closed')
        # Drain before submit: the executor never has queued apply jobs behind
        # an executing one. Arguments bind this exact frame, not loop variables.
        self.drain()
        self._future = self._executor.submit(function, *args, **kwargs)

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.drain()
        finally:
            self._executor.shutdown(wait=True)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            self.close()
        except BaseException:
            if exc_type is None:
                raise
        return False
