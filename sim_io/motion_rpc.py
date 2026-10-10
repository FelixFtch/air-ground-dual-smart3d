"""Acknowledged ApplyTransform RPC in an owned process, away from the map GIL.

The CARLA binding holds its interpreter GIL while ApplyBatchCommandsSync waits.
This worker isolates that wait; it does not make control fire-and-forget. Only
one caller batch may be pending, every batch keeps its ordered UE receipts, and
no world tick, spawning, actor destruction, or scene loading is exposed.
"""
from __future__ import annotations
import json
import math
import sys
import threading
import time
from types import SimpleNamespace

from sim_io.ipc import JsonProcess

SCHEMA = 'air_ground.motion_rpc/v1'


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class MotionServer:
    """Small synchronous protocol; injectable CARLA/clock for contract tests."""
    def __init__(self, carla_module, clock=time.monotonic):
        self.carla = carla_module
        self.clock = clock
        self.client = None
        self.epoch = None
        self.world_id = None
        self.allowed = frozenset()
        self.last_sequence = 0

    def dispatch(self, request):
        _require(isinstance(request, dict) and request.get('schema') == SCHEMA, 'invalid_motion_schema')
        if request.get('op') == 'init':
            _require(self.client is None, 'motion_already_initialized')
            host, port = request.get('host'), request.get('port')
            epoch, world_id, allowed = request.get('epoch'), request.get('world_id'), request.get('allowed_actor_ids')
            timeout = request.get('rpc_timeout_s', 2.)
            _require(isinstance(host, str) and 0 < len(host) <= 256, 'invalid_motion_host')
            _require(_integer(port) and 0 < port < 65536, 'invalid_motion_port')
            _require(isinstance(epoch, str) and 0 < len(epoch) <= 256, 'invalid_motion_epoch')
            _require(_integer(world_id) and world_id >= 0, 'invalid_motion_world')
            _require(isinstance(allowed, list) and 0 < len(allowed) <= 32 and
                     all(_integer(i) and i > 0 for i in allowed) and len(set(allowed)) == len(allowed), 'invalid_motion_actors')
            _require(_finite(timeout) and 0 < timeout <= 10, 'invalid_motion_timeout')
            client = self.carla.Client(host, port)
            client.set_timeout(float(timeout))
            _require(client.get_world().id == world_id, 'motion_world_mismatch')
            self.client, self.epoch, self.world_id, self.allowed = client, epoch, world_id, frozenset(allowed)
            return dict(schema=SCHEMA, ok=True, op='init', epoch=epoch, world_id=world_id,
                        allowed_actor_ids=sorted(self.allowed), sequence=0)
        _require(self.client is not None, 'motion_not_initialized')
        _require(request.get('op') == 'apply', 'unsupported_motion_operation')
        _require(request.get('epoch') == self.epoch and request.get('world_id') == self.world_id, 'motion_identity_mismatch')
        sequence, deadline = request.get('sequence'), request.get('deadline_monotonic')
        _require(_integer(sequence) and sequence == self.last_sequence + 1, 'motion_sequence_mismatch')
        _require(_finite(deadline) and self.clock() <= deadline, 'motion_dispatch_expired')
        rows = request.get('transforms')
        _require(isinstance(rows, list) and 0 < len(rows) <= len(self.allowed), 'invalid_motion_batch')
        ids, commands = [], []
        for row in rows:
            _require(isinstance(row, dict), 'invalid_motion_transform')
            actor_id, values = row.get('actor_id'), row.get('xyz_pitch_yaw_roll')
            _require(_integer(actor_id) and actor_id in self.allowed and actor_id not in ids, 'motion_actor_not_allowed')
            _require(isinstance(values, list) and len(values) == 6 and all(_finite(v) for v in values), 'invalid_motion_transform')
            ids.append(actor_id)
            commands.append(self.carla.command.ApplyTransform(actor_id, self.carla.Transform(
                self.carla.Location(*values[:3]), self.carla.Rotation(pitch=values[3], yaw=values[4], roll=values[5]))))
        # Both checks are immediately before dispatch; get_world may itself wait.
        _require(self.client.get_world().id == self.world_id, 'motion_world_mismatch')
        _require(self.clock() <= deadline, 'motion_dispatch_expired')
        self.last_sequence = sequence
        receipts = self.client.apply_batch_sync(commands, False)
        _require(len(receipts) == len(ids), 'motion_receipt_count_mismatch')
        result = []
        for actor_id, receipt in zip(ids, receipts):
            error = str(receipt.error)
            returned_id = int(receipt.actor_id)
            # CARLA returns actor_id=0 for failed commands, preserving order.
            _require(returned_id == actor_id or (error and returned_id == 0), 'motion_receipt_actor_mismatch')
            result.append(dict(command_actor_id=actor_id, actor_id=returned_id, error=error))
        return dict(schema=SCHEMA, ok=True, op='apply', epoch=self.epoch, world_id=self.world_id,
                    sequence=sequence, receipts=result)


class MotionClient:
    def __init__(self, host, port, world_id, epoch, allowed_actor_ids, stderr, *, timeout=2.):
        self.epoch, self.world_id = epoch, world_id
        allowed_actor_ids = tuple(allowed_actor_ids)
        self.allowed = frozenset(allowed_actor_ids)
        self.timeout = float(timeout)
        self.sequence = 0
        self._pending = threading.Lock()
        self.worker = JsonProcess([sys.executable, '-B', '-m', 'sim_io.motion_rpc'], stderr, timeout=self.timeout)
        try:
            response = self.worker.call(dict(schema=SCHEMA, op='init', host=host, port=port,
                world_id=world_id, epoch=epoch, allowed_actor_ids=list(allowed_actor_ids), rpc_timeout_s=self.timeout))
            self._validate(response, 0)
            if response.get('allowed_actor_ids') != sorted(self.allowed):
                raise RuntimeError('motion_actor_contract_mismatch')
        except BaseException:
            self.close()
            raise

    def _validate(self, response, sequence):
        if not isinstance(response, dict) or response.get('schema') != SCHEMA:
            raise RuntimeError('invalid_motion_response')
        if response.get('ok') is not True:
            raise RuntimeError('motion_rpc: ' + str(response.get('error', 'unknown_error')))
        if (response.get('epoch'), response.get('world_id'), response.get('sequence')) != (self.epoch, self.world_id, sequence):
            raise RuntimeError('motion_response_identity_mismatch')

    def apply_batch_sync(self, commands, do_tick=False):
        if do_tick is not False:
            raise ValueError('motion_world_tick_not_allowed')
        if not self._pending.acquire(blocking=False):
            raise RuntimeError('motion_batch_already_pending')
        try:
            import carla
            rows = []
            for command in commands:
                if not isinstance(command, carla.command.ApplyTransform):
                    raise ValueError('motion_only_apply_transform_allowed')
                actor_id, tr = int(command.actor_id), command.transform
                values = [tr.location.x, tr.location.y, tr.location.z, tr.rotation.pitch, tr.rotation.yaw, tr.rotation.roll]
                if actor_id not in self.allowed or any(row['actor_id'] == actor_id for row in rows):
                    raise ValueError('motion_actor_not_allowed')
                if not all(_finite(v) for v in values):
                    raise ValueError('invalid_motion_transform')
                rows.append(dict(actor_id=actor_id, xyz_pitch_yaw_roll=values))
            if not rows:
                return []
            self.sequence += 1
            response = self.worker.call(dict(schema=SCHEMA, op='apply', epoch=self.epoch, world_id=self.world_id,
                sequence=self.sequence, deadline_monotonic=time.monotonic()+self.timeout, transforms=rows))
            self._validate(response, self.sequence)
            receipts = response.get('receipts')
            if not isinstance(receipts, list) or len(receipts) != len(rows):
                raise RuntimeError('motion_receipt_count_mismatch')
            result = []
            for row, receipt in zip(rows, receipts):
                if receipt.get('command_actor_id') != row['actor_id']:
                    raise RuntimeError('motion_receipt_actor_mismatch')
                actor_id, error = receipt.get('actor_id'), receipt.get('error')
                if (not _integer(actor_id) or not isinstance(error, str) or
                        not (actor_id == row['actor_id'] or (error and actor_id == 0))):
                    raise RuntimeError('invalid_motion_receipt')
                result.append(SimpleNamespace(actor_id=actor_id, error=error))
            return result
        finally:
            self._pending.release()

    def close(self):
        worker, self.worker = getattr(self, 'worker', None), None
        if worker is not None:
            worker.close()


def main():
    import carla
    server = MotionServer(carla)
    for line in sys.stdin:
        request = None
        try:
            _require(len(line) <= 65536, 'motion_request_too_large')
            request = json.loads(line)
            response = server.dispatch(request)
        except Exception as exc:
            response = dict(schema=SCHEMA, ok=False, error=type(exc).__name__+': '+str(exc))
            if isinstance(request, dict):
                for key in ('epoch', 'world_id', 'sequence'):
                    response[key] = request.get(key)
        print(json.dumps(response, allow_nan=False, separators=(',', ':')), flush=True)


if __name__ == '__main__':
    main()
