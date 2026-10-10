"""YOLO26 people back end without weights: result contract, identities, service outputs."""
import hashlib
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

import numpy as np
import pytest

from perception.service import PeopleService
from perception.yolo26_backend import Yolo26People
from test_people_perception import frame, metadata


def offline_backend(shift_per_frame=.2):
    backend = Yolo26People.__new__(Yolo26People)
    backend.sessions = {}
    backend.identity = {'name': 'ultralytics/yolo26m-seg', 'tracking_ids': 'map_frame_association/v1'}
    backend.detect = lambda rgb: ([[0., 0., 39., 39.]], [.9], np.ones((1, 40, 40), bool))
    return backend


def moving_frame(i, robot='go2'):
    capture = frame(metadata(robot, stamp=10. + .35 * i, frame=i))
    capture['xyz_optical'][:, :, 0] += .35 * i
    return capture


def test_identity_persists_and_result_matches_the_sam3_contract():
    backend = offline_backend()
    key = ('epoch', 'go2', 'front_rgbd')
    results = [backend.infer(key, moving_frame(i)['rgb'], moving_frame(i)) for i in range(5)]
    assert {r['tracks'][0]['id'] for r in results} == {0}
    assert len({r['tracking_session_id'] for r in results}) == 1
    assert [r['session_frame_index'] for r in results] == list(range(5))
    last = results[-1]
    assert last['masks'].shape == (1, 40, 40) and last['masks'].dtype == bool
    assert set(last['tracks'][0]) == {'id', 'box', 'score'}
    assert last['lifecycle']['visible_ids'] == [0]


def test_reset_starts_a_new_session():
    backend = offline_backend()
    key = ('epoch', 'go2', 'front_rgbd')
    first = backend.infer(key, moving_frame(0)['rgb'], moving_frame(0))['tracking_session_id']
    backend.reset(key, 'camera_calibration_changed')
    second = backend.infer(key, moving_frame(1)['rgb'], moving_frame(1))
    assert second['tracking_session_id'] != first
    assert second['reset_reason'] == 'camera_calibration_changed'


def test_weights_with_a_different_hash_are_refused_before_loading():
    with tempfile.TemporaryDirectory() as folder:
        weights = Path(folder) / 'yolo26m-seg.pt'
        weights.write_bytes(b'not the reviewed weights')
        record = dict(model='yolo26m-seg', path=str(weights), bytes=weights.stat().st_size, sha256='0' * 64)
        provenance = Path(folder) / 'provenance.json'
        provenance.write_text(json.dumps(record))
        with pytest.raises(ValueError, match='hash_mismatch'):
            Yolo26People(provenance=provenance)


def test_service_runs_yolo26_into_its_own_outputs_and_shared_people():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        (root / 'resolved_profiles.json').write_text(json.dumps(dict(robots=[dict(robot_id='go2')])))
        (root / 'state.json').write_text(json.dumps(dict(scene_id='scene', epoch='epoch', sim_time=10.,
                                                          wall_time=1000., running=True)))
        (root / 'go2/rgbd').mkdir(parents=True)
        (root / 'go2/rgbd/latest.json').write_text(json.dumps(metadata()))
        backend = offline_backend()
        service = PeopleService(root, backend, wall=lambda: 1000., name='yolo26')
        with patch('perception.registered_rgbd.load_capture', return_value=frame()):
            assert service.step()
        status = json.loads((root / 'go2/yolo26/latest.json').read_text())
        image = (root / 'go2/yolo26/latest.jpg').read_bytes()
        assert status['overlay_sha256'] == hashlib.sha256(image).hexdigest()
        assert status['schema'] == 'air_ground.yolo26_source/v1'
        assert not (root / 'go2/sam3').exists()
        people = json.loads((root / 'shared/people.json').read_text())
        assert people['perception_backend'] == 'yolo26' and len(people['people']) == 1
        assert json.loads((root / 'shared/yolo26_status.json').read_text())['model_identity']['name'] == 'ultralytics/yolo26m-seg'


def test_unknown_backend_name_is_rejected():
    with pytest.raises(ValueError, match='unknown_people_backend'):
        PeopleService('/nonexistent', offline_backend(), name='yolo11')
