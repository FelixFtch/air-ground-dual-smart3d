"""./lab people: one people back end at a time, choice remembered for the next start."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]


def lifecycle(tmp_path, running):
    spec = importlib.util.spec_from_file_location('lifecycle_under_test', ROOT / 'simulation/lifecycle.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.PEOPLE_CHOICE = tmp_path / 'people_backend.json'
    calls = []
    module.active = lambda name: name in running
    module.subprocess = SimpleNamespace(run=lambda command, **kwargs: calls.append(command))
    return module, calls


def test_default_is_sam3_when_nothing_was_chosen(tmp_path):
    module, _ = lifecycle(tmp_path, set())
    assert module.people_backend() == 'sam3'


def test_switch_stops_the_running_backend_before_starting_the_other(tmp_path):
    running = {'air-ground-lab-runtime', 'air-ground-lab-rgbd-capture', 'air-ground-lab-sam3'}
    module, calls = lifecycle(tmp_path, running)
    module.choose_people('yolo26')
    assert json.loads(module.PEOPLE_CHOICE.read_text()) == {'backend': 'yolo26'}
    assert calls[0] == ['systemctl', '--user', 'stop', 'air-ground-lab-sam3.service']
    started = calls[1]
    assert '--unit=air-ground-lab-yolo26' in started
    assert started[started.index('perception.service') + 1:][:2] == ['--backend', 'yolo26']
    assert len(calls) == 2


def test_choice_without_a_running_experiment_only_records_it(tmp_path):
    module, calls = lifecycle(tmp_path, set())
    module.choose_people('yolo26')
    assert calls == [] and module.people_backend() == 'yolo26'
