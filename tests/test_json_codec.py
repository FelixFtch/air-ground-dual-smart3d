import importlib
import json
import math
from decimal import Decimal

import pytest

from sim_io import json_codec as codec


@pytest.fixture(params=['configured', 'fallback'])
def wire(request, monkeypatch):
    if request.param == 'fallback':
        monkeypatch.setattr(codec, '_fast', None)
    return codec


def test_wire_records_roundtrip_identical_to_stdlib(wire):
    doc = {'scene': '工厂/机器人', 'emoji': '🐕', 'text': '<script>é</script>\n',
           'cells': [{'key': (-11, 0, 100), 'state': 'free', 'stamp': 123.45678901234567}],
           'mixed': [None, True, False, -0.0, 1e-30, 1e30, 10**100]}
    encoded = wire.dumps(doc, allow_nan=False, separators=(',', ':'), ensure_ascii=False)
    assert isinstance(encoded, str)
    assert wire.loads(encoded) == json.loads(json.dumps(doc, allow_nan=False))
    assert json.loads(encoded) == wire.loads(json.dumps(doc))
    assert '工厂/机器人' in encoded


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_values_fail_loudly_at_any_depth(wire, value):
    for obj in [value, {'a': [1, {'x': value}]}, (0, value)]:
        with pytest.raises(ValueError):
            wire.dumps(obj, allow_nan=False)
        with pytest.raises(ValueError):
            wire.dumps(obj)


def test_no_nan_to_null_even_when_explicitly_allowed(wire):
    encoded = wire.dumps({'x': math.nan}, allow_nan=True)
    assert 'null' not in encoded
    assert math.isnan(wire.loads(encoded)['x'])


def test_finite_float_roundtrip_is_exact(wire):
    values = [math.nextafter(0., 1.), math.nextafter(1., 2.), math.pi,
              1.7976931348623157e308, -1.7976931348623157e308, .1, -.0]
    result = wire.loads(wire.dumps(values, allow_nan=False))
    assert result == values
    assert math.copysign(1, result[-1]) == -1


@pytest.mark.parametrize('encoded', ['{', '[1,]', '{"a":}', '{"a":1,}', 'no JSON'])
def test_malformed_native_errors_use_stdlib_jsondecodeerror(wire, encoded):
    with pytest.raises(json.JSONDecodeError):
        wire.loads(encoded)


def test_stdlib_custom_hooks_and_default_encoder_are_retained(wire):
    assert wire.loads('{"x":1.2}', parse_float=Decimal) == {'x': Decimal('1.2')}
    assert wire.loads('{"x":1}', object_hook=lambda o: list(o)) == ['x']
    assert wire.dumps({'x': Decimal('1.2')}, default=str) == '{"x": "1.2"}'


def test_binary_invalid_payload_and_utf16_behavior(wire):
    with pytest.raises(TypeError):
        wire.dumps({'x': b'bytes'})
    assert wire.loads('{"x":"工厂"}'.encode('utf-16')) == {'x': '工厂'}
    assert wire.loads(b'\xef\xbb\xbf{"x":1}') == {'x': 1}


def test_strict_nonfinite_dict_keys_use_general_stdlib_option_path(wire):
    # Exotic keys are outside the optimized wire contract; an explicit stdlib
    # option chooses the general path when needed.
    with pytest.raises(ValueError):
        wire.dumps({math.nan: 'bad'}, allow_nan=False, check_circular=True)


def test_missing_dependency_falls_back_without_changing_sys_path(monkeypatch, tmp_path):
    import sys
    before = list(sys.path)
    monkeypatch.setattr(codec, 'DEPENDENCY_ROOT', tmp_path)
    module, status = codec._load_isolated()
    assert module is None and status['backend'] == 'stdlib'
    assert sys.path == before


def test_checksum_mismatch_does_not_load_binary(monkeypatch, tmp_path):
    import sysconfig
    filename = 'ujson' + sysconfig.get_config_var('EXT_SUFFIX')
    (tmp_path / filename).write_bytes(b'not executable')
    (tmp_path / 'ujson.provenance.json').write_text(json.dumps(dict(name='ujson', version='5.12.1', binary=filename, sha256='bad')))
    monkeypatch.setattr(codec, 'DEPENDENCY_ROOT', tmp_path)
    module, status = codec._load_isolated()
    assert module is None and 'checksum mismatch' in status['reason']
