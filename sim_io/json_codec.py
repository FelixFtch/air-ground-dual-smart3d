"""Fast JSON for locally generated canonical simulator/map wire records.

The wire contract is string-keyed dictionaries, lists/tuples and JSON scalars.
Encoding rejects nonfinite float values. This is not a general-purpose validator
for untrusted JSON or exotic Python dictionary keys: UltraJSON has different
permissiveness there. Custom stdlib hooks/options use the stdlib path. Dependency
loading is explicit and project-local; missing/incompatible files fall back to
stdlib without changing PYTHONPATH or any shared Python installation.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json as _stdlib
import os
from pathlib import Path
import sysconfig

JSONDecodeError = _stdlib.JSONDecodeError
PINNED_VERSION = '5.12.1'
DEPENDENCY_ROOT = Path(__file__).resolve().parents[1] / 'runtime/python_deps'


def _load_isolated():
    if os.environ.get('AIR_GROUND_JSON_CODEC', '').lower() == 'stdlib':
        return None, {'backend': 'stdlib', 'reason': 'explicit process override'}
    try:
        receipt = _stdlib.loads((DEPENDENCY_ROOT / 'ujson.provenance.json').read_text())
        filename = 'ujson' + sysconfig.get_config_var('EXT_SUFFIX')
        if receipt['name'] != 'ujson' or receipt['version'] != PINNED_VERSION or receipt['binary'] != filename:
            raise ValueError('dependency version or Python ABI does not match')
        path = DEPENDENCY_ROOT / filename
        if hashlib.sha256(path.read_bytes()).hexdigest() != receipt['sha256']:
            raise ValueError('dependency checksum mismatch')
        spec = importlib.util.spec_from_file_location('ujson', path)
        if spec is None or spec.loader is None:
            raise ImportError('extension loader unavailable')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        if module.__version__ != PINNED_VERSION:
            raise ValueError('loaded dependency version does not match')
        return module, {'backend': 'ujson', 'version': PINNED_VERSION,
                        'path': str(path), 'sha256': receipt['sha256']}
    except (OSError, ValueError, KeyError, ImportError, TypeError, AttributeError) as exc:
        return None, {'backend': 'stdlib', 'reason': type(exc).__name__ + ': ' + str(exc)}


_fast, _backend = _load_isolated()


def backend_info():
    return dict(_backend)


def dumps(obj, *, allow_nan=False, **kwargs):
    """Return a JSON str; default and explicit allow_nan=False reject NaN/Inf.

    Fast path accepts the compact protocol options: ensure_ascii, sort_keys,
    indent and separators. Custom encoders, default callbacks and other stdlib
    options retain stdlib semantics via fallback. Semantically equivalent float
    exponent formatting may differ; this codec is not used for canonical hashes.
    """
    options = {'ensure_ascii', 'sort_keys', 'indent', 'separators'}
    if _fast is None or any(key not in options for key in kwargs):
        return _stdlib.dumps(obj, allow_nan=allow_nan, **kwargs)
    args = dict(kwargs, allow_nan=allow_nan, reject_bytes=True,
                escape_forward_slashes=False)
    try:
        return _fast.dumps(obj, **args)
    except (OverflowError, TypeError, ValueError):
        # Nonfinite floats become stdlib ValueError, never silently null. Also
        # preserve normal error types for unsupported bytes/objects/cycles.
        return _stdlib.dumps(obj, allow_nan=allow_nan, **kwargs)


def loads(data, **kwargs):
    """Decode canonical locally generated JSON; stdlib hooks keep their behavior.

    On parse failure stdlib supplies its familiar JSONDecodeError including
    position. Non-UTF8 byte encodings/BOMs go to stdlib. Do not use the native
    permissive parser to validate arbitrary user-supplied JSON syntax.
    """
    if _fast is None or kwargs or not isinstance(data, (str, bytes, bytearray)):
        return _stdlib.loads(data, **kwargs)
    if isinstance(data, (bytes, bytearray)) and (b'\x00' in data[:4] or data.startswith((b'\xef\xbb\xbf', b'\xff\xfe', b'\xfe\xff'))):
        return _stdlib.loads(data, **kwargs)
    try:
        return _fast.loads(data)
    except (ValueError, TypeError, OverflowError):
        return _stdlib.loads(data, **kwargs)
