# Project-local JSON transport acceleration

The map and native planner exchange large coordinate arrays. `sim_io/json_codec.py`
provides native encoding/decoding for these locally generated wire records, with
stdlib fallback when the isolated dependency is absent or incompatible.

## Current isolated dependency

- UltraJSON (`ujson`) **5.12.1**, CPython 3.10, Linux x86-64.
- File: `runtime/python_deps/ujson.cpython-310-x86_64-linux-gnu.so`.
- SHA-256: `2967cb08d9454085e9ed947e4d4598a59d50c817d383b8dc0c2d5fa4737eae3b`.
- Copied from the existing local installation; the binary checksum was verified
  against that installation's `dist-info/RECORD` before copying.
- Complete distribution metadata and BSD-3-Clause/TCL license are retained at
  `runtime/python_deps/ujson-5.12.1.dist-info/`.
- `runtime/python_deps/ujson.provenance.json` records the source and checksum.
- Runtime assets remain ignored by Git. No shared environment, user-site package,
  `PYTHONPATH`, Factory configuration or original single-robot project was changed.

The loader uses an explicit filename matching the running interpreter ABI and
checks its checksum and version. It never modifies `sys.path`. Missing files,
checksum mismatch or another Python ABI select the stdlib path. Setting
`AIR_GROUND_JSON_CODEC=stdlib` for one process selects the comparison baseline.
`backend_info()` reports which implementation was actually loaded.

## Contract and limits

Fast encoding is for **string-keyed dictionaries**, lists/tuples and JSON scalar
values. Tuples become arrays. `allow_nan=False` is the default: NaN and infinity
raise `ValueError`, including nested values; they are never converted silently to
null. Bytes and unsupported objects raise normally. Custom encoders, default
callbacks, parse hooks and other general stdlib options use stdlib instead.

This is **not a validator for arbitrary untrusted JSON**. UltraJSON accepts some
nonstandard syntax and converts exotic dictionary keys differently from stdlib;
those are outside the generated map/planner/state protocol. Keep public input
validation and canonical-hash generation on stdlib. Float exponent whitespace can
differ while decoded values are the same. Parse failures are re-run through
stdlib to retain `JSONDecodeError` and its diagnostic position.

## Reinstall without modifying a shared environment

If the same local verified installation is available, copying its binary,
distribution metadata and receipt reproduces the installation above. Otherwise,
install the pinned wheel into this project's ignored target only (this operation
may need network access):

```bash
/data/robotics/ue58_ohz_test_go2/.venv/bin/python -m pip install \
  --only-binary=:all: --no-deps --target \
  /data/robotics/air_ground_smart3d/runtime/python_deps ujson==5.12.1
```

Then create an installation receipt for the ABI-matched wheel. Verify the binary
against its wheel RECORD before enabling it:

```python
from pathlib import Path
import base64, csv, hashlib, json, sysconfig

folder = Path('/data/robotics/air_ground_smart3d/runtime/python_deps')
name = 'ujson' + sysconfig.get_config_var('EXT_SUFFIX')
binary = folder / name
metadata = folder / 'ujson-5.12.1.dist-info'
rows = list(csv.reader((metadata / 'RECORD').open()))
record = next(row for row in rows if row[0] == name)
digest = hashlib.sha256(binary.read_bytes()).digest()
assert record[1] == 'sha256=' + base64.urlsafe_b64encode(digest).decode().rstrip('=')
receipt = dict(schema='air_ground.local_python_dependency/v1', name='ujson',
               version='5.12.1', binary=name, sha256=digest.hex(),
               source='pinned wheel; binary verified against dist-info/RECORD',
               license='BSD-3-Clause AND TCL',
               license_path='ujson-5.12.1.dist-info/licenses/LICENSE.txt')
(folder / 'ujson.provenance.json').write_text(json.dumps(receipt, indent=2) + '\n')
```

Use the interpreter that will run the service for the receipt step. A compatible
wheel for another interpreter has a different binary checksum; the copied
CPython 3.10 extension must not be loaded into another ABI. Normal startup remains
available through stdlib until a compatible isolated dependency is ready.

## Verification

`tests/test_json_codec.py` exercises both configured and stdlib paths: finite
float roundtrips, Unicode, tuple/list conversion, nested NaN/Inf rejection,
malformed JSON diagnostics, hooks, bytes, UTF-16/BOM input, absent dependency and
checksum failure. The source-only checkout can run through stdlib fallback.

Offline benchmark evidence lives at `/tmp/maphot/json_codec_benchmark.json` for
this session. It compares actual state/display snapshots and a 500,000-coordinate
refresh message, with GC enabled and decoded content equality checked. This is a
codec benchmark, not proof of the final live mapping frequency.

## Relation to the live fusion pipeline

The codec changes serialization cost, not sensor evidence, occupancy rules or
execution freshness. Lossless packed refresh coordinates further reduce the
number of temporary Python containers. The current bridge additionally overlaps
one next native request with one previous ordered cache application; it never
queues multiple cache applies. Capture metadata is bound to each frame, and
execution freshness advances only after that frame's cache commit succeeds.

Codec-only timings above and offline pipeline throughput are separate from live
per-source fusion rates. See the [OctoMap real-time verification
report](verification/2026-09-29-octomap-realtime.md) for the current configuration,
measured rates, delays and remaining limitations.
