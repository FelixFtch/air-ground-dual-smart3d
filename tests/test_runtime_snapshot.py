import random
import json
import pytest
from observability.runtime_snapshot import MapDisplayIndex
from observability.projection import body_height_projection

def test_restored_and_incremental_display_matches_authoritative_cells():
    rng=random.Random(67);cells={(x,y,z):{'state':rng.choice(('free','occupied')),'last_observed_sim_time':None} for x in range(10) for y in range(8) for z in range(-2,7)}
    index=MapDisplayIndex(cells,.25,.33,.28)
    for _ in range(80):
        changes=[]
        for _ in range(25):
            key=(rng.randrange(12),rng.randrange(10),rng.randrange(-2,7));state=rng.choice(('free','occupied','unknown'));changes.append({'key':list(key),'state':state})
            if state=='unknown':cells.pop(key,None)
            else:cells[key]={'state':state}
        index.apply(cells,changes)
        free,blocked,occupied=index.snapshot();expected=body_height_projection(cells,.25,.33,.28)
        assert set(map(tuple,free))==set(map(tuple,expected[0]))
        assert set(map(tuple,blocked))==set(map(tuple,expected[1]))
        assert set(map(tuple,occupied))=={k for k,v in cells.items() if v['state']=='occupied'}

def test_snapshot_updates_never_scan_full_map_and_return_immutable_versions():
    class NoScan(dict):
        def items(self):raise AssertionError('full map scan')
    cells=NoScan({(0,0,1):{'state':'occupied'}});index=MapDisplayIndex({},.25,.33,.28)
    index.apply(cells,[{'key':[0,0,1],'state':'occupied'}]);first=index.snapshot()
    with pytest.raises(TypeError):first[2][0][0]=9
    assert first[2]==((0,0,1),)
    index.apply(cells,[])
    assert all(a is b for a,b in zip(first,index.snapshot()))
    cells[(1,0,1)]={'state':'occupied'}
    index.apply(cells,[{'key':[1,0,1],'state':'occupied'}])
    assert first[2]==((0,0,1),)  # Earlier publication remains an immutable version.
    assert set(index.snapshot()[2])=={(0,0,1),(1,0,1)}


def test_geometry_equivalent_json_and_refreshes_reuse_snapshots():
    cells={(0,0,z):{'state':'free'} for z in (0,1,2)}
    cells[(1,0,1)]={'state':'occupied'}
    index=MapDisplayIndex(cells,.25,.33,.28)
    first=index.snapshot();encoded=json.dumps(first)
    mutable=([list(k) for k in first[0]],[list(k) for k in first[1]],[list(k) for k in first[2]])
    assert encoded==json.dumps(mutable)
    for cell in cells.values():cell['last_observed_sim_time']=100.
    # Even a duplicate changed-cell row with unchanged geometry must not allocate
    # all coordinate lists again.
    index.apply(cells,[{'key':[1,0,1],'state':'occupied'}])
    assert all(a is b for a,b in zip(first,index.snapshot()))
    cells[(1,0,1)]={'state':'free'}
    transitions=index.apply(cells,[{'key':[1,0,1],'state':'free'}])
    assert transitions=={'new_occupied':0,'cleared_occupied':1}
    assert index.snapshot()[2]==()
    assert json.dumps(first)==encoded
