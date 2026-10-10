"""Enrich AirGroundLab_Main with red/blue/yellow walls (codex prepare_scene.py pattern).

Layout (map frame, CARLA y = -map_y): broken outer ring at |x| or |y| = 14,
two 20x20 rooms with door gaps centered (-20,-20)/(20,20), four pillars at
(|12|,0)/(0,|12|), two overhead beams z>=3.4m (above UAV band). Everything
outside the +/-10m patrol corridor; rollback = restore map backup.
"""
import hashlib
import json
from pathlib import Path
import traceback
import unreal

ROOT = Path('/data/robotics/air_ground_smart3d')
TARGET = '/Game/AirGroundLab/Maps/AirGroundLab_Main'
MATERIALS = {
    'red': '/Game/OHZTest/Materials/MI_OHZ_Red',
    'blue': '/Game/OHZTest/Materials/MI_OHZ_Blue',
    'yellow': '/Game/OHZTest/Materials/MI_OHZ_Yellow',
}
report = {'target': TARGET, 'added': [], 'errors': [], 'saved': False}


def close(a, b, tolerance=.01):
    return max(abs(x - y) for x, y in zip(a, b)) <= tolerance


def wall(name, x, y, z, sx, sy, sz, material):
    """Cube actor: (x,y)=map meters -> carla cm (x, -y); sx/sy/sz = scale in meters."""
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actor = actors.spawn_actor_from_class(unreal.StaticMeshActor,
                                          unreal.Vector(x * 100, -y * 100, z * 100))
    actor.set_actor_label('RICH_' + name)
    actor.tags = ['AirGroundLabOwned']
    comp = actor.static_mesh_component
    comp.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Cube'))
    comp.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    comp.set_collision_profile_name('BlockAll')
    comp.set_material(0, unreal.load_asset(MATERIALS[material]))
    actor.set_actor_scale3d(unreal.Vector(sx, sy, sz))
    loc = unreal.Vector(x * 100, -y * 100, z * 100)
    actor.set_actor_location(loc, False, False)
    actual = actor.get_actor_location()
    assert close([actual.x, actual.y, actual.z], [loc.x, loc.y, loc.z]), 'placement: ' + name
    report['added'].append({'name': name, 'map_xyz_m': [x, y, z], 'size_m': [sx, sy, sz],
                            'material': material})


try:
    world = unreal.EditorLoadingAndSavingUtils.load_map(TARGET)
    assert world and world.get_path_name().split('.')[0] == TARGET, 'target world not active'
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    for existing in actors.get_all_level_actors():
        if existing.get_actor_label().startswith('RICH_'):
            actors.destroy_actor(existing)
    H, T = 2.6, 0.4          # 墙高/厚
    # ① 外圈断续回廊(|坐标|=14,四角留 6m 门)
    wall('ring_N_a', -8, 14, H / 2, 12, T, H, 'red');  wall('ring_N_b', 8, 14, H / 2, 12, T, H, 'red')
    wall('ring_S_a', -8, -14, H / 2, 12, T, H, 'red'); wall('ring_S_b', 8, -14, H / 2, 12, T, H, 'red')
    wall('ring_E_a', 14, -8, H / 2, T, 12, H, 'blue'); wall('ring_E_b', 14, 8, H / 2, T, 12, H, 'blue')
    wall('ring_W_a', -14, -8, H / 2, T, 12, H, 'blue');wall('ring_W_b', -14, 8, H / 2, T, 12, H, 'blue')
    # ② 两个房间(±20,±20),20x20,每边留 4m 门
    for cx, cy, mat in ((-20, -20, 'yellow'), (20, 20, 'yellow')):
        for side in ('N', 'S', 'E', 'W'):
            gap = 4
            if side in ('N', 'S'):
                sy, sy2 = cy + 10, cy - 10
                wall(f'room{cx}_{side}L', cx - 6, sy if side == 'N' else sy2, H / 2, 8, T, H, mat)
                wall(f'room{cx}_{side}R', cx + 6, sy if side == 'N' else sy2, H / 2, 8, T, H, mat)
            else:
                sx, sx2 = cx + 10, cx - 10
                wall(f'room{cx}_{side}L', sx if side == 'E' else sx2, cy - 6, H / 2, T, 8, H, mat)
                wall(f'room{cx}_{side}R', sx if side == 'E' else sx2, cy + 6, H / 2, T, 8, H, mat)
    # 房间内隔断(错开门缝)
    wall('roomNW_div', -20, -20, H / 2, T, 12, H, 'red')
    wall('roomSE_div', 20, 20, H / 2, T, 12, H, 'blue')
    # ③ 四根立柱
    for x, y, mat in ((12, 12, 'red'), (-12, 12, 'blue'), (12, -12, 'blue'), (-12, -12, 'red')):
        wall(f'pillar_{x}_{y}', x, y, H / 2, 1.2, 1.2, H, mat)
    # ④ 高空横梁(UAV 上界 3.2m,梁底 3.4m)
    wall('beam_NS', 0, 0, 5.0, 0.6, 16, 0.4, 'blue')
    wall('beam_EW', -6, 6, 5.0, 16, 0.6, 0.4, 'red')
    assert unreal.EditorLoadingAndSavingUtils.save_current_level(), 'save failed'
    report['saved'] = True
    # 保存后重载验证
    assert unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    world = unreal.EditorLoadingAndSavingUtils.load_map(TARGET)
    labels = {a.get_actor_label() for a in unreal.get_editor_subsystem(
        unreal.EditorActorSubsystem).get_all_level_actors() if a.get_actor_label().startswith('RICH_')}
    expected = {item['name'] for item in report['added']}
    assert labels == expected, f'persisted set differs: {len(labels)} vs {len(expected)}'
    report['persisted'] = sorted(labels)
    mapfile = Path('/data/sim/AirGroundLab_UE58/CarlaUnreal/Content/AirGroundLab/Maps/AirGroundLab_Main.umap')
    report['file_bytes'] = mapfile.stat().st_size
except Exception:
    report['errors'].append(traceback.format_exc())
finally:
    (ROOT / 'runtime/scene_enrichment.json').write_text(json.dumps(report, indent=2) + '\n')
    unreal.log('AIR_GROUND_SCENE_ENRICHED ' + str(report))
    unreal.SystemLibrary.quit_editor()
