"""Prepare, persist and reload-verify only the isolated AirGroundLab map."""
import hashlib
import json
from pathlib import Path
import traceback
import unreal

ROOT = Path('/data/robotics/air_ground_smart3d')
PROJECT = Path('/data/sim/AirGroundLab_UE58/CarlaUnreal')
TARGET = '/Game/AirGroundLab/Maps/AirGroundLab_Main'
SOURCE = '/Game/OHZTest/Maps/OHZTest_Main'
report = {'target': TARGET, 'obstacles': [], 'errors': [], 'saved': False, 'reload_verified': False}


def close(a, b, tolerance=.01):
    return max(abs(x-y) for x, y in zip(a, b)) <= tolerance


try:
    unreal.EditorAssetLibrary.make_directory('/Game/AirGroundLab/Maps')
    # SaveMap(source, new_name) writes a copy but does NOT switch the active
    # editor world. Load that saved copy before adding actors or saving again.
    if not unreal.EditorAssetLibrary.does_asset_exist(TARGET):
        source_world = unreal.EditorLoadingAndSavingUtils.load_map(SOURCE)
        assert source_world, 'source map not loaded'
        assert unreal.EditorLoadingAndSavingUtils.save_map(source_world, TARGET), 'initial independent map copy failed'
        source_world = None
    world = unreal.EditorLoadingAndSavingUtils.load_map(TARGET)
    assert world and world.get_path_name().split('.')[0] == TARGET, 'independent target world not active'
    report['edited_world'] = world.get_path_name()
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    for existing in actors.get_all_level_actors():
        if existing.get_actor_label().startswith('AG_'):
            actors.destroy_actor(existing)
    existing = None
    cube = unreal.load_asset('/Engine/BasicShapes/Cube')
    assert cube, 'engine cube unavailable'
    obstacles = [
        ('AG_LowWall', [0, 300, 150], [.45, 6.0, 2.6], '/Game/OHZTest/Materials/MI_OHZ_Red'),
        ('AG_OverheadBeam', [800, -400, 320], [6.0, .65, .4], '/Game/OHZTest/Materials/MI_OHZ_Blue'),
        ('AG_Box', [-200, -300, 70], [1.0, 1.0, 1.0], '/Game/OHZTest/Materials/MI_OHZ_Yellow'),
    ]
    for name, location_cm, dimensions_m, material in obstacles:
        actor = actors.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(*location_cm))
        assert actor, 'actor spawn failed: ' + name
        actor.set_actor_label(name)
        actor.tags = ['AirGroundLabOwned']
        comp = actor.static_mesh_component
        comp.set_static_mesh(cube)
        comp.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
        comp.set_collision_profile_name('BlockAll')
        mat = unreal.load_asset(material)
        assert mat, 'test material unavailable: ' + material
        comp.set_material(0, mat)
        actor.set_actor_scale3d(unreal.Vector(*dimensions_m))
        actor.set_actor_location(unreal.Vector(*location_cm), False, False)
        actual = actor.get_actor_location()
        actual_cm = [actual.x, actual.y, actual.z]
        assert close(actual_cm, location_cm), 'unexpected actor placement: ' + name
        report['obstacles'].append({'name': name, 'carla_center_m': [v/100 for v in actual_cm], 'size_m': dimensions_m})
    actor = comp = None
    assert unreal.EditorLoadingAndSavingUtils.save_current_level(), 'save current target map failed'
    report['saved'] = True
    world = None
    # A successful save return is not acceptance: drop the world, reload its
    # disk package, then verify that the expected actors actually persisted.
    assert unreal.EditorLoadingAndSavingUtils.new_blank_map(False), 'could not unload target map for verification'
    world = unreal.EditorLoadingAndSavingUtils.load_map(TARGET)
    assert world and world.get_path_name().split('.')[0] == TARGET
    actual_actors = {a.get_actor_label(): a for a in actors.get_all_level_actors()
                     if a.get_actor_label().startswith('AG_')}
    assert set(actual_actors) == {r[0] for r in obstacles}, 'persisted actor names differ'
    for name, location_cm, dimensions_m, _ in obstacles:
        actor = actual_actors[name]
        location = actor.get_actor_location()
        scale = actor.get_actor_scale3d()
        assert close([location.x, location.y, location.z], location_cm), 'persisted location differs: ' + name
        assert close([scale.x, scale.y, scale.z], dimensions_m), 'persisted scale differs: ' + name
        assert actor.static_mesh_component.get_collision_profile_name() == 'BlockAll'
    report['reload_verified'] = True
    mapfile = PROJECT/'Content/AirGroundLab/Maps/AirGroundLab_Main.umap'
    report['file_bytes'] = mapfile.stat().st_size
    report['sha256'] = hashlib.sha256(mapfile.read_bytes()).hexdigest()
except Exception:
    report['errors'].append(traceback.format_exc())
finally:
    (ROOT/'runtime/scene_preparation.json').write_text(json.dumps(report, indent=2) + '\n')
    unreal.log('AIR_GROUND_SCENE_PREPARED ' + str(report))
    unreal.SystemLibrary.quit_editor()
