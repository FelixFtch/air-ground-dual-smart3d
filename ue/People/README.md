# Toni: put these people into the box factory

This folder is the pedestrian pack for a factory level built out of boxes. The characters are the same ones the lab spawns. They are already Unreal assets (meshes, clothes, skin, hair, and the walk animation). Do not run FBX Import on them.

The pack is about 2.1 GB. Pull the latest `main`. Felix Fang published it.

## 1. Check whether you already have them

Open your UE 5.8 project. In the Content Browser search box type:

```
BP_EuroM01_A_G2
```

If that blueprint opens, the people are already in the project. Skip to section 3.

## 2. Copy the pack in

Close the editor first. Unreal locks the files while it is open.

From this `People` folder, copy the inner `Carla` directory into your project's `Content` directory. Merge it with the `Content/Carla` you already have. Do not delete or replace the rest of `Content/Carla` (maps, vehicles, your factory geometry).

After the copy, these four paths must exist:

```
YourProject/Content/Carla/Static/Pedestrian/
YourProject/Content/Carla/Static/GenericMaterials/
YourProject/Content/Carla/Blueprints/Walkers/
YourProject/Content/Carla/Animations/
```

`GenericMaterials` should gain `000_Masters/Pedestrians` and `Pedestrian`. `Animations` should gain `Base`, `GEN2`, `GEN3`, `Depot`, and `Death_Anim`. Leave every folder name as it is. The assets point at each other by these paths. Renaming a folder, or dropping the files into a folder called `People` inside the Unreal project, makes the characters pink or stuck in a T-pose.

Reopen the project and wait until the editor finishes loading the new assets. Search for `BP_EuroM01_A_G2` again. It should appear under `Carla/Blueprints/Walkers`.

Use UE 5.8, the same engine line this pack was saved with.

`BP_Walker` also references CARLA's `BaseVehiclePawn`. A CARLA UE 5.8 project already has that class. A blank Epic project does not, so `BP_Walker` may fail to compile there. Use one of the individual `BP_*` characters instead, or the skeletal mesh in section 4.

## 3. Place a person on the factory floor

1. Open your factory level.
2. Content Browser → `Carla/Blueprints/Walkers`.
3. Drag a blueprint into the viewport. `BP_EuroM01_A_G2` is one clothed man. Every other `BP_*` in that folder is another clothed person (different body and clothes). `BP_Walker` is only the shared base.
4. Unreal units are centimeters. Stand the actor on the floor of your box geometry. A standing person is about 180 cm tall, so the feet, not the origin gizmo, should meet the floor. If the actor sinks or floats, move it on Z until the shoes touch the ground.
5. Rotate the actor on yaw so the face points along the direction you want them to face. The mesh is about 40 cm across the shoulders.
6. Save the level.

Repeat the drag for each extra person. Each dragged blueprint is its own actor.

## 4. Mesh only, no blueprint

If the blueprint will not drop in, drag the skeletal mesh instead:

```
Carla/Static/Pedestrian/EuroM01_G2/SK_EuroM01_A_G2
```

Other names follow the same pattern: `Carla/Static/Pedestrian/<PersonName>/SK_<PersonName>...`.

A mesh placed this way is a visible person. It does not carry the walker blueprint.

## 5. What you should see

Press Play. A character that is only placed, with no speed, stays in the idle pose. The walk cycle is `Carla/Animations/GEN2/ABP_GEN2_vA`. It plays when that actor is actually moving.

Pink materials mean a folder was renamed or the `GenericMaterials` copy did not land next to `Pedestrian`. A T-pose means `Carla/Animations` did not land, or the skeleton under `Carla/Static/Pedestrian/00_GenericComponents/Definitions` is missing. Copy those paths again without renaming, then restart the editor.

## Do not

- Do not import this pack through Add → Import Into Level, or through FBX.
- Do not move the assets to `Content/People` inside the Unreal project. On disk in the git repo the pack is called `People`. Inside Unreal the path has to stay `Content/Carla/...`.
- Do not replace your whole `Content/Carla` tree with this folder. Merge the four directories listed above.
