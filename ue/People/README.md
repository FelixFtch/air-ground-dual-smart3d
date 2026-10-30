# People for a box-built factory scene

These are the pedestrian characters the lab already spawns (`walker.pedestrian.*`). They are Unreal assets, not FBX. Meshes, clothes, skin, hair, and the walk animation are included, about 2.1 GB.

Keep every folder name. The assets point at each other by path. Renaming or flattening the tree makes the characters pink or T-pose.

## If your project is already CARLA UE 5.8

The same files are usually already in the CARLA project, under `Content/Carla/...`. Search the Content Browser for `BP_EuroM01_A_G2`. If it opens, do not copy this folder. Skip to "Place a person".

## If your factory project does not have them

1. Use UE 5.8, the same engine this CARLA project was built with.
2. Close the editor.
3. From this folder, copy the inner `Carla` directory into your project's `Content` directory, so the result is:

```
YourProject/Content/Carla/Static/Pedestrian/...
YourProject/Content/Carla/Static/GenericMaterials/...
YourProject/Content/Carla/Blueprints/Walkers/...
YourProject/Content/Carla/Animations/...
```

Merge with the `Content/Carla` you already have. Do not replace the whole `Content/Carla` tree if you already have maps and vehicles there. Copy only the four directories above.

4. Reopen the project and let the editor finish loading the new assets.

`BP_Walker` also refers to CARLA's `BaseVehiclePawn`. That class is already in a CARLA project. In a blank UE project that parent is missing, so `BP_Walker` may not compile. The individual `BP_*` characters and the `SK_*` meshes still drop into the level.

## Place a person

1. In the Content Browser open `Carla/Blueprints/Walkers`.
2. Drag one blueprint into the level, for example `BP_EuroM01_A_G2`. Any `BP_*` there is one clothed person. `BP_Walker` is the shared base.
3. Unreal units are centimeters. Put the feet on the floor of your box geometry. A standing person is about 180 cm tall.
4. The skeletal mesh is the thin box in walking direction (the camera only sees the front of the body) and about 40 cm across the shoulders. Rotate the actor so the face points along the path you want.
5. Press Play. The animation blueprint `ABP_GEN2_vA` plays the walk cycle when the actor moves. A character that is only placed, and never given a speed, stands in the idle pose.

To place the mesh without the blueprint, drag `SK_*` from `Carla/Static/Pedestrian/<name>/`, for example `Carla/Static/Pedestrian/EuroM01_G2/SK_EuroM01_A_G2`.

## What not to do

Do not import these through FBX. They are already Unreal assets. Do not move them to a folder named `People` inside the Unreal project. The Content path has to stay `Content/Carla/...`.
