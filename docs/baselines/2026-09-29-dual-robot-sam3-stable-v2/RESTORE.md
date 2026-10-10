# Using this configuration backup

This is an archive of the dual-robot isolated experiment's configuration, source, and native binaries compatible with this machine; it is not a backup of the full UE engine, commercial models, or SAM3 weights. Town10, Factory, and the original single-robot empty-scene configurations are not in this package.

The current local run needs no restore or restart. Normal local launch:

```bash
cd /data/robotics/air_ground_smart3d
./lab start --rviz
./lab status
# Only when you need to stop:
./lab stop
```

The launch order is managed by lab: isolated UE/CARLA 2400 → shared map, dual-robot planning and control → LiDAR display, ROS domain45/RViz → RGB-D and SAM3. The default starts from fresh observation; restoring an existing map requires an explicit --resume-map; restoring poses and patrol progress additionally requires --resume-state saved by the same stop. Do not use this package's live-status.json as a stopped snapshot. The last verified snapshot location and hashes are in saved-map-reference.json.

To restore the source, first stop this experiment, keep the original directory, then extract source-config.tar.gz into /data/robotics/air_ground_smart3d and native-linux.tar.gz into the same location. The native package includes both native binaries plus the private ujson dependency, provenance, and license. Keep this configuration directory under the project's docs/baselines/ for README links and cross-checking. The native files depend on the current Linux environment; other machines should rebuild per the README. The UE delta applies only to the current compatible AirGroundLab project: stop that instance, then extract it into /data/sim/AirGroundLab_UE58/CarlaUnreal; it contains nine integration source files, the current Carla module, the scene's main map, and project configuration, depending on the existing engine, plugin libraries, and referenced assets. Do not extract it over the old Factory or Town10.

dependencies.json records the external environment; virtual environments, Cargo caches, SAM3 weights, full UE Content, and the engine are not included. Paths inside the archive are all relative. manifest.json validates the whole configuration package; source-sha256.json, native-sha256.json, and ue-delta-sha256.json validate the individual archive entries. The old parameter baseline is retained, and the new version is referenced via CURRENT.json.
