// Frozen clone + whole-map diff implementation from 2026-09-29, before
// changed-key optimization. Deliberately independent of production delta.
use air_ground_smart3d_worker::voxel::*;
use json::JsonValue as J;
use std::collections::HashSet;

fn identity(v: &J) -> Result<Identity, String> {
    let r = number(&v["resolution"], "resolution")?;
    if !(0.025..=2.).contains(&r) {
        return Err("resolution outside [0.025,2]m".into());
    }
    Ok(Identity {
        scene_id: string(&v["scene_id"], "scene_id")?,
        epoch: string(&v["scene_epoch"], "scene_epoch")?,
        content_version: string(&v["content_version"], "content_version")?,
        frame_id: string(&v["frame_id"], "frame_id")?,
        resolution: r,
    })
}
fn parse_cells(v: &J, delta: bool) -> Result<Vec<(Key, Option<Cell>)>, String> {
    if !v.is_array() || v.len() > 2_000_000 {
        return Err("cell array required; limit 2000000".into());
    }
    let mut seen = HashSet::new();
    let mut out = Vec::with_capacity(v.len());
    for c in v.members() {
        if !c["key"].is_array() || c["key"].len() != 3 {
            return Err("cell.key must be signed [ix,iy,iz]".into());
        }
        let mut k = [0; 3];
        for i in 0..3 {
            k[i] = c["key"][i]
                .as_i32()
                .filter(|n| (-10_000_000..=10_000_000).contains(n))
                .ok_or("cell.key out of range")?;
        }
        if !seen.insert(k) {
            return Err("duplicate cell.key".into());
        }
        let state = match c["state"].as_str() {
            Some("occupied") => Some(true),
            Some("free") => Some(false),
            Some("unknown") if delta => None,
            _ => return Err("invalid cell.state".into()),
        };
        let value = if let Some(occupied) = state {
            Some(Cell {
                occupied,
                log_odds: number(&c["log_odds"], "log_odds")?,
                observed: if c["last_observed_sim_time"].is_null() {
                    None
                } else {
                    Some(number(
                        &c["last_observed_sim_time"],
                        "last_observed_sim_time",
                    )?)
                },
            })
        } else {
            None
        };
        out.push((k, value));
    }
    Ok(out)
}
pub fn delta(v: &J, old: &Map) -> Result<(Map, Changes), String> {
    if identity(v)? != old.identity {
        return Err("map_identity_changed_reinitialize".into());
    }
    if uint(&v["base_revision"], "base_revision")? != old.revision {
        return Err("snapshot_required: map_revision_gap".into());
    }
    let rev = uint(&v["revision"], "revision")?;
    if rev < old.revision {
        return Err("stale_map_revision".into());
    }
    let changes = parse_cells(&v["changed_cells"], true)?;
    if rev == old.revision && !changes.is_empty() {
        return Err("map_content_changed_without_revision".into());
    }
    let mut next = old.clone();
    next.revision = rev;
    next.watermark = number(
        &v["capture_sim_time_watermark"],
        "capture_sim_time_watermark",
    )?;
    for (k, c) in changes {
        if let Some(c) = c {
            next.cells.insert(k, c);
        } else {
            next.cells.remove(&k);
        }
    }
    if rev == old.revision && next.watermark != old.watermark {
        return Err("map_content_changed_without_revision".into());
    }
    // Compact mapper deltas preserve occupancy classes but refresh evidence time.
    if !v["refreshes"].is_null() {
        if !v["refreshes"].is_array() {
            return Err("refreshes must be an array".into());
        }
        for group in v["refreshes"].members() {
            let time = number(&group["capture_sim_time"], "refresh capture_sim_time")?;
            if !group["keys"].is_array() || group["keys"].len() > 2_000_000 {
                return Err("refresh keys array required".into());
            }
            for value in group["keys"].members() {
                if !value.is_array() || value.len() != 3 {
                    return Err("refresh key must have 3 integers".into());
                }
                let mut k = [0; 3];
                for i in 0..3 {
                    k[i] = value[i]
                        .as_i32()
                        .filter(|n| (-10_000_000..=10_000_000).contains(n))
                        .ok_or("refresh key invalid")?;
                }
                let cell = next
                    .cells
                    .get_mut(&k)
                    .ok_or("snapshot_required: refresh references unknown cell")?;
                if cell.observed.map_or(false, |observed| time < observed) {
                    return Err("stale_cell_refresh".into());
                }
                cell.observed = Some(time);
            }
        }
    }
    if rev == old.revision && next.cells != old.cells {
        return Err("map_content_changed_without_revision".into());
    }
    let ch = full_diff(&next, Some(old));
    Ok((next, ch))
}
pub fn full_diff(map: &Map, old: Option<&Map>) -> Changes {
    let mut c = Changes::default();
    for (k, v) in &map.cells {
        let was = old
            .and_then(|m| m.cells.get(k))
            .map_or(false, |v| v.occupied);
        if v.occupied && !was {
            c.added.insert(*k);
        } else if !v.occupied && was {
            c.cleared.push(*k);
        }
    }
    if let Some(o) = old {
        c.expired = o
            .cells
            .iter()
            .filter(|(k, v)| v.occupied && !map.cells.contains_key(*k))
            .map(|(k, _)| *k)
            .collect();
        c.expired.sort();
        c.unknown_removed = c.expired.len();
    }
    c.cleared.sort();
    c
}
