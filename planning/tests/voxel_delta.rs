//! Delta contract regression plus a release-only, ignored offline benchmark.
//! Run performance sample with:
//! cargo test --release --test voxel_delta benchmark_million_cells -- --ignored --nocapture --test-threads=1
#[path = "support/delta_oracle.rs"]
mod oracle;

use air_ground_smart3d_worker::voxel::*;
use json::JsonValue as J;
use std::alloc::{GlobalAlloc, Layout, System};
use std::collections::{HashMap, HashSet};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Instant;

struct CountingAllocator;
static COUNT: AtomicBool = AtomicBool::new(false);
static ALLOCS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNT.load(Ordering::Relaxed) {
            ALLOCS.fetch_add(1, Ordering::Relaxed);
            BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        }
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        System.dealloc(p, layout)
    }
    unsafe fn realloc(&self, p: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        if COUNT.load(Ordering::Relaxed) {
            ALLOCS.fetch_add(1, Ordering::Relaxed);
            BYTES.fetch_add(size, Ordering::Relaxed);
        }
        System.realloc(p, layout, size)
    }
}
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

fn empty() -> Map {
    Map {
        identity: Identity {
            scene_id: "test".into(),
            epoch: "epoch".into(),
            content_version: "v1".into(),
            frame_id: "map".into(),
            resolution: 0.25,
        },
        revision: 0,
        watermark: 1.,
        cells: HashMap::new(),
    }
}
fn cell(k: Key, state: &str, time: Option<f64>) -> J {
    json::object! {key:J::Array(k.into_iter().map(J::from).collect()),state:state,
    log_odds:if state=="occupied"{1.0}else{-1.0},
    last_observed_sim_time:time.map(J::from).unwrap_or(J::Null)}
}
fn delta(old: &Map, cells: Vec<J>) -> J {
    let mut q = old.json_identity();
    q["base_revision"] = old.revision.into();
    q["revision"] = (old.revision + 1).into();
    q["capture_sim_time_watermark"] = (old.watermark + 1.).into();
    q["changed_cells"] = cells.into();
    q
}
fn same_changes(a: &Changes, b: &Changes) {
    assert_eq!(a.added, b.added);
    assert_eq!(a.cleared, b.cleared);
    assert_eq!(a.expired, b.expired);
    assert_eq!(a.unknown_removed, b.unknown_removed);
    assert!(a.cleared.windows(2).all(|w| w[0] < w[1]));
    assert!(a.expired.windows(2).all(|w| w[0] < w[1]));
}
fn same_map(a: &Map, b: &Map) {
    assert_eq!(a.identity, b.identity);
    assert_eq!(a.revision, b.revision);
    assert_eq!(a.watermark, b.watermark);
    assert_eq!(a.cells, b.cells);
}
fn compare(q: &J, old: &Map) -> Result<(Map, Changes), String> {
    let old_copy = old.clone();
    let actual = Map::delta(q, old);
    let expected = oracle::delta(q, old);
    same_map(old, &old_copy); // Both success and failure preserve prior snapshots.
    match (&actual, &expected) {
        (Ok((a, ac)), Ok((b, bc))) => {
            same_map(a, b);
            same_changes(ac, bc);
        }
        (Err(a), Err(b)) => assert_eq!(a, b),
        _ => panic!("different results: {actual:?} / {expected:?}"),
    }
    actual
}

#[test]
fn all_geometry_transitions_and_restored_first_observation() {
    // None means unknown, false means free, true means occupied. Restored cells
    // retain occupancy while observed=None; first observation must not add a
    // second obstacle or report a false removal.
    for before in [None, Some(false), Some(true)] {
        for after in [None, Some(false), Some(true)] {
            for restored in [false, true] {
                let mut old = empty();
                if let Some(occupied) = before {
                    old.cells.insert(
                        [-2, 3, 4],
                        Cell {
                            occupied,
                            log_odds: if occupied { 1. } else { -1. },
                            observed: if restored { None } else { Some(1.) },
                        },
                    );
                }
                let q = delta(
                    &old,
                    vec![cell(
                        [-2, 3, 4],
                        match after {
                            None => "unknown",
                            Some(false) => "free",
                            Some(true) => "occupied",
                        },
                        Some(2.),
                    )],
                );
                let (_, ch) = compare(&q, &old).unwrap();
                assert_eq!(
                    ch.added.len(),
                    usize::from(after == Some(true) && before != Some(true))
                );
                assert_eq!(
                    ch.cleared.len(),
                    usize::from(before == Some(true) && after == Some(false))
                );
                assert_eq!(
                    ch.expired.len(),
                    usize::from(before == Some(true) && after.is_none())
                );
            }
        }
    }
}

#[test]
fn refresh_only_and_changed_then_refresh_have_no_extra_geometry_events() {
    let mut old = empty();
    for i in 0..4 {
        old.cells.insert(
            [i, 0, 0],
            Cell {
                occupied: i % 2 == 0,
                log_odds: 1.,
                observed: None,
            },
        );
    }
    let mut q = delta(&old, vec![]);
    q["refreshes"] = json::array![json::object! {capture_sim_time:2.,keys:
    json::array![json::array![0,0,0],json::array![1,0,0],json::array![2,0,0]]}];
    let (next, ch) = compare(&q, &old).unwrap();
    assert!(ch.added.is_empty() && ch.cleared.is_empty() && ch.expired.is_empty());
    assert_eq!(next.cells[&[0, 0, 0]].observed, Some(2.));
    assert_eq!(next.cells[&[1, 0, 0]].observed, Some(2.));
    assert_eq!(next.cells[&[3, 0, 0]].observed, None);
    let mut q = delta(
        &next,
        vec![
            cell([0, 0, 0], "free", Some(3.)),
            cell([4, 0, 0], "occupied", None),
        ],
    );
    q["refreshes"] = json::array![json::object! {capture_sim_time:4.,keys:
    json::array![json::array![0,0,0],json::array![4,0,0],json::array![4,0,0]]}];
    let (refreshed, ch) = compare(&q, &next).unwrap();
    assert_eq!(ch.cleared, vec![[0, 0, 0]]);
    assert_eq!(ch.added, HashSet::from([[4, 0, 0]]));
    assert_eq!(refreshed.cells[&[4, 0, 0]].observed, Some(4.));
}

#[test]
fn invalid_deltas_remain_transactional_and_match_original_errors() {
    let mut old = empty();
    old.revision = 4;
    old.cells.insert(
        [0, 0, 0],
        Cell {
            occupied: true,
            log_odds: 1.,
            observed: Some(1.),
        },
    );
    let valid = delta(&old, vec![cell([1, 0, 0], "occupied", Some(2.))]);
    let mut invalid = Vec::new();
    for field in ["scene_id", "scene_epoch", "content_version", "frame_id"] {
        let mut q = valid.clone();
        q[field] = "other".into();
        invalid.push(q);
    }
    for (field, value) in [
        ("resolution", 0.5),
        ("base_revision", 3.),
        ("revision", 3.),
        ("revision", 4.),
    ] {
        let mut q = valid.clone();
        q[field] = value.into();
        invalid.push(q);
    }
    let mut q = valid.clone();
    q["changed_cells"]
        .push(cell([1, 0, 0], "free", Some(2.)))
        .unwrap();
    invalid.push(q);
    for bad in [
        json::array![1, 2],
        json::array![1, 2, "x"],
        json::array![10000001, 0, 0],
    ] {
        let mut q = valid.clone();
        q["changed_cells"][0]["key"] = bad;
        invalid.push(q);
    }
    let mut q = valid.clone();
    q["changed_cells"][0]["state"] = "bad".into();
    invalid.push(q);
    for bad in [J::from("bad"), J::from(10000001)] {
        let mut q = valid.clone();
        q["changed_cells"][0]["log_odds"] = bad;
        invalid.push(q);
    }
    for refresh in [
        J::from("bad"),
        json::array![json::object! {capture_sim_time:2.,keys:"bad"}],
        json::array![json::object! {capture_sim_time:2.,keys:json::array![json::array![9,0,0]]}],
        json::array![json::object! {capture_sim_time:0.5,keys:json::array![json::array![0,0,0]]}],
        json::array![json::object! {capture_sim_time:2.,keys:json::array![json::array![0,0]]}],
        json::array![json::object! {capture_sim_time:2.,keys:json::array![json::array!["x",0,0]]}],
        json::array![
            json::object! {capture_sim_time:2.,keys:json::array![json::array![10000001,0,0]]}
        ],
        json::array![json::object! {capture_sim_time:"bad",keys:json::array![json::array![0,0,0]]}],
        json::array![
            json::object! {capture_sim_time:3.,keys:json::array![json::array![0,0,0]]},
            json::object! {capture_sim_time:2.,keys:json::array![json::array![0,0,0]]}
        ],
    ] {
        let mut q = valid.clone();
        q["refreshes"] = refresh;
        invalid.push(q);
    }
    let mut q = delta(&old, vec![cell([0, 0, 0], "unknown", None)]);
    q["refreshes"] =
        json::array![json::object! {capture_sim_time:2.,keys:json::array![json::array![0,0,0]]}];
    invalid.push(q);
    // Same-revision refresh and watermark changes must still fail, even with no changed keys.
    let mut q = delta(&old, vec![]);
    q["revision"] = old.revision.into();
    invalid.push(q.clone());
    q["capture_sim_time_watermark"] = old.watermark.into();
    q["refreshes"] =
        json::array![json::object! {capture_sim_time:2.,keys:json::array![json::array![0,0,0]]}];
    invalid.push(q);
    for q in invalid {
        assert!(compare(&q, &old).is_err(), "{}", q.dump());
    }
    let mut q = delta(&old, vec![]);
    q["revision"] = old.revision.into();
    q["capture_sim_time_watermark"] = old.watermark.into();
    compare(&q, &old).unwrap();
    q["refreshes"] =
        json::array![json::object! {capture_sim_time:1.,keys:json::array![json::array![0,0,0]]}];
    compare(&q, &old).unwrap();
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
}
#[test]
fn deterministic_random_snapshots_and_delta_sequences_match_full_diff() {
    let mut rng = Rng(0x8da7213568f48223);
    for _ in 0..12 {
        let mut old = empty();
        for i in -100..100 {
            old.cells.insert(
                [i, i % 7, i % 13],
                Cell {
                    occupied: rng.next() % 3 == 0,
                    log_odds: 0.7,
                    observed: if rng.next() % 4 == 0 { None } else { Some(1.) },
                },
            );
        }
        for step in 0..80 {
            let mut keys = HashSet::new();
            let count = (rng.next() % 70) as usize;
            let mut cells = Vec::new();
            for _ in 0..count {
                let i = (rng.next() % 240) as i32 - 120;
                let k = [i, i % 7, i % 13];
                if !keys.insert(k) {
                    continue;
                }
                let state = match rng.next() % 3 {
                    0 => "unknown",
                    1 => "free",
                    _ => "occupied",
                };
                cells.push(cell(
                    k,
                    state,
                    if rng.next() % 9 == 0 {
                        None
                    } else {
                        Some(step as f64 + 2.)
                    },
                ));
            }
            let mut q = delta(&old, cells);
            // Unchanged restored cells are eligible for their first timestamp refresh.
            let mut refresh_keys = old
                .cells
                .keys()
                .copied()
                .filter(|k| !keys.contains(k))
                .collect::<Vec<_>>();
            refresh_keys.sort();
            refresh_keys.truncate(16);
            q["refreshes"] = json::array![json::object! {capture_sim_time:step as f64 + 2.,
            keys:J::Array(refresh_keys.iter().copied().map(|k| J::Array(k.into_iter().map(J::from).collect())).collect())}];
            old = compare(&q, &old).unwrap().0;
        }
    }
}

fn percentile(v: &[f64], p: f64) -> f64 {
    let mut a = v.to_vec();
    a.sort_by(f64::total_cmp);
    a[((a.len() - 1) as f64 * p).ceil() as usize]
}
fn measure<T>(f: impl FnOnce() -> T) -> (T, f64, usize, usize) {
    ALLOCS.store(0, Ordering::Relaxed);
    BYTES.store(0, Ordering::Relaxed);
    COUNT.store(true, Ordering::Relaxed);
    let start = Instant::now();
    let value = f();
    let ms = start.elapsed().as_secs_f64() * 1000.;
    COUNT.store(false, Ordering::Relaxed);
    (
        value,
        ms,
        ALLOCS.load(Ordering::Relaxed),
        BYTES.load(Ordering::Relaxed),
    )
}
#[test]
#[ignore = "offline million-cell release benchmark; no live services changed"]
fn benchmark_million_cells() {
    assert!(!cfg!(debug_assertions), "run with --release");
    let mut old = empty();
    old.cells = HashMap::with_capacity(1_000_000);
    for i in 0..1_000_000i32 {
        old.cells.insert(
            [i % 1000, i / 1000, 0],
            Cell {
                occupied: i % 3 == 0,
                log_odds: if i % 3 == 0 { 1. } else { -1. },
                observed: Some(1.),
            },
        );
    }
    for count in [0, 32, 4096] {
        let cells = (0..count)
            .map(|i| {
                let k = [i % 1000, i / 1000, 0];
                cell(
                    k,
                    match i % 3 {
                        0 => "free",
                        1 => "occupied",
                        _ => "unknown",
                    },
                    Some(2.),
                )
            })
            .collect();
        let mut q = delta(&old, cells);
        if count == 0 {
            q["refreshes"] = json::array![json::object! {capture_sim_time:2.,
            keys:J::Array((0..8192i32).map(|i| json::array![i%1000,i/1000,0]).collect())}];
        }
        // Warm up both implementations; all comparison work stays outside timing.
        compare(&q, &old).unwrap();
        let mut times = [Vec::new(), Vec::new()];
        let mut allocs = [Vec::new(), Vec::new()];
        let mut bytes = [Vec::new(), Vec::new()];
        for rep in 0..12 {
            for index in if rep % 2 == 0 { [0, 1] } else { [1, 0] } {
                let (result, ms, n, b) = measure(|| {
                    if index == 0 {
                        oracle::delta(black_box(&q), black_box(&old))
                    } else {
                        Map::delta(black_box(&q), black_box(&old))
                    }
                });
                let result = result.unwrap();
                black_box(&result);
                times[index].push(ms);
                allocs[index].push(n as f64);
                bytes[index].push(b as f64);
            }
        }
        for index in 0..2 {
            println!("{{\"benchmark\":\"million_cell_delta\",\"implementation\":\"{}\",\"cells\":1000000,\"changed_keys\":{},\"refresh_keys\":{},\"samples\":12,\"p50_ms\":{:.4},\"p95_ms\":{:.4},\"allocation_calls_p50\":{},\"allocated_bytes_p50\":{}}}",
                if index==0 {"clone_full_diff_oracle"} else {"clone_changed_key_diff"}, count, if count==0 {8192}else{0},
                percentile(&times[index],0.5),percentile(&times[index],0.95),percentile(&allocs[index],0.5),percentile(&bytes[index],0.5));
        }
    }
    let mut clone_ms = Vec::new();
    let mut scan_ms = Vec::new();
    for _ in 0..12 {
        let (copy, elapsed, _, _) = measure(|| black_box(&old).clone());
        clone_ms.push(elapsed);
        let (ch, elapsed, _, _) =
            measure(|| oracle::full_diff(black_box(&copy), Some(black_box(&old))));
        scan_ms.push(elapsed);
        black_box(ch);
    }
    println!("{{\"benchmark\":\"million_cell_component\",\"clone_p50_ms\":{:.4},\"clone_p95_ms\":{:.4},\"unchanged_full_diff_p50_ms\":{:.4},\"unchanged_full_diff_p95_ms\":{:.4}}}",
        percentile(&clone_ms,0.5),percentile(&clone_ms,0.95),percentile(&scan_ms,0.5),percentile(&scan_ms,0.95));
}
