//! Analytic sphere/capsule/ellipse OHZ geometry; enclosing spheres are broadphase only.
use crate::obstacles::{SphericalObstacle, StaticSphericalObstacle};
use crate::rrt::RealVectorState;
use num_traits::Float;

#[derive(Clone, Debug)]
pub enum HazardZone<F: Float, const N: usize> {
    Sphere(StaticSphericalObstacle<F, N>),
    /// Fully specified affine ellipsoid, shared with external guard/display.
    /// The caller validates finite orthonormal axes and positive semi-axes.
    Ellipsoid {
        center: [f64; N],
        axes: [[f64; N]; N],
        semi_axes: [f64; N],
        bound: StaticSphericalObstacle<F, N>,
    },
    Capsule {
        start: RealVectorState<F, N>,
        end: RealVectorState<F, N>,
        radius: F,
        bound: StaticSphericalObstacle<F, N>,
    },
    /// In 2D, the conservative ellipse of a swept disk. The same affine
    /// predicate extends to a prolate ellipsoid in higher dimensions.
    Ellipse {
        center: [f64; N],
        axis: [f64; N],
        semi_major: f64,
        semi_minor: f64,
        bound: StaticSphericalObstacle<F, N>,
    },
}

impl<F: Float, const N: usize> HazardZone<F, N> {
    pub fn ellipsoid(center: [f64; N], axes: [[f64; N]; N], semi_axes: [f64; N]) -> Self {
        let bound_center = RealVectorState::new(center.map(|v| F::from(v).unwrap()));
        let rounding = center.iter().zip(bound_center.values()).map(|(a,b)|
            (a-b.to_f64().unwrap()).powi(2)).sum::<f64>().sqrt();
        let desired = semi_axes.iter().copied().fold(0.0_f64, f64::max) + rounding;
        let mut radius = F::from(desired).unwrap();
        if radius.to_f64().unwrap() < desired { radius = radius + radius.abs()*F::epsilon(); }
        Self::Ellipsoid { center, axes, semi_axes,
            bound: StaticSphericalObstacle::new(bound_center, radius) }
    }

    pub fn capsule(start: RealVectorState<F, N>, end: RealVectorState<F, N>, radius: F) -> Self {
        let half = F::from(0.5).unwrap();
        let bound = StaticSphericalObstacle::new(start + (end - start) * half,
            radius + start.euclidean_distance(&end) * half);
        Self::Capsule { start, end, radius, bound }
    }

    /// Enclose the entire swept disk, including both endpoint disks. With
    /// h = |end-start|/2 and rho = radius, a = h+rho and b = sqrt(rho*a).
    /// For any unit direction with axial component c, the difference between
    /// squared ellipse and capsule supports is rho*h*(1-|c|)^2 >= 0.
    /// Thus a strict ellipse is conservative; choosing b=rho would not be.
    /// Callers must supply finite endpoints and a finite positive radius.
    pub fn ellipse(start: RealVectorState<F, N>, end: RealVectorState<F, N>, radius: F) -> Self {
        let start64 = start.values().map(|x| x.to_f64().unwrap());
        let end64 = end.values().map(|x| x.to_f64().unwrap());
        let delta: [f64; N] = std::array::from_fn(|i| end64[i] - start64[i]);
        let travel = delta.iter().map(|x| x*x).sum::<f64>().sqrt();
        let center: [f64; N] = std::array::from_fn(|i| start64[i] + delta[i]*0.5);
        let axis = std::array::from_fn(|i| if travel > 0.0 { delta[i]/travel }
            else if i == 0 { 1.0 } else { 0.0 });
        let rho = radius.to_f64().unwrap();
        let semi_major = travel*0.5 + rho;
        let semi_minor = (rho*semi_major).sqrt();
        // Keep narrowphase in f64. Account for midpoint/radius rounding only
        // in the enclosing sphere so the broadphase never clips the ellipse.
        let bound_center = RealVectorState::new(center.map(|x| F::from(x).unwrap()));
        let center_rounding = center.iter().zip(bound_center.values()).map(|(a,b)|
            (a-b.to_f64().unwrap()).powi(2)).sum::<f64>().sqrt();
        let desired_bound = semi_major + center_rounding;
        let mut bound_radius = F::from(desired_bound).unwrap();
        if bound_radius.to_f64().unwrap() < desired_bound {
            bound_radius = bound_radius + bound_radius.abs()*F::epsilon();
        }
        let bound = StaticSphericalObstacle::new(bound_center, bound_radius);
        Self::Ellipse { center, axis, semi_major, semi_minor, bound }
    }

    pub fn center(&self) -> &RealVectorState<F, N> {
        match self { Self::Sphere(s) => s.center(),
            Self::Capsule { bound, .. } | Self::Ellipse { bound, .. } | Self::Ellipsoid { bound, .. } => bound.center() }
    }

    /// Radius of the enclosing sphere, never a narrowphase collision predicate.
    pub fn radius(&self) -> F {
        match self { Self::Sphere(s) => s.radius(),
            Self::Capsule { bound, .. } | Self::Ellipse { bound, .. } | Self::Ellipsoid { bound, .. } => bound.radius() }
    }

    pub fn contains(&self, point: &RealVectorState<F, N>) -> bool {
        match self {
            Self::Sphere(s) => s.contains(point),
            Self::Ellipsoid { center, axes, semi_axes, .. } =>
                ellipsoid_edge_distance_squared(point, point, center, axes, semi_axes) <= 1.0 + 1.0e-6,
            Self::Capsule { start, end, radius, .. } =>
                segment_distance_squared(point, point, start, end) <= radius.to_f64().unwrap().powi(2),
            Self::Ellipse { center, axis, semi_major, semi_minor, .. } =>
                ellipse_edge_distance_squared(point, point, center, axis, *semi_major, *semi_minor)
                    <= 1.0 + 1.0e-6,
        }
    }

    pub fn intersects_edge(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> bool {
        match self {
            Self::Sphere(s) => s.intersects_edge(a, b),
            Self::Ellipsoid { center, axes, semi_axes, .. } =>
                ellipsoid_edge_distance_squared(a, b, center, axes, semi_axes) <= 1.0 + 1.0e-6,
            Self::Capsule { start, end, radius, .. } =>
                segment_distance_squared(a, b, start, end) <= radius.to_f64().unwrap().powi(2),
            Self::Ellipse { center, axis, semi_major, semi_minor, .. } =>
                ellipse_edge_distance_squared(a, b, center, axis, *semi_major, *semi_minor)
                    <= 1.0 + 1.0e-6,
        }
    }

    /// Preserve the original spherical recovery behavior. A robot already in a
    /// directional sweep stays blocked: shrinking it to a sphere would discard
    /// the very future occupancy selected by this experiment. The caller's
    /// temporal policy can wait for the next safe snapshot.
    pub fn for_robot_inside(&self, robot: &RealVectorState<F, N>, margin: F) -> Self {
        match self {
            Self::Sphere(s) => Self::Sphere(StaticSphericalObstacle::new(*s.center(),
                (s.center().euclidean_distance(robot) - margin).max(F::zero()))),
            Self::Capsule { .. } | Self::Ellipse { .. } | Self::Ellipsoid { .. } => self.clone(),
        }
    }
}

/// Exact affine segment-to-unit-ball minimum, matching perception/hazards.py.
fn ellipsoid_edge_distance_squared<F: Float, const N: usize>(
    a: &RealVectorState<F, N>, b: &RealVectorState<F, N>,
    center: &[f64; N], axes: &[[f64; N]; N], semi_axes: &[f64; N],
) -> f64 {
    let transform = |p: &RealVectorState<F, N>| std::array::from_fn::<_, N, _>(|i|
        (0..N).map(|k| (p.values()[k].to_f64().unwrap()-center[k])*axes[i][k]).sum::<f64>()/semi_axes[i]);
    let start = transform(a); let end = transform(b);
    let direction: [f64; N] = std::array::from_fn(|i| end[i]-start[i]);
    let length = direction.iter().map(|x| x*x).sum::<f64>();
    let t = if length > 0.0 {
        (-(0..N).map(|i| start[i]*direction[i]).sum::<f64>()/length).clamp(0.0,1.0)
    } else {0.0};
    (0..N).map(|i| (start[i]+t*direction[i]).powi(2)).sum()
}

/// Apply the ellipse's affine map to the unit ball, then exactly minimize
/// squared distance to the transformed segment. The normalized 1e-6 comparison
/// tolerance above only absorbs f32 endpoint/boundary rounding; no sampling,
/// bounding circle or angular discretization enters narrowphase checks.
fn ellipse_edge_distance_squared<F: Float, const N: usize>(
    a: &RealVectorState<F, N>, b: &RealVectorState<F, N>,
    center: &[f64; N], axis: &[f64; N], semi_major: f64, semi_minor: f64,
) -> f64 {
    let transform = |point: &RealVectorState<F, N>| {
        let delta: [f64; N] = std::array::from_fn(|i| point.values()[i].to_f64().unwrap()-center[i]);
        let parallel = (0..N).map(|i| delta[i]*axis[i]).sum::<f64>();
        std::array::from_fn::<_, N, _>(|i| (delta[i]-parallel*axis[i])/semi_minor
            + parallel*axis[i]/semi_major)
    };
    let from = transform(a);
    let to = transform(b);
    let direction: [f64; N] = std::array::from_fn(|i| to[i]-from[i]);
    let squared_length = direction.iter().map(|x| x*x).sum::<f64>();
    let t = if squared_length > 0.0 {
        (-(0..N).map(|i| from[i]*direction[i]).sum::<f64>()/squared_length).clamp(0.0, 1.0)
    } else { 0.0 };
    (0..N).map(|i| (from[i]+t*direction[i]).powi(2)).sum()
}

/// Minimum distance of two finite line segments in any dimension. Evaluate
/// the four boundary minima and the interior stationary point. f64 arithmetic
/// avoids f32 cancellation on almost parallel paths; degenerate segments are
/// handled explicitly, including a stationary obstacle or a zero-length guard.
fn segment_distance_squared<F: Float, const N: usize>(
    a: &RealVectorState<F, N>, b: &RealVectorState<F, N>,
    c: &RealVectorState<F, N>, d: &RealVectorState<F, N>,
) -> f64 {
    let av = a.values().map(|v| v.to_f64().unwrap());
    let bv = b.values().map(|v| v.to_f64().unwrap());
    let cv = c.values().map(|v| v.to_f64().unwrap());
    let dv = d.values().map(|v| v.to_f64().unwrap());
    let u: [f64; N] = std::array::from_fn(|i| bv[i] - av[i]);
    let v: [f64; N] = std::array::from_fn(|i| dv[i] - cv[i]);
    let w: [f64; N] = std::array::from_fn(|i| av[i] - cv[i]);
    let dot = |x: &[f64; N], y: &[f64; N]| (0..N).map(|i| x[i] * y[i]).sum::<f64>();
    let (uu, uv, vv, uw, vw) = (dot(&u, &u), dot(&u, &v), dot(&v, &v), dot(&u, &w), dot(&v, &w));
    let dist = |s: f64, t: f64| (0..N).map(|i| (w[i] + u[i] * s - v[i] * t).powi(2)).sum::<f64>();
    let project = |numerator: f64, denominator: f64| if denominator > 0.0 {
        (numerator / denominator).clamp(0.0, 1.0)
    } else { 0.0 };
    let mut best = dist(0.0, project(vw, vv))
        .min(dist(1.0, project(vw + uv, vv)))
        .min(dist(project(-uw, uu), 0.0))
        .min(dist(project(uv - uw, uu), 1.0));
    // Sum of squared 2x2 minors avoids cancellation in uu*vv-uv*uv.
    let mut determinant = 0.0;
    for i in 0..N { for j in (i + 1)..N {
        determinant += (u[i] * v[j] - u[j] * v[i]).powi(2);
    }}
    if determinant > 0.0 {
        let s = (uv * vw - vv * uw) / determinant;
        let t = (uu * vw - uv * uw) / determinant;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            best = best.min(dist(s, t));
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    type State = RealVectorState<f32, 2>;
    fn p(x: f32, y: f32) -> State { State::new([x, y]) }
    fn sweep() -> HazardZone<f32, 2> { HazardZone::capsule(p(0., 0.), p(4., 0.), 1.) }

    #[test]
    fn directed_sweep_contains_current_disk_and_forward_cap_but_not_backward_dilation() {
        let h = sweep();
        for xy in [[-1., 0.], [0., 1.], [0., -1.], [4., 1.], [5., 0.]] {
            assert!(h.contains(&State::new(xy)));
        }
        assert!(!h.contains(&p(-2., 0.)));
        assert!(StaticSphericalObstacle::new(p(0., 0.), 5.).contains(&p(-2., 0.)));
        assert_eq!(*h.center(), p(2., 0.));
        assert_eq!(h.radius(), 3.);
    }

    #[test]
    fn crossing_tangent_parallel_and_degenerate_edges_are_exact() {
        let h = sweep();
        assert!(h.intersects_edge(&p(2., -5.), &p(2., 5.)));
        assert!(h.intersects_edge(&p(-4., 1.), &p(8., 1.)));
        assert!(!h.intersects_edge(&p(-4., 1.001), &p(8., 1.001)));
        assert!(!h.intersects_edge(&p(-2., -2.), &p(-2., 2.)));
        assert!(h.intersects_edge(&p(5., 0.), &p(5., 0.)));
        assert!(!h.intersects_edge(&p(5.001, 0.), &p(5.001, 0.)));
        let stationary = HazardZone::capsule(p(2., 2.), p(2., 2.), 1.);
        assert!(stationary.intersects_edge(&p(0., 2.), &p(4., 2.)));
        assert!(stationary.contains(&p(2., 3.)));
        assert!(!stationary.contains(&p(2., 3.001)));
    }

    #[test]
    fn reversed_and_diagonal_axes_do_not_change_distance() {
        let a = HazardZone::capsule(p(1., -2.), p(4., 2.), 0.5);
        let b = HazardZone::capsule(p(4., 2.), p(1., -2.), 0.5);
        for edge in [(p(-2., 0.), p(6., 0.)), (p(2., 4.), p(5., 4.)), (p(2., -1.), p(2., -1.))] {
            assert_eq!(a.intersects_edge(&edge.0, &edge.1), b.intersects_edge(&edge.0, &edge.1));
        }
    }

    #[test]
    fn robot_inside_directional_hazard_never_erases_future_occupancy() {
        let h = sweep().for_robot_inside(&p(2., 0.5), 0.01);
        assert!(h.contains(&p(2., 0.5)));
        assert!(h.intersects_edge(&p(3., -2.), &p(3., 2.)));
        let sphere = HazardZone::Sphere(StaticSphericalObstacle::new(p(0., 0.), 5.));
        assert!(!sphere.for_robot_inside(&p(2., 0.), 0.01).contains(&p(2., 0.)));
    }

    #[test]
    fn ellipse_matches_shared_python_fixture_and_uses_exact_narrowphase() {
        // Shared Python fixture: p=(2,3), v=(1,0), T=2, rho=.5.
        let h = HazardZone::ellipse(p(2.,3.),p(4.,3.),0.5);
        assert_eq!(*h.center(),p(3.,3.));
        assert_eq!(h.radius(),1.5);
        for xy in [[1.5,3.],[4.5,3.],[3.,3.86]] {
            assert!(h.contains(&State::new(xy)), "{xy:?}");
        }
        assert!(!h.contains(&p(3.,3.87)));
        assert!(h.intersects_edge(&p(3.,1.),&p(3.,5.)));
        assert!(h.intersects_edge(&p(3.,5.),&p(3.,1.)));
        let tangent = 3. + 0.75_f32.sqrt();
        assert!(h.intersects_edge(&p(0.,tangent),&p(6.,tangent)));
        assert!(!h.intersects_edge(&p(0.,3.9),&p(6.,3.9)));
        assert!(!h.intersects_edge(&p(0.,3.9),&p(0.,3.9)));
        assert!(h.intersects_edge(&p(4.5,3.),&p(4.5,3.)));
        // Inside the midpoint sphere, outside the ellipse.
        assert!(StaticSphericalObstacle::new(*h.center(),h.radius()).contains(&p(3.,4.)));
        assert!(!h.contains(&p(3.,4.)));
        // An actual ellipse, wider at the middle than the capsule.
        assert!(!HazardZone::capsule(p(2.,3.),p(4.,3.),0.5).contains(&p(3.,3.8)));
        assert!(h.contains(&p(3.,3.8)));
    }

    #[test]
    fn stationary_ellipse_is_a_circle_and_robot_inside_never_shrinks_it() {
        let h = HazardZone::ellipse(p(2.,3.),p(2.,3.),0.5);
        let sphere = StaticSphericalObstacle::new(p(2.,3.),0.5);
        for xy in [[2.,3.],[2.4,3.2],[2.4,3.4],[2.,3.501]] {
            let point = State::new(xy);
            assert_eq!(h.contains(&point),sphere.contains(&point));
        }
        // Directed shapes use closed boundaries, as the capsule does; the
        // legacy sphere's strict contains predicate remains unchanged.
        assert!(h.contains(&p(2.5,3.)));
        assert!(h.contains(&p(2.,3.5)));
        for (a,b) in [(p(0.,3.),p(4.,3.)),(p(0.,3.5),p(4.,3.5)),
            (p(0.,3.501),p(4.,3.501))] {
            assert_eq!(h.intersects_edge(&a,&b),sphere.intersects_edge(&a,&b));
        }
        let moving = HazardZone::ellipse(p(0.,0.),p(4.,0.),1.);
        let guarded = moving.for_robot_inside(&p(2.,0.5),0.01);
        assert!(guarded.contains(&p(2.,0.5)));
        assert!(guarded.contains(&p(5.,0.)));
        assert_eq!(guarded.radius(),moving.radius());
    }

    #[test]
    fn ellipse_capsule_containment_including_diagonal_endpoint_disks() {
        for (start,end) in [(p(0.,0.),p(6.,0.)),(p(1.,-2.),p(4.,2.)),
            (p(-3.,2.),p(-3.,2.)),(p(4.,2.),p(1.,-2.))] {
            let h = HazardZone::ellipse(start,end,0.5);
            // Includes complete endpoint disks, side walls and the interior.
            for step in 0..=20 {
                let center = start+(end-start)*(step as f32/20.);
                for angle in 0..360 {
                    let radians = (angle as f32).to_radians();
                    let point = center+p(radians.cos()*0.5,radians.sin()*0.5);
                    assert!(h.contains(&point), "{start:?} {end:?} {point:?}");
                }
            }
        }
    }

    #[test]
    fn ellipse_rotates_with_diagonal_motion_and_rotated_paths() {
        let base = HazardZone::ellipse(p(0.,0.),p(4.,0.),1.);
        let rotate = |q: State| p(10.+0.6*q.values()[0]-0.8*q.values()[1],
            -5.+0.8*q.values()[0]+0.6*q.values()[1]);
        let diagonal = HazardZone::ellipse(rotate(p(0.,0.)),rotate(p(4.,0.)),1.);
        let reversed = HazardZone::ellipse(rotate(p(4.,0.)),rotate(p(0.,0.)),1.);
        for (a,b) in [(p(2.,-5.),p(2.,5.)),(p(-4.,1.8),p(8.,1.8)),
            (p(-4.,1.7),p(8.,1.7)),(p(-2.,-2.),p(-2.,2.)),
            (p(4.9,0.),p(4.9,0.)),(p(5.01,0.),p(5.01,0.))] {
            let expected = base.intersects_edge(&a,&b);
            assert_eq!(diagonal.intersects_edge(&rotate(a),&rotate(b)),expected);
            assert_eq!(reversed.intersects_edge(&rotate(b),&rotate(a)),expected);
        }
    }
}
