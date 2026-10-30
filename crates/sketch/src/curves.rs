//! Free-form sketch curves: ellipses, splines (through fit points, or a clamped B-spline over
//! control points) and conics (rational quadratic Béziers). Evaluation works on positions so
//! the solver can use it with trial values.

use std::f64::consts::TAU;

use solidcraft_geom::Vec2;

/// Polyline samples per spline span / conic / ellipse quarter.
const SPAN_SAMPLES: usize = 24;
/// Most points a spline may have.
pub const MAX_SPLINE_POINTS: usize = 500;

/// Point on an ellipse with centre `c`, major-axis end `m` and minor radius `r` at angle `t`.
pub fn ellipse_point(c: Vec2, m: Vec2, r: f64, t: f64) -> Vec2 {
    let u = m - c;
    let a = u.len().max(1e-12);
    let v = u.perp() / a * r;
    c + u * t.cos() + v * t.sin()
}

pub fn ellipse_polyline(c: Vec2, m: Vec2, r: f64) -> Vec<Vec2> {
    let n = SPAN_SAMPLES * 4;
    (0..=n).map(|i| ellipse_point(c, m, r, TAU * i as f64 / n as f64)).collect()
}

/// Rational quadratic Bézier from `a` to `b` with control `apex` and shape `rho` in (0, 1)
/// (0.5 = parabola, below = ellipse, above = hyperbola).
pub fn conic_point(a: Vec2, apex: Vec2, b: Vec2, rho: f64, t: f64) -> Vec2 {
    let w = rho / (1.0 - rho).max(1e-9);
    let (b0, b1, b2) = ((1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t) * w, t * t);
    let den = b0 + b1 + b2;
    (a * b0 + apex * b1 + b * b2) / den.max(1e-12)
}

/// The `rho` that puts the conic's midpoint (`conic_point` at `t = 0.5`) at `p`, projected onto
/// the segment from the chord's midpoint to `apex`: at `t = 0.5`, `conic_point` is exactly
/// `mid + rho * (apex - mid)` (its two chord weights are equal there), so `rho` is `p`'s fraction
/// along that segment. Lets a UI turn a dragged or hovered point into a shape (drag the apex and
/// `rho` stays; drag the curve's midpoint handle and `rho` follows the drag instead). Clamped
/// well inside (0, 1) so the result is always a usable conic; `apex` on the chord's midpoint (no
/// segment to project onto) falls back to a parabola.
pub fn conic_rho_from_point(a: Vec2, apex: Vec2, b: Vec2, p: Vec2) -> f64 {
    let mid = (a + b) * 0.5;
    let d = apex - mid;
    let len2 = d.len2();
    if !len2.is_finite() || len2 < 1e-12 {
        return 0.5;
    }
    let t = (p - mid).dot(d) / len2;
    if t.is_finite() { t.clamp(1e-3, 1.0 - 1e-3) } else { 0.5 }
}

pub fn conic_polyline(a: Vec2, apex: Vec2, b: Vec2, rho: f64) -> Vec<Vec2> {
    // Samples by how far the curve bows from its chord (small text curves need few).
    let chord = b - a;
    let bow = chord.cross(apex - a).abs() / chord.len().max(1e-12);
    let n = ((bow / 2e-3).sqrt().ceil() as usize).clamp(2, SPAN_SAMPLES * 2);
    (0..=n).map(|i| conic_point(a, apex, b, rho, i as f64 / n as f64)).collect()
}

/// Natural cubic interpolation through `p` (chord-length parameters): per span the cubic
/// Hermite data (start, end, start tangent, end tangent) in span-parameter units.
fn fit_spans(p: &[Vec2]) -> Vec<(Vec2, Vec2, Vec2, Vec2)> {
    let n = p.len();
    if n < 2 {
        return Vec::new();
    }
    let h: Vec<f64> = p.windows(2).map(|w| w[0].dist(w[1]).max(1e-9)).collect();
    if n == 2 {
        let d = p[1] - p[0];
        return vec![(p[0], p[1], d, d)];
    }
    // Second derivatives M (natural ends: M0 = Mn = 0) by the tridiagonal (Thomas) solve.
    let m = n - 2;
    let mut diag = vec![0.0; m];
    let mut up = vec![0.0; m];
    let mut low = vec![0.0; m];
    let mut rhs = vec![Vec2::ZERO; m];
    for i in 0..m {
        let (h0, h1) = (h[i], h[i + 1]);
        diag[i] = 2.0 * (h0 + h1);
        low[i] = h0;
        up[i] = h1;
        rhs[i] = ((p[i + 2] - p[i + 1]) / h1 - (p[i + 1] - p[i]) / h0) * 6.0;
    }
    for i in 1..m {
        let w = low[i] / diag[i - 1];
        diag[i] -= w * up[i - 1];
        rhs[i] = rhs[i] - rhs[i - 1] * w;
    }
    let mut sol = vec![Vec2::ZERO; m];
    for i in (0..m).rev() {
        let next = if i + 1 < m { sol[i + 1] * up[i] } else { Vec2::ZERO };
        sol[i] = (rhs[i] - next) / diag[i];
    }
    let mm = |i: usize| if i == 0 || i == n - 1 { Vec2::ZERO } else { sol[i - 1] };
    (0..n - 1)
        .map(|i| {
            let hi = h[i];
            let slope = (p[i + 1] - p[i]) / hi;
            // First derivatives (per unit chord parameter) at both ends of the span.
            let d0 = slope - (mm(i) * 2.0 + mm(i + 1)) * (hi / 6.0);
            let d1 = slope + (mm(i) + mm(i + 1) * 2.0) * (hi / 6.0);
            (p[i], p[i + 1], d0 * hi, d1 * hi)
        })
        .collect()
}

fn hermite(p0: Vec2, p1: Vec2, t0: Vec2, t1: Vec2, s: f64) -> Vec2 {
    let (s2, s3) = (s * s, s * s * s);
    p0 * (2.0 * s3 - 3.0 * s2 + 1.0) + t0 * (s3 - 2.0 * s2 + s) + p1 * (-2.0 * s3 + 3.0 * s2) + t1 * (s3 - s2)
}

/// Clamped uniform B-spline of `degree` over control points `p` at `t` in [0, 1] (de Boor).
fn bspline_at(p: &[Vec2], degree: usize, t: f64) -> Vec2 {
    let n = p.len();
    let k = degree.min(n.saturating_sub(1)).max(1);
    let spans = n - k;
    let mut knots = vec![0.0; k + 1];
    for i in 1..spans {
        knots.push(i as f64 / spans as f64);
    }
    knots.extend(std::iter::repeat_n(1.0, k + 1));
    let mut s = k;
    while s + 1 < n && knots.get(s + 1).is_some_and(|x| *x <= t) {
        s += 1;
    }
    let mut d: Vec<Vec2> = (0..=k).map(|j| p.get(j + s - k).copied().unwrap_or_default()).collect();
    for r in 1..=k {
        for j in (r..=k).rev() {
            let i0 = j + s - k;
            let (lo, hi) = (knots.get(i0).copied().unwrap_or(0.0), knots.get(i0 + k + 1 - r).copied().unwrap_or(1.0));
            let a = if hi - lo > 1e-15 { (t - lo) / (hi - lo) } else { 0.0 };
            d[j] = d[j - 1] * (1.0 - a) + d[j] * a;
        }
    }
    d[k]
}

/// Clamped uniform B-spline of `degree` over control points `p`, sampled.
fn bspline_polyline(p: &[Vec2], degree: usize) -> Vec<Vec2> {
    let n = p.len();
    let k = degree.min(n.saturating_sub(1)).max(1);
    let total = SPAN_SAMPLES * (n - k);
    (0..=total).map(|i| bspline_at(p, degree, i as f64 / total as f64)).collect()
}

/// Point of a spline at global parameter `t` in [0, 1].
pub fn spline_point(p: &[Vec2], control: bool, degree: u8, t: f64) -> Vec2 {
    let t = t.clamp(0.0, 1.0);
    if p.len() < 2 {
        return p.first().copied().unwrap_or_default();
    }
    if control {
        return bspline_at(p, degree.clamp(1, solidcraft_geom::MAX_BEZIER_DEGREE as u8) as usize, t);
    }
    let spans = fit_spans(p);
    let n = spans.len().max(1);
    let x = t * n as f64;
    let i = (x.floor() as usize).min(n - 1);
    match spans.get(i) {
        Some((p0, p1, t0, t1)) => hermite(*p0, *p1, *t0, *t1, x - i as f64),
        None => p[0],
    }
}

/// Curvature vector (towards the centre of curvature, magnitude 1/radius) of a parametric
/// curve at an end (`at_end`), by one-sided second-order differences.
pub fn end_curvature(f: &dyn Fn(f64) -> Vec2, at_end: bool) -> Vec2 {
    let h = 1e-3;
    let (t0, sgn) = if at_end { (1.0, -1.0) } else { (0.0, 1.0) };
    let (f0, f1, f2, f3) = (f(t0), f(t0 + sgn * h), f(t0 + sgn * 2.0 * h), f(t0 + sgn * 3.0 * h));
    let d1 = (f0 * -3.0 + f1 * 4.0 - f2) / (2.0 * h) * sgn;
    let d2 = (f0 * 2.0 - f1 * 5.0 + f2 * 4.0 - f3) / (h * h);
    let l2 = d1.len2();
    if l2 < 1e-24 {
        return Vec2::ZERO;
    }
    let t = d1 / l2.sqrt();
    (d2 - t * t.dot(d2)) / l2
}

/// The cubic Bézier pieces of a fit spline (exact: each span is a cubic Hermite).
pub fn fit_beziers(p: &[Vec2]) -> Vec<[Vec2; 4]> {
    fit_spans(p).into_iter().map(|(p0, p1, t0, t1)| [p0, p0 + t0 / 3.0, p1 - t1 / 3.0, p1]).collect()
}

/// Fit pieces in the explicitly selected interpolation mode.
pub fn fit_beziers_mode(p: &[Vec2], periodic: bool) -> crate::model::Result<Vec<[Vec2; 4]>> {
    if periodic { crate::periodic_beziers(p) } else { Ok(fit_beziers(p)) }
}
pub fn spline_point_mode(p: &[Vec2], control: bool, degree: u8, periodic: bool, t: f64) -> Vec2 {
    if !periodic {
        return spline_point(p, control, degree, t);
    }
    let Ok(pieces) = crate::periodic_beziers(p) else {
        return p.first().copied().unwrap_or_default();
    };
    let lengths: Vec<f64> = p.windows(2).map(|p| p[0].dist(p[1])).collect();
    let total = lengths.iter().sum::<f64>();
    let mut remaining = t.clamp(0.0, 1.0) * total;
    let mut i = 0;
    while i + 1 < pieces.len() && remaining > lengths[i] {
        remaining -= lengths[i];
        i += 1;
    }
    let Some([a, b, c, d]) = pieces.get(i).copied() else {
        return Vec2::ZERO;
    };
    let t = (remaining / lengths[i]).clamp(0.0, 1.0);
    a * (1.0 - t).powi(3) + b * (3.0 * t * (1.0 - t).powi(2)) + c * (3.0 * t * t * (1.0 - t)) + d * t.powi(3)
}
pub fn spline_polyline_mode(p: &[Vec2], control: bool, degree: u8, periodic: bool) -> Vec<Vec2> {
    if !periodic {
        return spline_polyline(p, control, degree);
    }
    let Ok(pieces) = crate::periodic_beziers(p) else {
        return Vec::new();
    };
    let mut out = Vec::with_capacity(pieces.len() * SPAN_SAMPLES + 1);
    for [a, b, c, d] in pieces {
        for i in 0..SPAN_SAMPLES {
            let t = i as f64 / SPAN_SAMPLES as f64;
            out.push(a * (1.0 - t).powi(3) + b * (3.0 * t * (1.0 - t).powi(2)) + c * (3.0 * t * t * (1.0 - t)) + d * t.powi(3));
        }
    }
    if let Some(first) = p.first() {
        out.push(*first);
    }
    out
}
pub fn spline_end_tangent_mode(p: &[Vec2], control: bool, periodic: bool, at_end: bool) -> Option<Vec2> {
    if !periodic {
        return spline_end_tangent(p, control, at_end);
    }
    let pieces = crate::periodic_beziers(p).ok()?;
    if at_end {
        let c = pieces.last()?;
        (c[3] - c[2]).normalized()
    } else {
        let c = pieces.first()?;
        (c[1] - c[0]).normalized()
    }
}

/// The Bézier pieces (each `degree + 1` points) of the clamped uniform B-spline over `ctrl`: the
/// sketch's control-point spline, [`nurbs_beziers`] on its evenly spaced knots.
pub fn bspline_beziers(ctrl: &[Vec2], degree: usize) -> Vec<Vec<Vec2>> {
    let n = ctrl.len();
    let k = degree.min(n.saturating_sub(1)).max(1);
    let spans = n.saturating_sub(k);
    let mut knots = vec![0.0; k + 1];
    for i in 1..spans {
        knots.push(i as f64 / spans as f64);
    }
    knots.extend(std::iter::repeat_n(1.0, k + 1));
    nurbs_beziers(ctrl, None, &knots, k).map(|pieces| pieces.into_iter().map(|b| b.points).collect()).unwrap_or_default()
}

/// One Bézier piece of a B-spline: its `degree + 1` control points and, for a rational curve,
/// their weights (only their ratios matter: (1, 0.7, 1) and (2, 1.4, 2) are the same curve).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierPiece {
    pub points: Vec<Vec2>,
    pub weights: Option<Vec<f64>>,
}

/// The Bézier pieces of the B-spline of `degree` over `ctrl` with the knot vector `knots`
/// (`ctrl.len() + degree + 1` finite, non-decreasing values: clamped or not, evenly spaced or
/// not) and, for a rational curve, a positive weight per control point. Boehm knot insertion, in
/// homogeneous coordinates when there are weights, until every knot of the curve's domain
/// `[knots[degree], knots[ctrl.len()]]` has multiplicity `degree`; then one piece per span of
/// non-zero length, in order. The pieces trace exactly the curve the knots and weights describe.
pub fn nurbs_beziers(ctrl: &[Vec2], weights: Option<&[f64]>, knots: &[f64], degree: usize) -> crate::model::Result<Vec<BezierPiece>> {
    let bad = |what: &str| crate::SketchError::Invalid(format!("B-spline: {what}"));
    let (n, p) = (ctrl.len(), degree);
    if !(1..=solidcraft_geom::MAX_BEZIER_DEGREE).contains(&p) {
        return Err(bad(&format!("degree {p} is not between 1 and {}", solidcraft_geom::MAX_BEZIER_DEGREE)));
    }
    if n < p + 1 || n > MAX_SPLINE_POINTS {
        return Err(bad(&format!("{n} control points for degree {p} (from {} to {MAX_SPLINE_POINTS})", p + 1)));
    }
    if knots.len() != n + p + 1 {
        return Err(bad(&format!("{} knots for {n} control points of degree {p} (needs {})", knots.len(), n + p + 1)));
    }
    if !knots.iter().all(|k| k.is_finite()) || knots.windows(2).any(|w| w[1] < w[0]) {
        return Err(bad("the knots must be finite and never decrease"));
    }
    if !ctrl.iter().all(|q| q.x.is_finite() && q.y.is_finite()) {
        return Err(bad("a control point is not finite"));
    }
    if let Some(w) = weights
        && (w.len() != n || !w.iter().all(|w| w.is_finite() && *w > 0.0))
    {
        return Err(bad("there must be one positive, finite weight per control point"));
    }
    let (Some(&lo), Some(&hi)) = (knots.get(p), knots.get(n)) else { return Err(bad("no domain")) };
    if hi <= lo {
        return Err(bad("the curve's domain is empty"));
    }
    // Homogeneous points (x·w, y·w, w); w = 1 throughout a non-rational curve.
    let w_at = |i: usize| weights.and_then(|w| w.get(i).copied()).unwrap_or(1.0);
    let mut pts: Vec<[f64; 3]> = ctrl.iter().enumerate().map(|(i, q)| [q.x * w_at(i), q.y * w_at(i), w_at(i)]).collect();
    let mut u = knots.to_vec();
    let mut values: Vec<f64> = u.iter().copied().filter(|x| (lo..=hi).contains(x)).collect();
    values.dedup();
    for v in values {
        for _ in u.iter().filter(|x| **x == v).count()..p {
            insert_knot(&mut u, &mut pts, p, v).ok_or_else(|| bad("knot insertion"))?;
        }
    }
    let mut out = Vec::new();
    for i in p..pts.len() {
        let (Some(&a), Some(&b)) = (u.get(i), u.get(i + 1)) else { break };
        if a < b && a >= lo && b <= hi {
            let piece = pts.get(i - p..=i).ok_or_else(|| bad("piece"))?;
            let points: Vec<Vec2> = piece.iter().map(|q| Vec2::new(q[0] / q[2], q[1] / q[2])).collect();
            // Finite inputs can still overflow together (a huge weight on a large coordinate).
            if !points.iter().all(|q| q.x.is_finite() && q.y.is_finite()) || !piece.iter().all(|q| q[2].is_finite() && q[2] > 0.0) {
                return Err(bad("the curve's points overflow"));
            }
            out.push(BezierPiece { points, weights: weights.map(|_| piece.iter().map(|q| q[2]).collect()) });
        }
    }
    Ok(out)
}

/// Insert the knot `v` once into the B-spline of `degree` with knots `u` and homogeneous control
/// points `pts` (Boehm): in the last span of non-zero length that starts at or before `v`, so a
/// knot at the domain's end goes into the span it closes. `None` when no span holds `v`.
fn insert_knot(u: &mut Vec<f64>, pts: &mut Vec<[f64; 3]>, degree: usize, v: f64) -> Option<()> {
    let p = degree;
    let k = (p..pts.len()).rev().find(|&j| u.get(j).is_some_and(|a| *a <= v) && u.get(j).zip(u.get(j + 1)).is_some_and(|(a, b)| a < b))?;
    let mut q = Vec::with_capacity(pts.len() + 1);
    for i in 0..=pts.len() {
        q.push(if i + p <= k {
            *pts.get(i)?
        } else if i > k {
            *pts.get(i - 1)?
        } else {
            let (lo, hi) = (*u.get(i)?, *u.get(i + p)?);
            let a = (v - lo) / (hi - lo);
            let (x, y) = (pts.get(i - 1)?, pts.get(i)?);
            [x[0] * (1.0 - a) + y[0] * a, x[1] * (1.0 - a) + y[1] * a, x[2] * (1.0 - a) + y[2] * a]
        });
    }
    u.insert(k + 1, v);
    *pts = q;
    Some(())
}

/// Polyline of a spline through (`control == false`) or over (`control`) the points.
pub fn spline_polyline(p: &[Vec2], control: bool, degree: u8) -> Vec<Vec2> {
    if p.len() < 2 {
        return p.to_vec();
    }
    if control {
        return bspline_polyline(p, degree.clamp(1, solidcraft_geom::MAX_BEZIER_DEGREE as u8) as usize);
    }
    let mut out = Vec::new();
    for (p0, p1, t0, t1) in fit_spans(p) {
        for i in 0..SPAN_SAMPLES {
            out.push(hermite(p0, p1, t0, t1, i as f64 / SPAN_SAMPLES as f64));
        }
    }
    if let Some(l) = p.last() {
        out.push(*l);
    }
    out
}

/// Unit tangent of a spline at its start (`at_end == false`) or end, pointing along the curve.
pub fn spline_end_tangent(p: &[Vec2], control: bool, at_end: bool) -> Option<Vec2> {
    let n = p.len();
    if n < 2 {
        return None;
    }
    if control {
        return if at_end { (p[n - 1] - p[n - 2]).normalized() } else { (p[1] - p[0]).normalized() };
    }
    let spans = fit_spans(p);
    if at_end { spans.last().and_then(|s| s.3.normalized()) } else { spans.first().and_then(|s| s.2.normalized()) }
}

/// Signed distance from `q` to a polyline (positive on the left of the nearest segment).
pub fn polyline_signed_dist(poly: &[Vec2], q: Vec2) -> f64 {
    let mut best = (f64::INFINITY, 1.0);
    for w in poly.windows(2) {
        let (a, b) = (w[0], w[1]);
        let d = b - a;
        let l2 = d.len2();
        let t = if l2 > 0.0 { ((q - a).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
        let dist = q.dist(a + d * t);
        if dist < best.0 {
            best = (dist, if d.cross(q - a) >= 0.0 { 1.0 } else { -1.0 });
        }
    }
    best.0 * best.1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_spline_passes_through_its_points_and_is_smooth() {
        let p = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0), Vec2::new(20.0, 0.0), Vec2::new(30.0, 8.0)];
        let poly = spline_polyline(&p, false, 3);
        for q in p {
            assert!(poly.iter().any(|x| x.dist(q) < 1e-9));
        }
        // A straight row of points stays straight.
        let line = [Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(3.0, 3.0)];
        assert!(spline_polyline(&line, false, 3).iter().all(|q| (q.x - q.y).abs() < 1e-9));
        let t = spline_end_tangent(&line, false, false).unwrap();
        assert!((t.x - t.y).abs() < 1e-9);
    }

    #[test]
    fn end_curvature_of_a_circle_quarter() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let rho = w / (1.0 + w);
        let (a, x, b) = (Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0));
        let k = end_curvature(&|t| conic_point(a, x, b, rho, t), false);
        assert!(k.dist(Vec2::new(-0.1, 0.0)) < 1e-5, "{k:?}");
        let k1 = end_curvature(&|t| conic_point(a, x, b, rho, t), true);
        assert!(k1.dist(Vec2::new(0.0, -0.1)) < 1e-5, "{k1:?}");
        // Fit splines have natural (straight) ends.
        let p = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0), Vec2::new(20.0, 0.0)];
        assert!(end_curvature(&|t| spline_point(&p, false, 3, t), false).len() < 1e-3);
    }

    #[test]
    fn spline_bezier_pieces_match_the_spline() {
        let p = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0), Vec2::new(20.0, -3.0), Vec2::new(30.0, 8.0), Vec2::new(35.0, 0.0)];
        let bez = |c: &[Vec2], t: f64| -> Vec2 {
            // de Casteljau.
            let mut v = c.to_vec();
            while v.len() > 1 {
                v = v.windows(2).map(|w| w[0] * (1.0 - t) + w[1] * t).collect();
            }
            v[0]
        };
        for degree in [2usize, 3] {
            let pieces = bspline_beziers(&p, degree);
            assert_eq!(pieces.len(), p.len() - degree, "degree {degree}");
            let spans = pieces.len();
            for (i, c) in pieces.iter().enumerate() {
                for t in [0.0, 0.3, 0.7, 1.0] {
                    let g = (i as f64 + t) / spans as f64;
                    let want = spline_point(&p, true, degree as u8, g);
                    assert!(bez(c, t).dist(want) < 1e-9, "degree {degree} piece {i} t {t}: {:?} vs {want:?}", bez(c, t));
                }
            }
        }
        for (i, c) in fit_beziers(&p).iter().enumerate() {
            let g = (i as f64 + 0.4) / 4.0;
            assert!(bez(c, 0.4).dist(spline_point(&p, false, 3, g)) < 1e-9);
        }
    }

    /// The point at `u` of the B-spline of `degree` over `ctrl` with `knots` and, for a rational
    /// curve, `weights`: de Boor's recurrence on homogeneous points, independent of knot insertion.
    fn de_boor(ctrl: &[Vec2], weights: Option<&[f64]>, knots: &[f64], degree: usize, u: f64) -> Vec2 {
        let n = ctrl.len();
        let s = (degree..n).rev().find(|&j| knots[j] <= u && knots[j] < knots[j + 1]).unwrap();
        let w = |i: usize| weights.map_or(1.0, |w| w[i]);
        let mut d: Vec<[f64; 3]> = (0..=degree)
            .map(|j| ctrl[j + s - degree])
            .zip(0..)
            .map(|(q, j)| {
                let wi = w(j + s - degree);
                [q.x * wi, q.y * wi, wi]
            })
            .collect();
        for r in 1..=degree {
            for j in (r..=degree).rev() {
                let i = j + s - degree;
                let a = (u - knots[i]) / (knots[i + degree + 1 - r] - knots[i]);
                let (prev, cur) = (d[j - 1], d[j]);
                d[j] = [0, 1, 2].map(|c| prev[c] * (1.0 - a) + cur[c] * a);
            }
        }
        Vec2::new(d[degree][0] / d[degree][2], d[degree][1] / d[degree][2])
    }

    /// A Bézier piece's point at `t` (rational de Casteljau when it has weights).
    fn piece_at(b: &BezierPiece, t: f64) -> Vec2 {
        let w = b.weights.clone().unwrap_or_else(|| vec![1.0; b.points.len()]);
        let mut v: Vec<[f64; 3]> = b.points.iter().zip(&w).map(|(q, w)| [q.x * w, q.y * w, *w]).collect();
        while v.len() > 1 {
            v = v.windows(2).map(|p| [0, 1, 2].map(|c| p[0][c] * (1.0 - t) + p[1][c] * t)).collect();
        }
        Vec2::new(v[0][0] / v[0][2], v[0][1] / v[0][2])
    }

    /// Every piece against de Boor on the file's own knots, spans in order (#9355).
    fn assert_pieces_trace(ctrl: &[Vec2], weights: Option<&[f64]>, knots: &[f64], degree: usize, want_pieces: usize) {
        let pieces = nurbs_beziers(ctrl, weights, knots, degree).unwrap();
        assert_eq!(pieces.len(), want_pieces);
        let n = ctrl.len();
        let mut spans: Vec<(f64, f64)> = (degree..n).map(|i| (knots[i], knots[i + 1])).filter(|(a, b)| a < b).collect();
        spans.dedup();
        assert_eq!(spans.len(), pieces.len());
        for (b, (a, e)) in pieces.iter().zip(&spans) {
            assert_eq!(b.points.len(), degree + 1);
            for t in [0.0, 0.17, 0.5, 0.83, 1.0] {
                let want = de_boor(ctrl, weights, knots, degree, a + t * (e - a) - if t == 1.0 { 1e-12 * (e - a) } else { 0.0 });
                assert!(piece_at(b, t).dist(want) < 1e-9, "span {a}..{e} t {t}: {:?} vs {want:?}", piece_at(b, t));
            }
        }
    }

    #[test]
    fn a_non_uniform_b_spline_splits_into_the_pieces_of_its_own_knots() {
        let ctrl = [(0.0, 0.0), (10.0, 6.0), (18.0, -4.0), (30.0, 9.0), (36.0, 1.0), (48.0, 5.0), (52.0, -2.0)].map(|(x, y)| Vec2::new(x, y));
        // Uneven gaps (0.1, 0.05, 0.45, 0.4), as a DXF from another program has them.
        assert_pieces_trace(&ctrl, None, &[0.0, 0.0, 0.0, 0.0, 0.1, 0.15, 0.6, 1.0, 1.0, 1.0, 1.0], 3, 4);
        // A double interior knot: one span fewer, and a piece boundary where the curve may kink.
        assert_pieces_trace(&ctrl, None, &[0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.7, 1.0, 1.0, 1.0, 1.0], 3, 3);
        // Read as evenly spaced, the same control points give another curve.
        let uneven = nurbs_beziers(&ctrl, None, &[0.0, 0.0, 0.0, 0.0, 0.1, 0.15, 0.6, 1.0, 1.0, 1.0, 1.0], 3).unwrap();
        let even = bspline_beziers(&ctrl, 3);
        assert!(piece_at(&uneven[2], 0.5).dist(piece_at(&BezierPiece { points: even[2].clone(), weights: None }, 0.5)) > 0.5);
    }

    #[test]
    fn evenly_spaced_knots_give_the_sketch_splines_pieces() {
        let ctrl = [(0.0, 0.0), (10.0, 5.0), (20.0, -3.0), (30.0, 8.0), (35.0, 0.0)].map(|(x, y)| Vec2::new(x, y));
        let general = nurbs_beziers(&ctrl, None, &[0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0], 3).unwrap();
        let pieces: Vec<Vec<Vec2>> = general.into_iter().map(|b| b.points).collect();
        assert_eq!(pieces, bspline_beziers(&ctrl, 3));
    }

    #[test]
    fn weighted_quadratics_trace_circles_exactly() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        // A quarter circle of radius 25: one rational quadratic piece.
        let quarter = [(25.0, 0.0), (25.0, 25.0), (0.0, 25.0)].map(|(x, y)| Vec2::new(x, y));
        let pieces = nurbs_beziers(&quarter, Some(&[1.0, h, 1.0]), &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2).unwrap();
        assert_eq!(pieces.len(), 1);
        for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
            assert!((piece_at(&pieces[0], t).len() - 25.0).abs() < 1e-12, "t {t}");
        }
        // The whole circle, as CAD programs export one: nine points, four pieces.
        let c = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (-1.0, 0.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0), (1.0, 0.0)];
        let ctrl = c.map(|(x, y)| Vec2::new(10.0 * x, 10.0 * y));
        let w = [1.0, h, 1.0, h, 1.0, h, 1.0, h, 1.0];
        let knots = [0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
        assert_pieces_trace(&ctrl, Some(&w), &knots, 2, 4);
        for b in nurbs_beziers(&ctrl, Some(&w), &knots, 2).unwrap() {
            for t in [0.0, 0.3, 0.6, 1.0] {
                assert!((piece_at(&b, t).len() - 10.0).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn an_unclamped_spline_is_cut_to_its_domain() {
        // A closed (periodic) cubic as DXF writes one: evenly spaced, unclamped knots; the curve
        // runs over [knots[3], knots[n]] and does not start or end on a control point.
        let ctrl = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.0), (20.0, 0.0), (20.0, 20.0)].map(|(x, y)| Vec2::new(x, y));
        let knots: Vec<f64> = (0..=10).map(f64::from).collect();
        assert_pieces_trace(&ctrl, None, &knots, 3, 4);
        let pieces = nurbs_beziers(&ctrl, None, &knots, 3).unwrap();
        let (first, last) = (piece_at(&pieces[0], 0.0), piece_at(&pieces[3], 1.0));
        assert!(first.dist(last) < 1e-9, "a closed curve ends where it starts: {first:?} {last:?}");
        assert!(ctrl.iter().all(|q| q.dist(first) > 1.0), "it starts inside its control polygon, not on a point");
    }

    #[test]
    fn hostile_b_splines_are_refused() {
        let ctrl = [Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(2.0, 0.0)];
        let clamped = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        for (weights, knots, degree) in [
            (None, &[0.0, 0.0, 0.0, 1.0, 1.0][..], 2),
            (None, &[0.0, 0.0, 1.0, 0.5, 1.0, 1.0][..], 2),
            (None, &[0.0, 0.0, 0.0, f64::NAN, 1.0, 1.0][..], 2),
            (None, &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0][..], 2),
            (Some(&[1.0, 0.0, 1.0][..]), &clamped[..], 2),
            (Some(&[1.0, 1.0][..]), &clamped[..], 2),
            (Some(&[1.0, f64::INFINITY, 1.0][..]), &clamped[..], 2),
            (None, &clamped[..], 0),
            (None, &clamped[..], 10),
            (None, &clamped[..], 3),
        ] {
            assert!(nurbs_beziers(&ctrl, weights, knots, degree).is_err(), "{weights:?} {knots:?} {degree}");
        }
        let huge = vec![Vec2::ZERO; MAX_SPLINE_POINTS + 1];
        let knots: Vec<f64> = (0..huge.len() + 4).map(|i| i as f64).collect();
        assert!(nurbs_beziers(&huge, None, &knots, 3).is_err());
        assert!(nurbs_beziers(&[Vec2::new(f64::NAN, 0.0), Vec2::ZERO], None, &[0.0, 0.0, 1.0, 1.0], 1).is_err());
        // Each finite, together out of range: a huge weight on a large coordinate.
        let far = [Vec2::new(1e9, 0.0), Vec2::new(1e9, 1e9), Vec2::new(0.0, 1e9)];
        assert!(nurbs_beziers(&far, Some(&[1e300, 1e300, 1e300]), &clamped, 2).is_err());
    }

    #[test]
    fn conic_rho_from_point_round_trips_through_the_midpoint() {
        let (a, apex, b) = (Vec2::new(0.0, 0.0), Vec2::new(20.0, 40.0), Vec2::new(40.0, 0.0));
        for rho in [0.1, 0.3, 0.5, 0.7, 0.9] {
            let mid_point = conic_point(a, apex, b, rho, 0.5);
            let back = conic_rho_from_point(a, apex, b, mid_point);
            assert!((back - rho).abs() < 1e-9, "rho {rho} back {back}");
        }
        // A point off the apex-chord-midpoint segment is projected onto it, not just measured
        // as a raw distance: well off to the side at the same height as a known rho still
        // recovers that rho.
        let on_axis = conic_point(a, apex, b, 0.6, 0.5);
        let off_axis = on_axis + Vec2::new(15.0, 0.0);
        assert!((conic_rho_from_point(a, apex, b, off_axis) - 0.6).abs() < 1e-9);
        // Degenerate apex (on the chord's midpoint): no segment to project onto, so the
        // fallback is a parabola rather than a division by zero.
        let mid = (a + b) * 0.5;
        assert_eq!(conic_rho_from_point(a, mid, b, Vec2::new(100.0, 100.0)), 0.5);
        // Always stays a valid, usable conic even for points far past either end.
        let far = mid + (apex - mid) * 50.0;
        let r = conic_rho_from_point(a, apex, b, far);
        assert!(r > 0.0 && r < 1.0);
    }

    #[test]
    fn control_spline_is_clamped_and_conic_hits_its_ends() {
        let p = [Vec2::new(0.0, 0.0), Vec2::new(0.0, 10.0), Vec2::new(10.0, 10.0), Vec2::new(10.0, 0.0)];
        let poly = spline_polyline(&p, true, 3);
        assert!(poly.first().unwrap().dist(p[0]) < 1e-12 && poly.last().unwrap().dist(p[3]) < 1e-12);
        // Cubic Bézier midpoint.
        let mid = poly[poly.len() / 2];
        assert!(mid.dist(Vec2::new(5.0, 7.5)) < 1e-9, "{mid:?}");
        let c = conic_polyline(
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
            std::f64::consts::FRAC_1_SQRT_2 / (1.0 + std::f64::consts::FRAC_1_SQRT_2),
        );
        // w = cos(45°): a quarter circle of radius 10 about the origin.
        assert!(c.iter().all(|q| (q.len() - 10.0).abs() < 1e-9));
        let e = ellipse_polyline(Vec2::ZERO, Vec2::new(5.0, 0.0), 2.0);
        assert!(e.iter().all(|q| ((q.x / 5.0).powi(2) + (q.y / 2.0).powi(2) - 1.0).abs() < 1e-9));
    }
}
