// =============================================================================
// File:     paths.rs
// Location: laser-cnc-controller/src/paths.rs
// Version:  v0.1.1
// Comments: Burn-path engine (app v0.2.1): turns one shape into the strokes
//           the laser burns, and merges shapes into one outline.
//           Polygon offsets and unions use the geo crate (BooleanOps, Buffer).
//           Strokes (canvas millimetres):
//           - Open line / polyline, width 0: one pass on the centerline.
//           - Trace (width > 0), N = round(width x density) lines (2 mm at
//             10 lines/mm = 20), pitch = width / N, open ends (no outline):
//               Along:  N passes parallel to the path, offset from the
//                       centerline; Square cap = flush with the end points,
//                       Round cap = each pass extended to the half circle.
//               Across: rails perpendicular to each segment at 1/density
//                       along the path, clipped to the trace outline and to
//                       the corner bisectors (no overlap at corners).
//           - Closed shape (rectangle, circle, closed polyline, merged
//             region, outline-font text):
//               Outline pass: one pass on the drawn edge (closed loop).
//               Border: band of the given width inside / centred on /
//                 outside the drawn edge. Along = N closed contour passes;
//                 Across = rails across the band.
//               Interior: fill of the area inside the border's inner edge
//                 (abut), extended into the border (overlap) or with an
//                 unburned ring (gap). Horizontal / Vertical / Crosshatch
//                 raster (line positions as v0.2.0: from one edge to the
//                 other, both edges included) or Concentric contours.
//           - Raster passes alternate direction (bidirectional) unless the
//             shape is unidirectional.
//           - Overscan: every open pass gets a laser-off lead-in and lead-out
//             of the given length along its end directions.
//           Merge: shapes qualify when they share an edge of non-zero length
//           or overlap; corner-only contact and gaps do not. Coordinates are
//           rounded to 0.0001 mm first so snapped edges count as shared.
//           v0.1.1 (app v0.2.3): every pass is cleaned before it is emitted:
//           points closer than 0.05 mm to the previous kept point are dropped
//           (the end point is always kept), then points that deviate less than
//           0.002 mm from a straight line are removed (Ramer-Douglas-Peucker).
//           Round corners, small circles and offset arcs no longer produce
//           runs of 0.006 mm moves that GRBL 0.9 cannot parse and plan as fast
//           as the planner model assumes. Passes shorter than 0.001 mm are
//           dropped.
// =============================================================================
use geo::{BooleanOps, Buffer, Coord, LineString, MultiPolygon, Polygon};
use geo::algorithm::buffer::{BufferStyle, LineCap, LineJoin};
use crate::font;
use crate::geometry::{Align, Cap, Fill, Join, Layer, Operation, Poly, Pt, Relation, Shape, ShapeKind, StrokeDir, TextFont};

pub type MP = MultiPolygon<f64>;
type P = (f64, f64);

/// Corners sharper than this (radians) are bevelled instead of mitred.
const MITER_MIN_ANGLE: f64 = 0.25;
/// Mitre length limit for parallel trace passes, in multiples of the pass offset.
const MITER_LIMIT: f64 = 4.0;
/// Round joins and caps: segment length / radius.
const ROUND_STEP: f64 = 0.12;
/// Circles: maximum chord error, mm.
const CIRCLE_TOL: f64 = 0.002;
/// Raster segments shorter than this are dropped, mm.
const MIN_SEG: f64 = 0.001;
/// Merge: coordinates rounded to this grid, mm.
const MERGE_GRID: f64 = 1e-4;
/// Pass cleanup: shortest kept move, and largest deviation from a straight line removed, mm.
const MIN_STEP: f64 = 0.05;
const STRAIGHT_TOL: f64 = 0.002;

/// One continuous laser-on pass. `closed` passes end where they start (no overscan).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Stroke { pub pts: Vec<Pt>, pub closed: bool, pub lead_in: Option<Pt>, pub lead_out: Option<Pt> }
fn open(pts: Vec<P>) -> Stroke { Stroke { pts: pts.into_iter().map(f32p).collect(), ..Default::default() } }
fn closed_ring(r: &[P]) -> Stroke {
    let mut pts: Vec<Pt> = r.iter().map(|&p| f32p(p)).collect();
    if let Some(&f) = pts.first() { pts.push(f); }
    Stroke { pts, closed: true, ..Default::default() }
}

// ---------------------------------------------------------------- small vector helpers
fn f32p(p: P) -> Pt { (p.0 as f32, p.1 as f32) }
fn f64p(p: Pt) -> P { (p.0 as f64, p.1 as f64) }
fn add(a: P, b: P) -> P { (a.0 + b.0, a.1 + b.1) }
fn sub(a: P, b: P) -> P { (a.0 - b.0, a.1 - b.1) }
fn mul(a: P, k: f64) -> P { (a.0 * k, a.1 * k) }
fn dot(a: P, b: P) -> f64 { a.0 * b.0 + a.1 * b.1 }
fn cross(a: P, b: P) -> f64 { a.0 * b.1 - a.1 * b.0 }
fn len(a: P) -> f64 { dot(a, a).sqrt() }
fn unit(a: P) -> Option<P> { let l = len(a); if l > 1e-12 { Some((a.0 / l, a.1 / l)) } else { None } }

// ---------------------------------------------------------------- polygon conversion
fn ring_ls(r: &[Pt]) -> LineString<f64> {
    let mut v: Vec<Coord<f64>> = r.iter().map(|p| Coord { x: p.0 as f64, y: p.1 as f64 }).collect();
    if let (Some(a), Some(b)) = (v.first().copied(), v.last().copied()) { if a != b { v.push(a); } }
    LineString(v)
}
fn polygon(outer: &[Pt], holes: &[Vec<Pt>]) -> Polygon<f64> { Polygon::new(ring_ls(outer), holes.iter().map(|h| ring_ls(h)).collect()) }
/// Every ring (exteriors and holes), without the closing duplicate point.
pub fn rings(mp: &MP) -> Vec<Vec<P>> {
    let mut out = vec![];
    for p in &mp.0 {
        for r in std::iter::once(p.exterior()).chain(p.interiors().iter()) {
            let mut v: Vec<P> = r.0.iter().map(|c| (c.x, c.y)).collect();
            if v.len() > 1 && v[0] == v[v.len() - 1] { v.pop(); }
            if v.len() >= 3 { out.push(v); }
        }
    }
    out
}
/// Region polygons (f32, rings open) from a MultiPolygon.
pub fn to_polys(mp: &MP) -> Vec<Poly> {
    let strip = |r: &LineString<f64>| { let mut v: Vec<Pt> = r.0.iter().map(|c| (c.x as f32, c.y as f32)).collect(); if v.len() > 1 && v[0] == v[v.len() - 1] { v.pop(); } v };
    mp.0.iter().map(|p| Poly { outer: strip(p.exterior()), holes: p.interiors().iter().map(strip).collect() }).collect()
}
fn line_join(j: Join) -> LineJoin<f64> { match j { Join::Miter => LineJoin::Miter(MITER_MIN_ANGLE), Join::Round => LineJoin::Round(ROUND_STEP) } }
/// Grow (d > 0) or shrink (d < 0) an area.
fn offset(mp: &MP, d: f64, join: Join) -> MP {
    if d.abs() < 1e-9 { return mp.clone(); }
    if mp.0.is_empty() { return MultiPolygon(vec![]); }
    mp.buffer_with_style(BufferStyle::new(d).line_join(line_join(join)))
}
/// Polygon ring approximating an ellipse, starting at angle 0, counter-clockwise.
fn ellipse(cx: f64, cy: f64, rx: f64, ry: f64) -> Vec<Pt> {
    let r = rx.max(ry).max(1e-6);
    let step = 2.0 * (1.0 - (CIRCLE_TOL / r).min(1.0)).acos();
    let n = ((std::f64::consts::TAU / step.max(1e-6)).ceil() as usize).clamp(48, 1440);
    (0..n).map(|i| { let t = std::f64::consts::TAU * i as f64 / n as f64; ((cx + rx * t.cos()) as f32, (cy + ry * t.sin()) as f32) }).collect()
}

/// Closed area of a shape: rectangle, circle, closed polyline, merged region, outline-font text.
pub fn region(s: &Shape) -> Option<MP> {
    let mp = match s.kind {
        ShapeKind::Rectangle => {
            let (x0, y0, x1, y1) = s.extent();
            if x1 - x0 < 1e-6 || y1 - y0 < 1e-6 { return None; }
            MultiPolygon(vec![polygon(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)], &[])])
        }
        ShapeKind::Circle => {
            let (x0, y0, x1, y1) = s.extent();
            let (rx, ry) = ((x1 - x0) as f64 / 2.0, (y1 - y0) as f64 / 2.0);
            if rx < 1e-6 || ry < 1e-6 { return None; }
            MultiPolygon(vec![polygon(&ellipse((x0 + x1) as f64 / 2.0, (y0 + y1) as f64 / 2.0, rx, ry), &[])])
        }
        ShapeKind::Polyline if s.closed && s.points.len() >= 3 => MultiPolygon(vec![polygon(&s.points, &[])]).union(&MultiPolygon(vec![])),
        ShapeKind::Region => MultiPolygon(s.polys.iter().filter(|p| p.outer.len() >= 3).map(|p| polygon(&p.outer, &p.holes)).collect()),
        ShapeKind::Text if s.font == TextFont::Outline => {
            let mut acc: MP = MultiPolygon(vec![]);
            for r in font::outline_rings(&s.text, s.x, s.y, s.h) { acc = acc.xor(&MultiPolygon(vec![polygon(&r, &[])])); }
            acc
        }
        _ => return None,
    };
    if mp.0.is_empty() { None } else { Some(mp) }
}

// ---------------------------------------------------------------- fills
/// Raster lines across an area. `u` = line direction, `v` = step direction (orthonormal).
/// Positions from the lowest to the highest extent in steps of `pitch`, both edges included (v0.2.0 rule).
fn raster(area: &MP, u: P, v: P, pitch: f64, bidir: bool) -> Vec<Stroke> {
    let tr: Vec<Vec<P>> = rings(area).iter().map(|r| r.iter().map(|&p| (dot(p, u), dot(p, v))).collect()).collect();
    let (mut bmin, mut bmax) = (f64::MAX, f64::MIN);
    for r in &tr { for p in r { bmin = bmin.min(p.1); bmax = bmax.max(p.1); } }
    let dim = bmax - bmin;
    if tr.is_empty() || dim < 4e-6 || pitch <= 0.0 { return vec![]; }
    let n = (((dim / pitch) + 1e-6).floor() as usize).min(20000);
    let mut out = vec![];
    for i in 0..=n {
        let b = (bmin + i as f64 * pitch).min(bmax);
        let be = b.clamp(bmin + 1e-6, bmax - 1e-6);
        let mut xs: Vec<f64> = vec![];
        for r in &tr {
            for k in 0..r.len() {
                let (p, q) = (r[k], r[(k + 1) % r.len()]);
                if (p.1 <= be) != (q.1 <= be) { xs.push(p.0 + (be - p.1) * (q.0 - p.0) / (q.1 - p.1)); }
            }
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut segs: Vec<(f64, f64)> = xs.chunks_exact(2).map(|c| (c[0], c[1])).filter(|s| s.1 - s.0 > MIN_SEG).collect();
        if bidir && i % 2 == 1 { segs.reverse(); for s in &mut segs { *s = (s.1, s.0); } }
        for (a0, a1) in segs { out.push(open(vec![add(mul(u, a0), mul(v, b)), add(mul(u, a1), mul(v, b))])); }
    }
    out
}
/// Closed contour passes: rings of the area shrunk by start, start + pitch, … (count passes, or until empty).
fn contours(area: &MP, start: f64, pitch: f64, count: Option<usize>, join: Join) -> Vec<Stroke> {
    let mut out = vec![];
    if pitch <= 0.0 { return out; }
    for k in 0..count.unwrap_or(5000).min(5000) {
        let m = offset(area, -(start + k as f64 * pitch), join);
        let rs = rings(&m);
        if rs.is_empty() { break; }
        for r in rs { out.push(closed_ring(&r)); }
    }
    out
}
/// Interior fill with the shape's pattern and density.
fn fill(area: &MP, s: &Shape, bidir: bool) -> Vec<Stroke> {
    let pitch = (1.0 / s.lines_per_mm.clamp(0.1, 100.0) as f64).max(0.01);
    let (h, v) = (((1.0, 0.0), (0.0, 1.0)), ((0.0, 1.0), (1.0, 0.0)));
    match s.fill {
        Fill::Horizontal => raster(area, h.0, h.1, pitch, bidir),
        Fill::Vertical => raster(area, v.0, v.1, pitch, bidir),
        Fill::Crosshatch => { let mut o = raster(area, h.0, h.1, pitch, bidir); o.extend(raster(area, v.0, v.1, pitch, bidir)); o }
        Fill::Concentric => contours(area, 0.0, pitch, None, s.join),
    }
}

// ---------------------------------------------------------------- segment clipping
fn inside(rings: &[Vec<P>], p: P) -> bool {
    let mut c = false;
    for r in rings {
        for k in 0..r.len() {
            let (a, b) = (r[k], r[(k + 1) % r.len()]);
            if (a.1 > p.1) != (b.1 > p.1) && p.0 < a.0 + (p.1 - a.1) * (b.0 - a.0) / (b.1 - a.1) { c = !c; }
        }
    }
    c
}
/// Parts of p0→p1 with t in [t0, t1] that lie inside the rings (even-odd), as t intervals.
fn clip_segment(rings: &[Vec<P>], p0: P, p1: P, t0: f64, t1: f64) -> Vec<(f64, f64)> {
    let d = sub(p1, p0);
    let mut ts = vec![t0, t1];
    for r in rings {
        for k in 0..r.len() {
            let (q0, q1) = (r[k], r[(k + 1) % r.len()]);
            let e = sub(q1, q0);
            let den = cross(d, e);
            if den.abs() < 1e-14 { continue; }
            let w = sub(q0, p0);
            let t = cross(w, e) / den;
            let u = cross(w, d) / den;
            if t > t0 && t < t1 && (-1e-12..=1.0 + 1e-12).contains(&u) { ts.push(t); }
        }
    }
    ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mut out: Vec<(f64, f64)> = vec![];
    for w in ts.windows(2) {
        if w[1] - w[0] < 1e-12 { continue; }
        if inside(rings, add(p0, mul(d, 0.5 * (w[0] + w[1])))) {
            match out.last_mut() { Some(l) if (l.1 - w[0]).abs() < 1e-12 => l.1 = w[1], _ => out.push((w[0], w[1])) }
        }
    }
    out
}

// ---------------------------------------------------------------- traces
fn dedupe(path: &[P]) -> Vec<P> {
    let mut v: Vec<P> = vec![];
    for &p in path { if v.last().is_none_or(|l| len(sub(p, *l)) > 1e-9) { v.push(p); } }
    v
}
/// Trace outline (open path buffered by half the width).
fn strip(path: &[P], half: f64, cap: Cap, join: Join) -> MP {
    let ls = LineString::from(path.to_vec());
    let lc = match cap { Cap::Square => LineCap::Butt, Cap::Round => LineCap::Round(ROUND_STEP) };
    ls.buffer_with_style(BufferStyle::new(half).line_cap(lc).line_join(line_join(join)))
}
/// One pass parallel to an open path at signed offset d (left positive). Round caps extend the
/// pass to the half circle of radius `half`.
fn offset_open(path: &[P], d: f64, join: Join, cap: Cap, half: f64) -> Vec<P> {
    let n = path.len();
    let dirs: Vec<P> = (0..n - 1).map(|i| unit(sub(path[i + 1], path[i])).unwrap_or((1.0, 0.0))).collect();
    let nrm: Vec<P> = dirs.iter().map(|d| (-d.1, d.0)).collect();
    let ext = if cap == Cap::Round { (half * half - d * d).max(0.0).sqrt() } else { 0.0 };
    let mut pts = vec![sub(add(path[0], mul(nrm[0], d)), mul(dirs[0], ext))];
    for i in 1..n - 1 {
        let v = path[i];
        let (pa, pb) = (add(v, mul(nrm[i - 1], d)), add(v, mul(nrm[i], d)));
        if len(sub(pa, pb)) < 1e-9 { pts.push(pa); continue; }
        let turn = cross(dirs[i - 1], dirs[i]);
        let outer = turn * d < 0.0;
        if outer && join == Join::Round {
            let (a0, a1) = ((pa.1 - v.1).atan2(pa.0 - v.0), (pb.1 - v.1).atan2(pb.0 - v.0));
            let mut sweep = a1 - a0;
            while sweep > std::f64::consts::PI { sweep -= std::f64::consts::TAU; }
            while sweep < -std::f64::consts::PI { sweep += std::f64::consts::TAU; }
            let steps = ((sweep.abs() / 0.2).ceil() as usize).max(1);
            for k in 0..=steps { let a = a0 + sweep * k as f64 / steps as f64; pts.push((v.0 + d.abs() * a.cos(), v.1 + d.abs() * a.sin())); }
        } else {
            let den = cross(dirs[i - 1], dirs[i]);
            if den.abs() < 1e-12 { pts.push(pa); pts.push(pb); continue; }
            let x = add(pa, mul(dirs[i - 1], cross(sub(pb, pa), dirs[i]) / den));
            if outer && len(sub(x, v)) > d.abs() * MITER_LIMIT { pts.push(pa); pts.push(pb); } else { pts.push(x); }
        }
    }
    pts.push(add(add(path[n - 1], mul(nrm[n - 2], d)), mul(dirs[n - 2], ext)));
    pts
}
/// Rails across a path: perpendicular to each segment, every `pitch` along it, `half` to each side,
/// clipped to `clip` and to the bisectors of the neighbouring corners.
fn rails(path: &[P], closed: bool, half: f64, pitch: f64, clip: &MP, bidir: bool) -> Vec<Stroke> {
    let path = dedupe(path);
    let mut path = path;
    if closed && path.len() > 2 && len(sub(path[0], path[path.len() - 1])) < 1e-9 { path.pop(); }
    let n = path.len();
    if n < 2 || pitch <= 0.0 { return vec![]; }
    let m = if closed { n } else { n - 1 };
    let clip_rings = rings(clip);
    let dir = |i: usize| unit(sub(path[(i + 1) % n], path[i % n]));
    let ext = half * MITER_LIMIT + pitch;
    let mut out = vec![];
    let mut idx = 0usize;
    for i in 0..m {
        let (a, b) = (path[i], path[(i + 1) % n]);
        let Some(d) = dir(i) else { continue; };
        let l = len(sub(b, a));
        let nrm = (-d.1, d.0);
        let prev = if closed || i > 0 { dir((i + m - 1) % m) } else { None };
        let next = if closed || i + 1 < m { dir((i + 1) % m) } else { None };
        let hs = prev.and_then(|p| unit(add(p, d)));
        let he = next.and_then(|q| unit(add(d, q)));
        let count = ((l / pitch).round() as i64).max(1);
        let pp = l / count as f64;
        let kmin = -((ext / pp).ceil() as i64) - 1;
        let kmax = count + (ext / pp).ceil() as i64 + 1;
        for k in kmin..kmax {
            let st = (k as f64 + 0.5) * pp;
            let c = add(a, mul(d, st));
            let (p0, p1) = (sub(c, mul(nrm, half)), add(c, mul(nrm, half)));
            // Half-plane limits (linear in t along p0→p1).
            let (mut t0, mut t1) = (0.0f64, 1.0f64);
            let mut keep = true;
            for (h, origin, sign) in [(hs, a, 1.0), (he, b, -1.0)] {
                let Some(h) = h else { continue; };
                let f0 = sign * dot(sub(p0, origin), h);
                let f1 = sign * dot(sub(p1, origin), h);
                if f0 < 0.0 && f1 < 0.0 { keep = false; break; }
                if (f0 < 0.0) != (f1 < 0.0) {
                    let t = f0 / (f0 - f1);
                    if f0 < 0.0 { t0 = t0.max(t); } else { t1 = t1.min(t); }
                }
            }
            if !keep || t1 - t0 < 1e-9 { continue; }
            for (s0, s1) in clip_segment(&clip_rings, p0, p1, t0, t1) {
                let (q0, q1) = (add(p0, mul(sub(p1, p0), s0)), add(p0, mul(sub(p1, p0), s1)));
                if len(sub(q1, q0)) <= MIN_SEG { continue; }
                out.push(if bidir && idx % 2 == 1 { open(vec![q1, q0]) } else { open(vec![q0, q1]) });
                idx += 1;
            }
        }
    }
    out
}
/// Ramer-Douglas-Peucker: keep the points that deviate more than `tol` from the chord.
fn rdp(pts: &[P], tol: f64, out: &mut Vec<P>) {
    let n = pts.len();
    if n < 3 { return; }
    let (a, b) = (pts[0], pts[n - 1]);
    let ab = sub(b, a);
    let l = len(ab);
    let mut far = (0usize, 0.0f64);
    for (i, &p) in pts.iter().enumerate().take(n - 1).skip(1) {
        let d = if l > 1e-12 { (cross(ab, sub(p, a)) / l).abs() } else { len(sub(p, a)) };
        if d > far.1 { far = (i, d); }
    }
    if far.1 > tol {
        rdp(&pts[..=far.0], tol, out);
        out.push(pts[far.0]);
        rdp(&pts[far.0..], tol, out);
    }
}
/// Clean one pass: drop moves shorter than MIN_STEP (the end point is kept), then remove points
/// within STRAIGHT_TOL of a straight line. Closed passes keep their closing point.
fn clean(pts: &[Pt]) -> Vec<Pt> {
    if pts.len() <= 2 { return pts.to_vec(); }
    let p: Vec<P> = pts.iter().map(|&q| f64p(q)).collect();
    let last = p[p.len() - 1];
    let mut kept: Vec<P> = vec![p[0]];
    for &q in &p[1..p.len() - 1] { if len(sub(q, kept[kept.len() - 1])) >= MIN_STEP { kept.push(q); } }
    // The end point replaces a kept point that sits closer than MIN_STEP to it.
    if kept.len() > 1 && len(sub(last, kept[kept.len() - 1])) < MIN_STEP { kept.pop(); }
    kept.push(last);
    let mut out = vec![kept[0]];
    rdp(&kept, STRAIGHT_TOL, &mut out);
    out.push(last);
    out.into_iter().map(f32p).collect()
}
/// Number of parallel lines in a trace or border of width w at the given density (at least 1).
pub fn line_count(w: f32, lpm: f32) -> usize { ((w as f64 * lpm as f64).round() as usize).max(1) }
fn trace(path: &[P], s: &Shape, bidir: bool) -> Vec<Stroke> {
    let path = dedupe(path);
    if path.len() < 2 { return vec![]; }
    let w = s.width as f64;
    let half = w / 2.0;
    let lpm = s.stroke_lpm.clamp(0.1, 100.0) as f64;
    match s.stroke_dir {
        StrokeDir::Along => {
            let n = line_count(s.width, s.stroke_lpm);
            let pitch = w / n as f64;
            (0..n).map(|k| {
                let mut pts = offset_open(&path, -half + (k as f64 + 0.5) * pitch, s.join, s.cap, half);
                if bidir && k % 2 == 1 { pts.reverse(); }
                open(pts)
            }).collect()
        }
        StrokeDir::Across => rails(&path, false, half, 1.0 / lpm, &strip(&path, half, s.cap, s.join), bidir),
    }
}

// ---------------------------------------------------------------- borders
/// Outer area of the border band (the drawn region grown by the alignment).
fn border_outer(region: &MP, s: &Shape) -> MP {
    let w = s.border_w as f64;
    match s.border_align { Align::Inside => region.clone(), Align::Centered => offset(region, w / 2.0, s.join), Align::Outside => offset(region, w, s.join) }
}

/// Strokes for one pass of a shape, in burn order. Shapes on Notes / Guides burn nothing.
pub fn shape_strokes(s: &Shape) -> Vec<Stroke> {
    if s.layer != Layer::Burn { return vec![]; }
    let bidir = !s.unidirectional;
    let mut out: Vec<Stroke> = vec![];
    if s.kind == ShapeKind::Text && s.font == TextFont::Stroke {
        for p in font::stroke_text(&s.text, s.x, s.y, s.h) { out.push(open(p.into_iter().map(f64p).collect())); }
    } else if s.is_trace() {
        let path: Vec<P> = s.path().into_iter().map(f64p).collect();
        if s.width > 1e-4 { out.extend(trace(&path, s, bidir)); } else { let p = dedupe(&path); if p.len() >= 2 { out.push(open(p)); } }
    } else if let Some(region) = region(s) {
        if matches!(s.operation, Operation::Outline | Operation::Both) { for r in rings(&region) { out.push(closed_ring(&r)); } }
        let mut interior = region.clone();
        if s.border_w > 1e-4 {
            let w = s.border_w as f64;
            let outer = border_outer(&region, s);
            let inner = offset(&outer, -w, s.join);
            let n = line_count(s.border_w, s.stroke_lpm);
            match s.stroke_dir {
                StrokeDir::Along => out.extend(contours(&outer, w / n as f64 / 2.0, w / n as f64, Some(n), s.join)),
                StrokeDir::Across => {
                    let band = outer.difference(&inner);
                    let lpm = s.stroke_lpm.clamp(0.1, 100.0) as f64;
                    for r in rings(&offset(&outer, -w / 2.0, s.join)) { out.extend(rails(&r, true, w / 2.0, 1.0 / lpm, &band, bidir)); }
                }
            }
            let rel = (s.relation_mm as f64).clamp(0.0, w);
            interior = match s.relation { Relation::Abut => inner, Relation::Overlap => offset(&outer, -(w - rel), s.join), Relation::Gap => offset(&outer, -(w + rel), s.join) };
        }
        if matches!(s.operation, Operation::Fill | Operation::Both) { out.extend(fill(&interior, s, bidir)); }
    }
    for st in &mut out { st.pts = clean(&st.pts); }
    out.retain(|st| st.pts.len() >= 2 && st.pts.windows(2).map(|w| len(sub(f64p(w[1]), f64p(w[0])))).sum::<f64>() > MIN_SEG);
    let ov = s.overscan.max(0.0);
    if ov > 1e-4 {
        for st in &mut out {
            if st.closed || st.pts.len() < 2 { continue; }
            let (a, b) = (f64p(st.pts[0]), f64p(st.pts[1]));
            let (y, z) = (f64p(st.pts[st.pts.len() - 2]), f64p(st.pts[st.pts.len() - 1]));
            if let Some(d) = unit(sub(b, a)) { st.lead_in = Some(f32p(sub(a, mul(d, ov as f64)))); }
            if let Some(d) = unit(sub(z, y)) { st.lead_out = Some(f32p(add(z, mul(d, ov as f64)))); }
        }
    }
    out
}

/// Outlines that show a shape's burned extent on the canvas: the trace outline, or the border band
/// edges and the interior edge.
pub fn display_rings(s: &Shape) -> Vec<Vec<Pt>> {
    let conv = |mp: &MP| rings(mp).into_iter().map(|r| r.into_iter().map(f32p).collect::<Vec<Pt>>()).collect::<Vec<_>>();
    if s.is_trace() && s.width > 1e-4 {
        let path = dedupe(&s.path().into_iter().map(f64p).collect::<Vec<P>>());
        if path.len() < 2 { return vec![]; }
        return conv(&strip(&path, s.width as f64 / 2.0, s.cap, s.join));
    }
    if s.border_w > 1e-4 { if let Some(region) = region(s) {
        let outer = border_outer(&region, s);
        let mut v = conv(&outer);
        v.extend(conv(&offset(&outer, -(s.border_w as f64), s.join)));
        return v;
    }}
    vec![]
}

/// Merge closed shapes into one outline. Err: message and the indices (into `shapes`) of the
/// shapes that block the merge (not closed, or not touching the rest).
pub fn merge(shapes: &[&Shape]) -> Result<Vec<Poly>, (String, Vec<usize>)> {
    if shapes.len() < 2 { return Err(("Select two or more touching shapes to merge".into(), vec![])); }
    let bad: Vec<usize> = shapes.iter().enumerate().filter(|(_, s)| !s.is_closed() || s.kind == ShapeKind::Text).map(|(i, _)| i).collect();
    if !bad.is_empty() { return Err(("Merge needs closed shapes: rectangle, circle, closed polyline or merged region".into(), bad)); }
    let snap = |v: f64| (v / MERGE_GRID).round() * MERGE_GRID;
    let mut regs: Vec<MP> = vec![];
    for s in shapes {
        let r = region(s).unwrap_or(MultiPolygon(vec![]));
        let polys: Vec<Polygon<f64>> = r.0.iter().map(|p| {
            let f = |ls: &LineString<f64>| LineString(ls.0.iter().map(|c| Coord { x: snap(c.x), y: snap(c.y) }).collect());
            Polygon::new(f(p.exterior()), p.interiors().iter().map(f).collect())
        }).collect();
        regs.push(MultiPolygon(polys));
    }
    let n = regs.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut Vec<usize>, i: usize) -> usize { let mut r = i; while p[r] != r { r = p[r]; } let mut j = i; while p[j] != r { let nx = p[j]; p[j] = r; j = nx; } r }
    let bbox = |s: &Shape| s.extent();
    for i in 0..n {
        for j in i + 1..n {
            let (a, b) = (bbox(shapes[i]), bbox(shapes[j]));
            if a.0 > b.2 + 1e-3 || b.0 > a.2 + 1e-3 || a.1 > b.3 + 1e-3 || b.1 > a.3 + 1e-3 { continue; }
            let u = regs[i].union(&regs[j]);
            if !u.0.is_empty() && u.0.len() < regs[i].0.len() + regs[j].0.len() { let (ri, rj) = (find(&mut parent, i), find(&mut parent, j)); parent[ri] = rj; }
        }
    }
    let roots: Vec<usize> = (0..n).map(|i| find(&mut parent, i)).collect();
    let mut best = roots[0]; let mut best_n = 0;
    for &r in &roots { let c = roots.iter().filter(|&&x| x == r).count(); if c > best_n { best = r; best_n = c; } }
    if best_n < n {
        let others: Vec<usize> = (0..n).filter(|&i| roots[i] != best).collect();
        return Err(("Not merged: the highlighted shapes do not share an edge with, or overlap, the rest (corner contact does not count)".into(), others));
    }
    let mut acc = regs[0].clone();
    for r in &regs[1..] { acc = acc.union(r); }
    Ok(to_polys(&acc))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn rect(x: f32, y: f32, w: f32, h: f32) -> Shape { Shape::rectangle(x, y, w, h) }

    #[test]
    fn merge_rules() {
        let bar = rect(0.0, 0.0, 30.0, 4.0);
        let finger = rect(5.0, 4.0, 2.0, 10.0);   // shares part of the bar's top edge
        let overlap = rect(20.0, 2.0, 2.0, 10.0); // overlaps the bar
        let corner = rect(30.0, 4.0, 3.0, 3.0);   // touches the bar's corner only
        let apart = rect(40.0, 0.0, 3.0, 3.0);
        let polys = merge(&[&bar, &finger, &overlap]).expect("edge and overlap merge");
        assert_eq!(polys.len(), 1);
        let (_, bad) = merge(&[&bar, &finger, &corner]).unwrap_err();
        assert_eq!(bad, vec![2]);
        let (_, bad) = merge(&[&bar, &apart]).unwrap_err();
        assert_eq!(bad.len(), 1);
        // Snapped edges with float noise still count as shared.
        let near = rect(5.0, 4.000_000_2, 2.0, 10.0);
        assert!(merge(&[&bar, &near]).is_ok());
    }

    #[test]
    fn merged_comb_fill_runs_through_the_joints() {
        let bar = rect(0.0, 0.0, 30.0, 4.0);
        let f1 = rect(0.0, 4.0, 2.0, 10.0);
        let f2 = rect(10.0, 4.0, 2.0, 10.0);
        let mut m = Shape::region(merge(&[&bar, &f1, &f2]).unwrap());
        m.lines_per_mm = 1.0;
        let strokes = shape_strokes(&m);
        // y = 0..3: full-width lines (4); y = 4..14 (the bar's top edge and above): two finger
        // segments per line (11 x 2). No seam: the joints are inside one outline.
        let full = strokes.iter().filter(|s| (s.pts[0].0 - s.pts[1].0).abs() > 29.0).count();
        assert_eq!(full, 4);
        assert_eq!(strokes.len(), 4 + 11 * 2);
    }

    #[test]
    fn trace_along_line_count_and_open_ends() {
        let mut s = Shape::line((0.0, 0.0), (12.0, 0.0));
        s.width = 2.0; s.stroke_lpm = 10.0;
        let st = shape_strokes(&s);
        assert_eq!(line_count(2.0, 10.0), 20);
        assert_eq!(st.len(), 20);
        assert!(st.iter().all(|s| !s.closed && s.pts.len() == 2));
        // Square cap: flush with the end points; passes between -1 and +1 mm.
        let ys: Vec<f32> = st.iter().map(|s| s.pts[0].1).collect();
        assert!((ys.iter().cloned().fold(f32::MAX, f32::min) + 0.95).abs() < 1e-4);
        assert!(st.iter().all(|s| s.pts.iter().all(|p| p.0 >= -1e-4 && p.0 <= 12.0001)));
        // Round cap: the centre pass reaches the half circle (1 mm beyond each end).
        s.cap = Cap::Round;
        let st = shape_strokes(&s);
        let reach = st.iter().flat_map(|s| s.pts.iter()).map(|p| p.0).fold(f32::MIN, f32::max);
        assert!(reach > 12.95 && reach <= 13.0001, "reach {reach}");
    }

    #[test]
    fn trace_across_rails() {
        let mut s = Shape::line((0.0, 0.0), (0.0, 12.0));
        s.width = 2.0; s.stroke_lpm = 10.0; s.stroke_dir = StrokeDir::Across;
        let st = shape_strokes(&s);
        assert_eq!(st.len(), 120);
        assert!(st.iter().all(|r| ((r.pts[0].0 - r.pts[1].0).abs() - 2.0).abs() < 1e-3));
        assert!(st.iter().all(|r| r.pts[0].1 > 0.0 && r.pts[0].1 < 12.0));
    }

    #[test]
    fn border_and_interior() {
        let mut s = rect(0.0, 0.0, 10.0, 10.0);
        s.border_w = 2.0; s.stroke_lpm = 10.0; s.lines_per_mm = 10.0;
        let st = shape_strokes(&s);
        let rings = st.iter().filter(|r| r.closed).count();
        assert_eq!(rings, 20);
        // Abut: interior raster spans 2..8 mm.
        let raster: Vec<&Stroke> = st.iter().filter(|r| !r.closed).collect();
        assert!(raster.iter().all(|r| r.pts.iter().all(|p| p.0 > 1.999 && p.0 < 8.001)));
        // Gap 0.5 mm: interior 2.5..7.5.
        s.relation = Relation::Gap; s.relation_mm = 0.5;
        let st = shape_strokes(&s);
        assert!(st.iter().filter(|r| !r.closed).all(|r| r.pts.iter().all(|p| p.0 > 2.499 && p.0 < 7.501)));
    }

    #[test]
    fn rectangle_fill_matches_v020_positions() {
        let mut s = rect(15.0, 20.0, 75.0, 10.0);
        s.lines_per_mm = 10.0;
        let st = shape_strokes(&s);
        assert_eq!(st.len(), 101);
        assert_eq!(st[0].pts, vec![(15.0, 20.0), (90.0, 20.0)]);
        assert_eq!(st[1].pts, vec![(90.0, 20.1), (15.0, 20.1)]);
        assert!((st[100].pts[0].1 - 30.0).abs() < 1e-4);
    }

    #[test]
    fn passes_have_no_tiny_moves() {
        // Round-cornered polyline trace: offset arcs near the centerline have very short chords.
        let mut s = Shape::polyline(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 12.0)], false);
        s.width = 1.0; s.join = Join::Round;
        let st = shape_strokes(&s);
        assert!(!st.is_empty());
        for p in &st {
            let n = p.pts.len();
            for (k, w) in p.pts.windows(2).enumerate() {
                let d = ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
                // Interior moves at least MIN_STEP; only the last move may be shorter.
                assert!(d >= 0.0499 || k == n - 2, "move {d} mm");
            }
        }
        // A straight run of collinear points becomes one move.
        assert_eq!(clean(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]), vec![(0.0, 0.0), (3.0, 0.0)]);
    }

    #[test]
    fn overscan_leads() {
        let mut s = rect(0.0, 0.0, 10.0, 1.0);
        s.lines_per_mm = 1.0; s.overscan = 2.0;
        let st = shape_strokes(&s);
        assert_eq!(st[0].lead_in, Some((-2.0, 0.0)));
        assert_eq!(st[0].lead_out, Some((12.0, 0.0)));
    }
}
// =============================================================================
// End of file: paths.rs
// Location:    laser-cnc-controller/src/paths.rs
// Created:     2026-10-17 22:04:20 UTC
// =============================================================================
