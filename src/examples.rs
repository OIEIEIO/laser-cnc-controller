// =============================================================================
// File:     examples.rs
// Location: laser-cnc-controller/src/examples.rs
// Version:  v0.1.0
// Comments: Built-in example projects and calibration pattern generators
//           (app v0.2.1).
//           - builtin(): read-only projects written to redb at every start
//             (fixed ids, so run history stays linked; a database reset gets
//             them back). Opening one is read-only; edits offer Duplicate.
//               Example - Feature demo        every new canvas feature, full size
//               Example - LIG tape demo 50x100 the same features inside a
//                 50 x 100 mm tape with 5 mm clearance (40 x 90 usable)
//               Calibration - Power x feed / Line density / Fill direction /
//                 Overscan (tape)  default calibration patterns
//           - calibration(): parametric generator behind File > New
//             calibration pattern…; produces an editable, unsaved project.
//           - Recipe in the examples: S150 (15 % of S1000), F1200, 10 lines/mm.
//             Edit after Duplicate.
//           - Notes layer: labels with leaders attached to their items.
//             Guides layer: tape edge and clearance outlines.
//           - Tape layout: tape lower-left at canvas (20, 20) = job origin,
//             work fence ON 50 x 100 mm (no burn or jog outside the tape).
// =============================================================================
use crate::geometry::{Align, Cap, Fill, Join, Layer, Operation, Project, Pt, Relation, Shape, StrokeDir, TextFont};
use crate::paths;

const POWER: u32 = 150;
const FEED: f32 = 1200.0;
/// Tape lower-left on the canvas.
const TX: f32 = 20.0;
const TY: f32 = 20.0;
const TAPE_W: f32 = 50.0;
const TAPE_H: f32 = 100.0;
const CLEAR: f32 = 5.0;

fn recipe(mut s: Shape) -> Shape { s.power = POWER; s.feed = FEED; s.lines_per_mm = 10.0; s.stroke_lpm = 10.0; s }
fn t(x: f32, y: f32) -> Pt { (TX + x, TY + y) }
fn line(a: Pt, b: Pt, width: f32, dir: StrokeDir, cap: Cap) -> Shape { let mut s = recipe(Shape::line(a, b)); s.width = width; s.stroke_dir = dir; s.cap = cap; s }
fn rect(x: f32, y: f32, w: f32, h: f32) -> Shape { recipe(Shape::rectangle(x, y, w, h)) }
fn bordered(x: f32, y: f32, size: f32, border: f32, rel: Relation, rel_mm: f32) -> Shape {
    let mut s = rect(x, y, size, size); s.border_w = border; s.border_align = Align::Inside; s.relation = rel; s.relation_mm = rel_mm; s
}
fn circle(c: Pt, r: f32, fill: Fill) -> Shape { let mut s = recipe(Shape::circle(c, r)); s.fill = fill; s }
fn guide(x: f32, y: f32, w: f32, h: f32) -> Shape { let mut s = Shape::rectangle(x, y, w, h); s.layer = Layer::Guides; s.operation = Operation::Outline; s }
/// Notes-layer label; with a leader to `to`, attached to `target` when given.
fn note(at: Pt, text: &str, h: f32, to: Option<Pt>, target: Option<&Shape>) -> Shape {
    let mut s = Shape::text(at, text, h, Layer::Notes);
    s.leader = to; s.attach = target.map(|t| t.uid.clone()); s
}
/// Merge rectangles into one region shape with the first rectangle's settings.
fn merged(parts: &[Shape]) -> Shape {
    let refs: Vec<&Shape> = parts.iter().collect();
    let polys = paths::merge(&refs).expect("example parts touch");
    let mut m = parts[0].clone();
    m.kind = crate::geometry::ShapeKind::Region; m.polys = polys; m.uid = crate::geometry::new_id(); m.sync_box(); m
}
fn base(id: &str, name: &str, origin: Pt) -> Project {
    let mut p = Project::blank();
    p.id = id.into(); p.name = name.into(); p.origin = origin; p.read_only = true; p
}
fn tape_guides(p: &mut Project) {
    p.shapes.push(guide(TX, TY, TAPE_W, TAPE_H));
    p.shapes.push(guide(TX + CLEAR, TY + CLEAR, TAPE_W - 2.0 * CLEAR, TAPE_H - 2.0 * CLEAR));
    p.shapes.push(note(t(-17.0, TAPE_H - 3.0), "tape edge", 2.0, Some(t(0.0, TAPE_H - 2.0)), None));
    p.shapes.push(note(t(-17.0, TAPE_H - 8.0), "5 mm clear", 2.0, Some(t(CLEAR, TAPE_H - 7.0)), None));
    p.origin = (TX, TY);
    p.fence.on = true; p.fence.w = TAPE_W; p.fence.h = TAPE_H;
}

/// LIG tape demo: the feature set inside a 50 x 100 mm tape, 5 mm clearance.
pub fn tape_demo() -> Project {
    let mut p = base("builtin-lig-tape-demo", "Example - LIG tape demo 50x100", (TX, TY));
    tape_guides(&mut p);
    let nx = TX + TAPE_W + 6.0; // notes column, right of the tape
    let h = 2.2;
    // 1 vertical traces 2 x 15 mm: w 0, along, across, round
    let v0 = line(t(10.0, 80.0), t(10.0, 95.0), 0.0, StrokeDir::Along, Cap::Square);
    let v1 = line(t(18.0, 80.0), t(18.0, 95.0), 2.0, StrokeDir::Along, Cap::Square);
    let v2 = line(t(26.0, 80.0), t(26.0, 95.0), 2.0, StrokeDir::Across, Cap::Square);
    let v3 = line(t(34.0, 81.0), t(34.0, 94.0), 2.0, StrokeDir::Along, Cap::Round);
    let n1 = note((nx, TY + 88.0), "1 traces 2 × 15 mm: w 0, along, across, round", h, Some(t(35.0, 88.0)), Some(&v3));
    // 2 border 2 mm + interior: abut, overlap 0.5, gap 0.5
    let b0 = bordered(TX + 7.0, TY + 64.0, 10.0, 2.0, Relation::Abut, 0.5);
    let b1 = bordered(TX + 20.0, TY + 64.0, 10.0, 2.0, Relation::Overlap, 0.5);
    let b2 = bordered(TX + 33.0, TY + 64.0, 10.0, 2.0, Relation::Gap, 0.5);
    let n2 = note((nx, TY + 69.0), "2 border 2 mm + interior: abut, overlap 0.5, gap 0.5", h, Some(t(43.0, 69.0)), Some(&b2));
    // 3 merged comb 30 x 14: bar 30 x 4 + five 2 mm fingers
    let mut parts = vec![rect(TX + 10.0, TY + 44.0, 30.0, 4.0)];
    for x in [10.0, 17.0, 24.0, 31.0, 38.0] { parts.push(rect(TX + x, TY + 48.0, 2.0, 10.0)); }
    let comb = merged(&parts);
    let n3 = note((nx, TY + 51.0), "3 merged comb 30 × 14 (6 rectangles): one outline, one fill", h, Some(t(40.0, 46.0)), Some(&comb));
    // 4 circles ø10: concentric, raster
    let c0 = circle(t(15.0, 34.0), 5.0, Fill::Concentric);
    let c1 = circle(t(35.0, 34.0), 5.0, Fill::Horizontal);
    let n4 = note((nx, TY + 34.0), "4 circles ø10: concentric, raster", h, Some(t(40.0, 34.0)), Some(&c1));
    // 5 overscan strips 30 x 3: off (fused ends), 2 mm
    let o0 = rect(TX + 10.0, TY + 21.0, 30.0, 3.0);
    let mut o1 = rect(TX + 10.0, TY + 15.0, 30.0, 3.0); o1.overscan = 2.0;
    let n5 = note((nx, TY + 19.0), "5 overscan 30 × 3: off (fused ends), 2 mm", h, Some(t(40.0, 22.5)), Some(&o0));
    // 6 horizontal traces 30 mm: w 0, 2 mm along, 2 mm across
    let h0 = line(t(10.0, 12.5), t(40.0, 12.5), 0.0, StrokeDir::Along, Cap::Square);
    let h1 = line(t(10.0, 9.5), t(40.0, 9.5), 2.0, StrokeDir::Along, Cap::Square);
    let h2 = line(t(10.0, 6.5), t(40.0, 6.5), 2.0, StrokeDir::Across, Cap::Square);
    let n6 = note((nx, TY + 7.0), "6 traces 30 mm: w 0, 2 mm along, 2 mm across", h, Some(t(40.0, 9.5)), Some(&h1));
    p.shapes.extend([v0, v1, v2, v3, b0, b1, b2, comb, c0, c1, o0, o1, h0, h1, h2]);
    p.shapes.extend([n1, n2, n3, n4, n5, n6]);
    p.shapes.push(note(t(0.0, TAPE_H + 3.0), "LIG tape demo · S150 F1200 · 10 lines/mm · job 0,0 = tape corner · fence = tape", 2.4, None, None));
    p
}

/// Full-size demo of every canvas feature.
pub fn feature_demo() -> Project {
    let mut p = base("builtin-feature-demo", "Example - Feature demo", (10.0, 10.0));
    let add_note = |p: &mut Project, at: Pt, text: &str, to: Option<Pt>, target: Option<&Shape>| p.shapes.push(note(at, text, 3.0, to, target));
    // 1 merged comb (bar + five fingers, merged: one outline, one fill)
    let mut parts = vec![rect(20.0, 180.0, 47.0, 5.0)];
    for x in [20.0, 31.0, 42.0, 53.0, 64.0] { parts.push(rect(x, 185.0, 3.0, 15.0)); }
    let comb = merged(&parts);
    // 2 horizontal traces 50 mm: w 0, 3 mm along, 3 mm across
    let h0 = line((100.0, 236.0), (150.0, 236.0), 0.0, StrokeDir::Along, Cap::Square);
    let h1 = line((100.0, 229.0), (150.0, 229.0), 3.0, StrokeDir::Along, Cap::Square);
    let h2 = line((100.0, 221.0), (150.0, 221.0), 3.0, StrokeDir::Across, Cap::Square);
    // 3 polyline traces 2.5 mm along: miter corners, round corners
    let zig = |x: f32| vec![(x, 185.0), (x + 15.0, 215.0), (x + 30.0, 185.0), (x + 45.0, 215.0)];
    let mut pm = recipe(Shape::polyline(zig(180.0), false)); pm.width = 2.5;
    let mut pr = recipe(Shape::polyline(zig(245.0), false)); pr.width = 2.5; pr.join = Join::Round; pr.cap = Cap::Round;
    // 4 border 3 mm + interior: abut, overlap 1, gap 1
    let b0 = bordered(20.0, 110.0, 20.0, 3.0, Relation::Abut, 1.0);
    let b1 = bordered(50.0, 110.0, 20.0, 3.0, Relation::Overlap, 1.0);
    let b2 = bordered(80.0, 110.0, 20.0, 3.0, Relation::Gap, 1.0);
    // 5 circles ø24: concentric, raster
    let c0 = circle((140.0, 120.0), 12.0, Fill::Concentric);
    let c1 = circle((175.0, 120.0), 12.0, Fill::Horizontal);
    // 6 vertical traces 20 mm: w 0, 3 mm along, 3 mm across, round caps
    let v0 = line((215.0, 110.0), (215.0, 130.0), 0.0, StrokeDir::Along, Cap::Square);
    let v1 = line((225.0, 110.0), (225.0, 130.0), 3.0, StrokeDir::Along, Cap::Square);
    let v2 = line((235.0, 110.0), (235.0, 130.0), 3.0, StrokeDir::Across, Cap::Square);
    let v3 = line((245.0, 111.5), (245.0, 128.5), 3.0, StrokeDir::Along, Cap::Round);
    // 7 closed polyline: 3 mm border across (rails), crosshatch interior
    let mut pent = recipe(Shape::polyline(vec![(275.0, 105.0), (315.0, 105.0), (325.0, 135.0), (295.0, 155.0), (265.0, 135.0)], true));
    pent.border_w = 3.0; pent.stroke_dir = StrokeDir::Across; pent.fill = Fill::Crosshatch; pent.lines_per_mm = 2.0;
    // 8 overscan strips 60 x 4: off, 3 mm
    let o0 = rect(20.0, 50.0, 60.0, 4.0);
    let mut o1 = rect(20.0, 40.0, 60.0, 4.0); o1.overscan = 3.0;
    // 9 text: single-stroke burn ID, outline font with fill
    let mut id = recipe(Shape::text((110.0, 55.0), "S01 P150 F1200", 4.0, Layer::Burn)); id.operation = Operation::Outline;
    let mut big = recipe(Shape::text((110.0, 30.0), "LIG", 12.0, Layer::Burn)); big.font = TextFont::Outline;
    // 10 guides
    let g = guide(300.0, 20.0, 50.0, 70.0);
    add_note(&mut p, (20.0, 205.0), "1 merged comb: 6 rectangles, one outline, one fill", None, None);
    add_note(&mut p, (100.0, 244.0), "2 traces 50 mm: w 0, 3 mm along, 3 mm across", Some((150.0, 236.0)), Some(&h0));
    add_note(&mut p, (180.0, 222.0), "3 polyline traces 2.5 mm: miter, round", Some((225.0, 215.0)), Some(&pm));
    add_note(&mut p, (20.0, 100.0), "4 border 3 mm + interior: abut, overlap 1, gap 1", Some((90.0, 110.0)), Some(&b2));
    add_note(&mut p, (130.0, 140.0), "5 circles ø24: concentric, raster", Some((175.0, 132.0)), Some(&c1));
    add_note(&mut p, (205.0, 100.0), "6 traces 20 mm: w 0, along, across, round", Some((245.0, 110.0)), Some(&v3));
    add_note(&mut p, (262.0, 160.0), "7 closed polyline: border across, crosshatch", Some((295.0, 155.0)), Some(&pent));
    add_note(&mut p, (20.0, 60.0), "8 overscan 60 × 4: off (fused ends), 3 mm", Some((80.0, 52.0)), Some(&o0));
    add_note(&mut p, (170.0, 64.0), "9 text: single-stroke ID, outline font", Some((135.0, 42.0)), Some(&big));
    add_note(&mut p, (300.0, 12.0), "10 guides layer: never burned", Some((325.0, 20.0)), Some(&g));
    add_note(&mut p, (20.0, 262.0), "Feature demo (read-only) · select an item to see its properties · Duplicate to edit · labels are on the Notes layer", None, None);
    p.shapes.extend([comb, h0, h1, h2, pm, pr, b0, b1, b2, c0, c1, v0, v1, v2, v3, pent, o0, o1, id, big, g]);
    p
}

// ---------------------------------------------------------------- calibration
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CalKind { PowerFeed, Density, Direction, Overscan }
impl CalKind { pub fn name(self) -> &'static str { match self { CalKind::PowerFeed => "Power × feed matrix", CalKind::Density => "Line density", CalKind::Direction => "Fill direction", CalKind::Overscan => "Overscan" } } }

#[derive(Clone, Debug, PartialEq)]
pub struct CalParams {
    pub kind: CalKind,
    pub power_from: u32, pub power_to: u32, pub power_steps: u32,
    pub feed_from: f32, pub feed_to: f32, pub feed_steps: u32,
    pub lpm_from: f32, pub lpm_to: f32, pub lpm_steps: u32,
    pub overscan_max: f32, pub overscan_steps: u32,
    pub patch_w: f32, pub patch_h: f32, pub gap: f32,
    /// Fixed recipe for the values not being stepped.
    pub power: u32, pub feed: f32, pub lpm: f32,
    /// Place on the 50 x 100 mm tape (guides, origin at the tape corner, fence = tape).
    pub on_tape: bool,
}
impl Default for CalParams {
    fn default() -> Self {
        Self { kind: CalKind::PowerFeed, power_from: 50, power_to: 250, power_steps: 5, feed_from: 600.0, feed_to: 1500.0, feed_steps: 4,
            lpm_from: 5.0, lpm_to: 20.0, lpm_steps: 4, overscan_max: 3.0, overscan_steps: 4, patch_w: 6.0, patch_h: 6.0, gap: 3.0,
            power: POWER, feed: FEED, lpm: 10.0, on_tape: true }
    }
}
fn steps_f(a: f32, b: f32, n: u32) -> Vec<f32> { let n = n.max(1); (0..n).map(|i| if n == 1 { a } else { a + (b - a) * i as f32 / (n - 1) as f32 }).collect() }

/// Calibration pattern as a new, editable (unsaved) project.
pub fn calibration(c: &CalParams) -> Result<Project, String> {
    let mut p = Project::blank();
    let label_h = 2.0;
    // Items: (label, shape) in rows (top to bottom) and columns.
    let mut grid: Vec<Vec<Shape>> = vec![];
    let mut row_labels: Vec<String> = vec![];
    let mut col_labels: Vec<String> = vec![];
    let patch = |pw: u32, f: f32, lpm: f32| { let mut s = Shape::rectangle(0.0, 0.0, c.patch_w, c.patch_h); s.power = pw; s.feed = f; s.lines_per_mm = lpm; s.stroke_lpm = lpm; s };
    match c.kind {
        CalKind::PowerFeed => {
            let powers: Vec<u32> = steps_f(c.power_from as f32, c.power_to as f32, c.power_steps).iter().map(|v| v.round() as u32).collect();
            let feeds = steps_f(c.feed_from, c.feed_to, c.feed_steps);
            for &pw in &powers { row_labels.push(format!("S{pw}")); grid.push(feeds.iter().map(|&f| patch(pw, f, c.lpm)).collect()); }
            col_labels = feeds.iter().map(|f| format!("F{:.0}", f)).collect();
            p.name = format!("Calibration S{}-{} x F{:.0}-{:.0}", c.power_from, c.power_to, c.feed_from, c.feed_to);
        }
        CalKind::Density => {
            for l in steps_f(c.lpm_from, c.lpm_to, c.lpm_steps) { row_labels.push(format!("{:.1} l/mm", l)); grid.push(vec![patch(c.power, c.feed, l)]); }
            p.name = format!("Calibration density {:.1}-{:.1} l/mm", c.lpm_from, c.lpm_to);
        }
        CalKind::Direction => {
            for (name, f) in [("horizontal", Fill::Horizontal), ("vertical", Fill::Vertical), ("crosshatch", Fill::Crosshatch), ("concentric", Fill::Concentric)] {
                let mut s = patch(c.power, c.feed, c.lpm); s.fill = f; row_labels.push(name.into()); grid.push(vec![s]);
            }
            for (name, dir) in [("trace along", StrokeDir::Along), ("trace across", StrokeDir::Across)] {
                // A trace as wide as the patch is tall, as long as the patch is wide.
                let mut s = Shape::line((0.0, c.patch_h / 2.0), (c.patch_w, c.patch_h / 2.0));
                s.width = c.patch_h; s.stroke_dir = dir; s.power = c.power; s.feed = c.feed; s.stroke_lpm = c.lpm;
                row_labels.push(name.into()); grid.push(vec![s]);
            }
            p.name = format!("Calibration fill direction {:.1} l/mm", c.lpm);
        }
        CalKind::Overscan => {
            for o in steps_f(0.0, c.overscan_max, c.overscan_steps) { let mut s = patch(c.power, c.feed, c.lpm); s.overscan = o; row_labels.push(format!("overscan {:.1}", o)); grid.push(vec![s]); }
            p.name = format!("Calibration overscan 0-{:.1} mm", c.overscan_max);
        }
    }
    let rows = grid.len();
    let cols = grid.iter().map(|r| r.len()).max().unwrap_or(0);
    if rows == 0 || cols == 0 { return Err("Nothing to generate: set at least one step".into()); }
    let gw = cols as f32 * c.patch_w + (cols - 1) as f32 * c.gap;
    let gh = rows as f32 * c.patch_h + (rows - 1) as f32 * c.gap;
    let (ax, ay) = if c.on_tape {
        let (uw, uh) = (TAPE_W - 2.0 * CLEAR, TAPE_H - 2.0 * CLEAR);
        if gw > uw + 1e-3 || gh > uh + 1e-3 {
            return Err(format!("Pattern {gw:.1} × {gh:.1} mm does not fit the tape's usable area {uw:.0} × {uh:.0} mm. Use fewer steps, smaller patches or a smaller gap."));
        }
        // Top-left of the usable area, centred horizontally.
        (TX + CLEAR + (uw - gw) / 2.0, TY + TAPE_H - CLEAR - gh)
    } else { (20.0, 20.0) };
    if c.on_tape { tape_guides(&mut p); } else { p.origin = (ax, ay); }
    for (r, row) in grid.into_iter().enumerate() {
        let y = ay + gh - (r as f32 + 1.0) * c.patch_h - r as f32 * c.gap;
        for (k, mut s) in row.into_iter().enumerate() {
            let x = ax + k as f32 * (c.patch_w + c.gap);
            s.translate(x, y);
            p.shapes.push(s);
        }
        let lx = if c.on_tape { TX - 2.0 - crate::font::text_width(&row_labels[r], label_h, TextFont::Stroke, false) } else { ax - 2.0 - crate::font::text_width(&row_labels[r], label_h, TextFont::Stroke, false) };
        p.shapes.push(note((lx, y + (c.patch_h - label_h) / 2.0), &row_labels[r], label_h, None, None));
    }
    let top = if c.on_tape { TY + TAPE_H + 2.0 } else { ay + gh + 2.0 };
    for (k, l) in col_labels.iter().enumerate() {
        let x = ax + k as f32 * (c.patch_w + c.gap);
        // Stagger column labels on two lines so narrow columns stay readable.
        p.shapes.push(note((x, top + (k % 2) as f32 * (label_h + 1.0)), l, label_h, None, None));
    }
    let what = match c.kind {
        CalKind::PowerFeed => format!("{} · {:.1} l/mm", c.kind.name(), c.lpm),
        CalKind::Density => format!("{} · S{} F{:.0}", c.kind.name(), c.power, c.feed),
        CalKind::Direction | CalKind::Overscan => format!("{} · S{} F{:.0} {:.1} l/mm", c.kind.name(), c.power, c.feed, c.lpm),
    };
    let ty = if c.on_tape { TY + TAPE_H + 2.0 + 2.0 * (label_h + 1.0) + 1.0 } else { top + 2.0 * (label_h + 1.0) + 1.0 };
    p.shapes.push(note((if c.on_tape { TX } else { ax }, ty), &what, 2.4, None, None));
    Ok(p)
}

/// Read-only built-in projects (written to the database at every start).
pub fn builtin() -> Vec<Project> {
    let mut out = vec![feature_demo(), tape_demo()];
    for (id, name, kind) in [("builtin-cal-power-feed", "Calibration - Power x feed (tape)", CalKind::PowerFeed),
                             ("builtin-cal-density", "Calibration - Line density (tape)", CalKind::Density),
                             ("builtin-cal-direction", "Calibration - Fill direction (tape)", CalKind::Direction),
                             ("builtin-cal-overscan", "Calibration - Overscan (tape)", CalKind::Overscan)] {
        if let Ok(mut p) = calibration(&CalParams { kind, ..CalParams::default() }) { p.id = id.into(); p.name = name.into(); p.read_only = true; out.push(p); }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn tape_demo_burns_inside_the_clearance() {
        let p = tape_demo();
        for s in p.shapes.iter().filter(|s| s.layer == Layer::Burn) {
            for st in paths::shape_strokes(s) {
                for q in st.pts.iter().chain(st.lead_in.iter()).chain(st.lead_out.iter()) {
                    assert!(q.0 >= TX + CLEAR - 1e-3 && q.0 <= TX + TAPE_W - CLEAR + 1e-3 && q.1 >= TY + CLEAR - 1e-3 && q.1 <= TY + TAPE_H - CLEAR + 1e-3,
                        "{:?} at {:?} outside the clearance", s.kind, q);
                }
            }
        }
    }
    #[test]
    fn builtins_generate_without_warnings() {
        for p in builtin() {
            let plan = crate::gcode::generate(&p, 1000, &motion_planner::MotionConfig::default());
            assert!(plan.warnings.is_empty(), "{}: {:?}", p.name, plan.warnings);
            assert!(!plan.moves.is_empty(), "{} burns nothing", p.name);
        }
    }
    #[test]
    fn calibration_refuses_what_does_not_fit() {
        let c = CalParams { power_steps: 20, ..CalParams::default() };
        assert!(calibration(&c).is_err());
    }
}
// =============================================================================
// End of file: examples.rs
// Location:    laser-cnc-controller/src/examples.rs
// Created:     2026-10-17 17:35:38 UTC
// =============================================================================
