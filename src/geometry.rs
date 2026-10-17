// =============================================================================
// File:     geometry.rs
// Location: laser-cnc-controller/src/geometry.rs
// Version:  v0.1.4
// Comments: Serializable millimetre geometry and laser process parameters.
//           v0.1.1: Project gains a permanent `id` used to link run records.
//           Older saved projects without an id load with an empty id and are
//           assigned one on first open (see app.rs).
//           v0.1.2:
//           - Project `origin`: job origin marker in canvas millimetres. The
//             generated program is relative to it (marker = job 0,0 = the
//             spot zeroed on the machine with G92). Default (0,0), so older
//             projects generate the same coordinates as before.
//           - Project `fence`: work fence anchored at the origin marker,
//             0..w by 0..h relative to it, with an on/off switch.
//             Older projects load with the fence off.
//           v0.1.3: default work area 380 x 280 mm (the app overrides it with
//           the saved Machine Profile).
//           v0.1.4 (canvas CAD update, app v0.2.1):
//           - Shape kinds Polyline (open or closed), Region (merged outline,
//             polygons with holes) and Text.
//           - Layers: Burn (generates G-code), Notes and Guides (shown on the
//             canvas only, never burned).
//           - Trace parameters for open lines/polylines: width (0 =
//             centerline), direction along/across, density, end cap, corner.
//           - Border on closed shapes: width, alignment inside/centered/
//             outside, direction, density; interior relation abut/overlap/gap.
//           - Operation::None (no outline pass, no interior fill: border only).
//           - Fill::Concentric. Unidirectional scan option. Overscan (mm).
//           - Text: string, height, font (single-stroke or outline), leader
//             point and attachment to another shape (uid).
//           - Shape uid (attachment target). Project read_only (built-in
//             examples).
//           - Every new field has a serde default: projects saved by v0.2.0
//             load unchanged and generate the same raster fills.
// =============================================================================
use serde::{Deserialize, Serialize};
use crate::font;

/// Point in canvas millimetres.
pub type Pt = (f32, f32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ShapeKind { Rectangle, Line, Circle, Polyline, Region, Text }
/// Closed shapes: single pass on the drawn edge (Outline) and/or interior fill (Fill).
/// None: neither (a border-only shape).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Operation { Outline, Fill, Both, None }
/// Interior fill pattern.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Fill { Horizontal, Vertical, Crosshatch, Concentric }
/// Canvas layer. Only Burn generates G-code.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Layer { #[default] Burn, Notes, Guides }
/// Raster direction of a trace or border: passes along the path, or rails across it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum StrokeDir { #[default] Along, Across }
/// Trace end: Square = flush with the end point; Round = half circle centred on the end point.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Cap { #[default] Square, Round }
/// Corner style of polylines, traces and borders.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Join { #[default] Miter, Round }
/// Border position relative to the drawn edge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Align { #[default] Inside, Centered, Outside }
/// Interior relative to the border's inner edge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Relation { #[default] Abut, Overlap, Gap }
/// Text font: single-stroke (centerlines) or outline (closed glyph shapes with fill).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum TextFont { #[default] Stroke, Outline }

impl Layer { pub const ALL: [Layer; 3] = [Layer::Burn, Layer::Notes, Layer::Guides];
    pub fn index(self) -> usize { match self { Layer::Burn => 0, Layer::Notes => 1, Layer::Guides => 2 } }
    pub fn name(self) -> &'static str { match self { Layer::Burn => "Burn", Layer::Notes => "Notes", Layer::Guides => "Guides" } } }

/// One polygon of a Region: outer ring and holes (rings not closed: last point != first).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Poly { pub outer: Vec<Pt>, #[serde(default)] pub holes: Vec<Vec<Pt>> }

fn d_lpm() -> f32 { 10.0 }
fn d_rel() -> f32 { 0.5 }

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Shape {
    pub kind: ShapeKind,
    /// Rectangle / Circle: lower-left corner and size (circle bounding box).
    /// Line: start point and end - start. Text: anchor (lower-left) and h = text height.
    /// Polyline / Region: bounding box kept for older readers (points / polys hold the geometry).
    pub x: f32, pub y: f32, pub w: f32, pub h: f32,
    pub operation: Operation,
    pub fill: Fill,
    /// Interior fill density, lines/mm.
    pub lines_per_mm: f32,
    pub feed: f32,
    pub power: u32,
    pub passes: u32,
    /// Permanent id (attachment target for notes). Empty in older projects; assigned on open.
    #[serde(default)] pub uid: String,
    #[serde(default)] pub layer: Layer,
    /// Polyline vertices.
    #[serde(default)] pub points: Vec<Pt>,
    /// Polyline: closed loop.
    #[serde(default)] pub closed: bool,
    /// Region: polygons with holes.
    #[serde(default)] pub polys: Vec<Poly>,
    /// Open line / polyline trace width (0 = single centerline pass).
    #[serde(default)] pub width: f32,
    /// Trace and border direction.
    #[serde(default)] pub stroke_dir: StrokeDir,
    /// Trace and border density, lines/mm.
    #[serde(default = "d_lpm")] pub stroke_lpm: f32,
    #[serde(default)] pub cap: Cap,
    #[serde(default)] pub join: Join,
    /// Border width on closed shapes (0 = no border).
    #[serde(default)] pub border_w: f32,
    #[serde(default)] pub border_align: Align,
    #[serde(default)] pub relation: Relation,
    /// Overlap or gap distance, mm.
    #[serde(default = "d_rel")] pub relation_mm: f32,
    /// Laser-off lead-in / lead-out along each open pass, mm (0 = off).
    #[serde(default)] pub overscan: f32,
    /// All raster passes in one direction (travel back between passes).
    #[serde(default)] pub unidirectional: bool,
    #[serde(default)] pub text: String,
    #[serde(default)] pub font: TextFont,
    /// Leader end point (notes).
    #[serde(default)] pub leader: Option<Pt>,
    /// Uid of the shape this note is attached to (moves with it).
    #[serde(default)] pub attach: Option<String>,
}
impl Shape {
    fn base(kind: ShapeKind, x: f32, y: f32, w: f32, h: f32) -> Self {
        Self { kind, x, y, w, h, operation: Operation::Fill, fill: Fill::Horizontal, lines_per_mm: 10.0, feed: 600.0, power: 5, passes: 1,
            uid: new_id(), layer: Layer::Burn, points: vec![], closed: false, polys: vec![], width: 0.0, stroke_dir: StrokeDir::Along,
            stroke_lpm: 10.0, cap: Cap::Square, join: Join::Miter, border_w: 0.0, border_align: Align::Inside, relation: Relation::Abut,
            relation_mm: 0.5, overscan: 0.0, unidirectional: false, text: String::new(), font: TextFont::Stroke, leader: None, attach: None }
    }
    pub fn rectangle(x: f32, y: f32, w: f32, h: f32) -> Self { Self::base(ShapeKind::Rectangle, x, y, w, h) }
    pub fn line(a: Pt, b: Pt) -> Self { let mut s = Self::base(ShapeKind::Line, a.0, a.1, b.0 - a.0, b.1 - a.1); s.operation = Operation::Outline; s }
    pub fn circle(c: Pt, r: f32) -> Self { Self::base(ShapeKind::Circle, c.0 - r, c.1 - r, 2.0 * r, 2.0 * r) }
    pub fn polyline(points: Vec<Pt>, closed: bool) -> Self {
        let mut s = Self::base(ShapeKind::Polyline, 0.0, 0.0, 0.0, 0.0);
        s.points = points; s.closed = closed;
        if !closed { s.operation = Operation::Outline; }
        s.sync_box(); s
    }
    pub fn region(polys: Vec<Poly>) -> Self { let mut s = Self::base(ShapeKind::Region, 0.0, 0.0, 0.0, 0.0); s.polys = polys; s.sync_box(); s }
    pub fn text(anchor: Pt, text: &str, h: f32, layer: Layer) -> Self {
        let mut s = Self::base(ShapeKind::Text, anchor.0, anchor.1, 0.0, h.max(0.1));
        s.text = text.to_string(); s.layer = layer; s
    }
    /// Closed outline (has an interior that can be filled and bordered).
    pub fn is_closed(&self) -> bool {
        match self.kind {
            ShapeKind::Rectangle | ShapeKind::Circle | ShapeKind::Region => true,
            ShapeKind::Polyline => self.closed,
            ShapeKind::Line => false,
            ShapeKind::Text => self.font == TextFont::Outline,
        }
    }
    /// Open path that burns as a trace (line or open polyline).
    pub fn is_trace(&self) -> bool { matches!(self.kind, ShapeKind::Line) || (self.kind == ShapeKind::Polyline && !self.closed) }
    /// Path of an open line / polyline.
    pub fn path(&self) -> Vec<Pt> {
        match self.kind { ShapeKind::Line => vec![(self.x, self.y), (self.x + self.w, self.y + self.h)], _ => self.points.clone() }
    }
    /// Text width in mm for the current font and height.
    pub fn text_width(&self) -> f32 { font::text_width(&self.text, self.h, self.font, self.layer == Layer::Burn) }
    /// Polyline / Region: keep x, y, w, h equal to the bounding box.
    pub fn sync_box(&mut self) {
        let pts: Vec<Pt> = match self.kind {
            ShapeKind::Polyline => self.points.clone(),
            ShapeKind::Region => self.polys.iter().flat_map(|p| p.outer.iter().copied()).collect(),
            _ => return,
        };
        if pts.is_empty() { return; }
        let (mut x0, mut y0, mut x1, mut y1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
        for p in pts { x0 = x0.min(p.0); y0 = y0.min(p.1); x1 = x1.max(p.0); y1 = y1.max(p.1); }
        self.x = x0; self.y = y0; self.w = x1 - x0; self.h = y1 - y0;
    }
    /// Bounding box (x0, y0, x1, y1) of the drawn geometry.
    pub fn extent(&self) -> (f32, f32, f32, f32) {
        match self.kind {
            ShapeKind::Text => (self.x, self.y, self.x + self.text_width().max(self.h * 0.5), self.y + self.h),
            ShapeKind::Polyline | ShapeKind::Region => {
                let pts: Vec<Pt> = if self.kind == ShapeKind::Polyline { self.points.clone() } else { self.polys.iter().flat_map(|p| p.outer.iter().copied()).collect() };
                if pts.is_empty() { return (self.x, self.y, self.x, self.y); }
                let (mut x0, mut y0, mut x1, mut y1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
                for p in pts { x0 = x0.min(p.0); y0 = y0.min(p.1); x1 = x1.max(p.0); y1 = y1.max(p.1); }
                (x0, y0, x1, y1)
            }
            _ => {
                let (x0, x1) = if self.w >= 0.0 { (self.x, self.x + self.w) } else { (self.x + self.w, self.x) };
                let (y0, y1) = if self.h >= 0.0 { (self.y, self.y + self.h) } else { (self.y + self.h, self.y) };
                (x0, y0, x1, y1)
            }
        }
    }
    /// Move the shape (and its leader point) by dx, dy.
    pub fn translate(&mut self, dx: f32, dy: f32) {
        self.x += dx; self.y += dy;
        for p in &mut self.points { p.0 += dx; p.1 += dy; }
        for poly in &mut self.polys {
            for p in &mut poly.outer { p.0 += dx; p.1 += dy; }
            for h in &mut poly.holes { for p in h { p.0 += dx; p.1 += dy; } }
        }
        if let Some(l) = &mut self.leader { l.0 += dx; l.1 += dy; }
    }
}

/// Work fence: 0..w by 0..h relative to the job origin marker.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Fence { pub on: bool, pub w: f32, pub h: f32 }
impl Default for Fence {
    fn default() -> Self { Self { on: false, w: 100.0, h: 100.0 } }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Project {
    #[serde(default)]
    pub id: String,
    pub name: String,
    pub shapes: Vec<Shape>,
    pub work_x: f32,
    pub work_y: f32,
    pub travel_feed: f32,
    #[serde(default)]
    pub origin: (f32, f32),
    #[serde(default)]
    pub fence: Fence,
    /// Built-in example: opens read-only; edits offer Duplicate.
    #[serde(default)]
    pub read_only: bool,
}
impl Default for Project {
    fn default() -> Self { Self {id:String::new(),name:"LIG Strip Test".into(), shapes:vec![Shape::rectangle(15.0,20.0,75.0,10.0)],work_x:380.0,work_y:280.0,travel_feed:2500.0,origin:(0.0,0.0),fence:Fence::default(),read_only:false} }
}
impl Project {
    /// Empty project with a fresh permanent id.
    pub fn blank()->Self { Self {id:new_id(),name:"Untitled".into(),shapes:vec![],..Self::default()} }
    /// Give every shape a uid (projects saved before v0.2.1 have none).
    pub fn ensure_uids(&mut self) { for s in &mut self.shapes { if s.uid.is_empty() { s.uid = new_id(); } } }
}

/// Permanent unique id: time in nanoseconds + process id + a per-process counter, hex.
pub fn new_id() -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
    format!("{:x}-{:x}-{:x}", nanos, std::process::id(), COUNTER.fetch_add(1, Ordering::Relaxed))
}
// =============================================================================
// End of file: geometry.rs
// Location:    laser-cnc-controller/src/geometry.rs
// Created:     2026-10-17 17:35:38 UTC
// =============================================================================
