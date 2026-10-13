// =============================================================================
// File:     geometry.rs
// Location: laser-cnc-controller/src/geometry.rs
// Version:  v0.1.1
// Comments: Serializable millimetre geometry and laser process parameters.
//           v0.1.1: Project gains a permanent `id` used to link run records.
//           Older saved projects without an id load with an empty id and are
//           assigned one on first open (see app.rs).
// =============================================================================
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ShapeKind { Rectangle, Line, Circle }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Operation { Outline, Fill, Both }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Fill { Horizontal, Vertical, Crosshatch }

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Shape {
    pub kind: ShapeKind,
    pub x: f32, pub y: f32, pub w: f32, pub h: f32,
    pub operation: Operation,
    pub fill: Fill,
    pub lines_per_mm: f32,
    pub feed: f32,
    pub power: u32,
    pub passes: u32,
}
impl Shape {
    pub fn rectangle(x: f32, y: f32, w: f32, h: f32) -> Self {
        Self { kind: ShapeKind::Rectangle, x, y, w, h, operation: Operation::Fill,
            fill: Fill::Horizontal, lines_per_mm: 10.0, feed: 600.0, power: 5, passes: 1 }
    }
    pub fn extent(&self) -> (f32, f32, f32, f32) {
        let (x0,x1) = if self.w >= 0.0 {(self.x,self.x+self.w)} else {(self.x+self.w,self.x)};
        let (y0,y1) = if self.h >= 0.0 {(self.y,self.y+self.h)} else {(self.y+self.h,self.y)};
        (x0,y0,x1,y1)
    }
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
}
impl Default for Project {
    fn default() -> Self { Self {id:String::new(),name:"LIG Strip Test".into(), shapes:vec![Shape::rectangle(15.0,20.0,75.0,10.0)],work_x:300.0,work_y:200.0,travel_feed:2500.0} }
}
impl Project {
    /// Empty project with a fresh permanent id.
    pub fn blank()->Self { Self {id:new_id(),name:"Untitled".into(),shapes:vec![],..Self::default()} }
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
// Created:     2026-10-12 22:40:08 UTC
// =============================================================================
