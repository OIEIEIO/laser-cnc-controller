// =============================================================================
// File:     gcode.rs
// Location: laser-cnc-controller/src/gcode.rs
// Version:  v0.2.2
// Comments: 2D toolpath creation, GRBL 0.9i G-code generation (M3 S / M5 laser
//           control) and planned time estimate.
//           v0.1.1:
//           - Program coordinates are relative to the job origin marker
//             (Project::origin): G-code X/Y = canvas X/Y - origin. `moves`
//             stay in canvas coordinates for drawing and simulation; the
//             toolpath starts at the origin (Plan::origin).
//           - The per-shape "outside machine work area" check (canvas 0..work)
//             is replaced by two job checks on the relative coordinates of
//             every move (burn and travel), including the 0,0 start point:
//               * span larger than the machine travel (work_x by work_y);
//               * work fence ON: any move outside 0..fence.w by 0..fence.h.
//           - Negative zero is written as 0.000.
//           v0.2.0:
//           - Time estimate from the motion-planner crate: the generated lines
//             are planned from job 0,0 with the machine's acceleration, max
//             rates, junction deviation and look-ahead (MotionConfig), with a
//             full stop at every M3/M5 as GRBL 0.9 does. Replaces the
//             constant-speed lower bound.
//           v0.2.1:
//           - Toolpath from src/paths.rs: each shape becomes strokes (raster
//             lines, trace passes, rails, contour loops), burned in the order
//             of the shape list (burn order). Notes and Guides shapes are
//             skipped.
//           - A stroke burns continuously: one M3, then G1 through all its
//             points. Outline passes on rectangles and circles are now one
//             closed loop (v0.2.0 started each edge / circle segment with its
//             own M5 / G0 / M3).
//           - Overscan: G0 to the lead-in point, laser-off G1 at the burn feed
//             to the pass start, M3, burn, M5, laser-off G1 to the lead-out
//             point. Move::rapid tells G0 travel from laser-off G1 moves.
//           - Fill raster positions unchanged from v0.2.0.
//           v0.2.2: the planned time is split by the laser state of each planned
//           move (acceleration and deceleration included): engrave_seconds
//           (laser on) and travel_seconds (laser off: G0 travel, overscan
//           leads, the stops around M3/M5). They add up to estimated_seconds.
// =============================================================================
use crate::geometry::{Layer, Project};
use crate::paths;
use motion_planner::MotionConfig;
/// One move in canvas coordinates. laser = burn; !laser && rapid = G0 travel; !laser && !rapid = laser-off G1 (overscan).
#[derive(Clone, Copy, Debug)]
pub struct Move { pub x:f32, pub y:f32, pub feed:f32, pub power:u32, pub laser:bool, pub rapid:bool }
#[derive(Default)]
pub struct Plan { pub moves:Vec<Move>, pub lines:Vec<String>, pub move_lines:Vec<usize>, pub engraving_mm:f32, pub travel_mm:f32, pub estimated_seconds:f32,
    /// Planned time with the laser on / off; their sum is estimated_seconds.
    pub engrave_seconds:f32, pub travel_seconds:f32, pub warnings:Vec<String>, pub origin:(f32,f32) }

const EPS:f32=0.001;

fn add(plan:&mut Plan, m:Move, cursor:&mut (f32,f32)) {
    let distance = ((m.x-cursor.0).powi(2)+(m.y-cursor.1).powi(2)).sqrt();
    if distance > 0.00001 {
        if m.laser {plan.engraving_mm += distance;} else {plan.travel_mm += distance;}
    }
    plan.moves.push(m);
    *cursor=(m.x,m.y);
}
/// Coordinate as written to G-code: no negative zero.
pub fn coord(v:f32)->f32{if v.abs()<0.0005{0.0}else{v}}

/// Relative extent (x0,y0,x1,y1) of a set of canvas points, including the 0,0 start.
pub fn relative_span(points:&[(f32,f32)],origin:(f32,f32))->(f32,f32,f32,f32){
    let (mut x0,mut y0,mut x1,mut y1)=(0.0f32,0.0f32,0.0f32,0.0f32);
    for p in points{let (x,y)=(p.0-origin.0,p.1-origin.1);x0=x0.min(x);y0=y0.min(y);x1=x1.max(x);y1=y1.max(y);}
    (x0,y0,x1,y1)
}
/// Machine-travel and work-fence checks on relative coordinates. Empty when the job may run.
pub fn limit_warnings(p:&Project,span:(f32,f32,f32,f32),what:&str)->Vec<String>{
    let (x0,y0,x1,y1)=span;let mut out=vec![];
    if x1-x0>p.work_x+EPS||y1-y0>p.work_y+EPS{
        out.push(format!("{what} spans {:.1} × {:.1} mm from the job origin, larger than the machine travel {} × {} mm",x1-x0,y1-y0,p.work_x,p.work_y));
    }
    if p.fence.on&&(x0< -EPS||y0< -EPS||x1>p.fence.w+EPS||y1>p.fence.h+EPS){
        out.push(format!("{what} leaves the work fence (0–{:.1} × 0–{:.1} mm): X {:.2} … {:.2}, Y {:.2} … {:.2}",p.fence.w,p.fence.h,x0,x1,y0,y1));
    }
    out
}
pub fn generate(p:&Project, max_power:u32, motion:&MotionConfig)->Plan {
    let mut plan=Plan{origin:p.origin,..Plan::default()};
    let mut cursor=p.origin;
    let travel=p.travel_feed.max(1.0);
    for (idx,s) in p.shapes.iter().enumerate() {
        if s.layer!=Layer::Burn {continue;}
        if s.feed<=0.0||s.lines_per_mm<=0.0||s.stroke_lpm<=0.0||s.passes==0||s.power>max_power {plan.warnings.push(format!("Shape {} has invalid process settings",idx+1));continue;}
        let strokes=paths::shape_strokes(s);
        for _ in 0..s.passes.min(100) {
            for st in &strokes {
                let Some(&first)=st.pts.first() else {continue;};
                match st.lead_in {
                    Some(l)=>{
                        add(&mut plan,Move{x:l.0,y:l.1,feed:travel,power:0,laser:false,rapid:true},&mut cursor);
                        add(&mut plan,Move{x:first.0,y:first.1,feed:s.feed,power:0,laser:false,rapid:false},&mut cursor);
                    }
                    None=>add(&mut plan,Move{x:first.0,y:first.1,feed:travel,power:0,laser:false,rapid:true},&mut cursor),
                }
                for q in &st.pts[1..] {add(&mut plan,Move{x:q.0,y:q.1,feed:s.feed,power:s.power,laser:true,rapid:false},&mut cursor);}
                if let Some(l)=st.lead_out {add(&mut plan,Move{x:l.0,y:l.1,feed:s.feed,power:0,laser:false,rapid:false},&mut cursor);}
            }
        }
    }
    // Job limits on relative coordinates (every burn and travel move, plus the 0,0 start).
    let points:Vec<(f32,f32)>=plan.moves.iter().map(|m|(m.x,m.y)).collect();
    if !points.is_empty(){let w=limit_warnings(p,relative_span(&points,p.origin),"Job");plan.warnings.extend(w);}
    // Explicit M5 before every laser-off move. No assumption that G0 extinguishes GRBL 0.9 laser output.
    let (ox,oy)=p.origin;
    plan.lines.push("G21".into());plan.lines.push("G90".into());plan.lines.push("M5".into());
    let mut last_laser=false; let mut last_power=0u32;
    for m in &plan.moves {
        let (x,y)=(coord(m.x-ox),coord(m.y-oy));
        if !m.laser {
            if last_laser {plan.lines.push("M5".into()); last_laser=false;}
            if m.rapid {plan.lines.push(format!("G0 X{x:.3} Y{y:.3}"));}
            else {plan.lines.push(format!("G1 X{x:.3} Y{y:.3} F{:.1}",m.feed));}
        } else {
            if !last_laser||last_power!=m.power {plan.lines.push(format!("M3 S{}",m.power));last_laser=true;last_power=m.power;}
            plan.lines.push(format!("G1 X{x:.3} Y{y:.3} F{:.1}",m.feed));
        }
        plan.move_lines.push(plan.lines.len().saturating_sub(1));
    }
    plan.lines.push("M5".into());
    // Planned time of the lines as written, from job 0,0 (acceleration, corners, stops at M3/M5).
    let blocks=motion_planner::gcode::blocks_from_lines(&plan.lines,[0.0,0.0]);
    let traj=motion_planner::plan(motion,&blocks);
    for seg in &traj.segments {
        let t=seg.profile.duration();
        if seg.block.laser {plan.engrave_seconds+=t;} else {plan.travel_seconds+=t;}
    }
    plan.estimated_seconds=traj.duration();
    plan
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Shape;
    #[test]
    fn notes_do_not_burn_and_overscan_writes_laser_off_g1() {
        let mut p=Project::blank();
        let mut r=Shape::rectangle(10.0,10.0,10.0,1.0);r.lines_per_mm=1.0;r.overscan=2.0;
        let mut n=Shape::rectangle(0.0,0.0,5.0,5.0);n.layer=Layer::Notes;
        p.shapes=vec![n,r];
        let plan=generate(&p,1000,&MotionConfig::default());
        let l=&plan.lines;
        // Lead-in: G0 to x=8, laser-off G1 to x=10, M3, burn to x=20, M5, laser-off G1 to x=22.
        let i=l.iter().position(|s|s=="G0 X8.000 Y10.000").expect("lead-in G0");
        assert_eq!(l[i+1],"G1 X10.000 Y10.000 F600.0");
        assert_eq!(l[i+2],"M3 S5");
        assert_eq!(l[i+3],"G1 X20.000 Y10.000 F600.0");
        assert_eq!(l[i+4],"M5");
        assert_eq!(l[i+5],"G1 X22.000 Y10.000 F600.0");
        assert!(!l.iter().any(|s|s.contains("X5.000 Y5.000")));
        // Burn and travel time add up to the estimate.
        assert!(plan.engrave_seconds>0.0&&plan.travel_seconds>0.0);
        assert!((plan.engrave_seconds+plan.travel_seconds-plan.estimated_seconds).abs()<1e-3);
    }
}
// =============================================================================
// End of file: gcode.rs
// Location:    laser-cnc-controller/src/gcode.rs
// Created:     2026-10-17 18:43:10 UTC
// =============================================================================
