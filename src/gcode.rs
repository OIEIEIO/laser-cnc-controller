// =============================================================================
// File:     gcode.rs
// Location: laser-cnc-controller/src/gcode.rs
// Version:  v0.1.0
// Comments: 2D toolpath creation, GRBL 0.9i G-code generation (M3 S / M5 laser
//           control) and constant-speed time estimate. Logic unchanged.
// =============================================================================
use crate::geometry::{Fill, Operation, Project, Shape, ShapeKind};
#[derive(Clone, Copy, Debug)]
pub struct Move { pub x:f32, pub y:f32, pub feed:f32, pub power:u32, pub laser:bool }
#[derive(Default)]
pub struct Plan { pub moves:Vec<Move>, pub lines:Vec<String>, pub move_lines:Vec<usize>, pub engraving_mm:f32, pub travel_mm:f32, pub estimated_seconds:f32, pub warnings:Vec<String> }

fn add(plan:&mut Plan, x:f32,y:f32,feed:f32,power:u32,laser:bool, cursor:&mut (f32,f32),travel:f32) {
    let distance = ((x-cursor.0).powi(2)+(y-cursor.1).powi(2)).sqrt();
    let f = if laser {feed} else {travel};
    if distance > 0.00001 {
        if laser {plan.engraving_mm += distance;} else {plan.travel_mm += distance;}
        plan.estimated_seconds += distance / f.max(1.0) * 60.0;
    }
    plan.moves.push(Move{x,y,feed:f,power,laser});
    *cursor=(x,y);
}
fn stroke(plan:&mut Plan, from:(f32,f32), to:(f32,f32),s:&Shape,cursor:&mut (f32,f32),travel:f32) {
    add(plan,from.0,from.1,travel,0,false,cursor,travel);
    add(plan,to.0,to.1,s.feed,s.power,true,cursor,travel);
}
fn fill_shape(plan:&mut Plan,s:&Shape,cursor:&mut (f32,f32),travel:f32,vertical:bool) {
    let (x0,y0,x1,y1)=s.extent();
    let step=(1.0/s.lines_per_mm.clamp(0.1,100.0)).max(0.01);
    let dim=if vertical {x1-x0} else {y1-y0};
    let n=((dim/step).floor() as usize).min(20000);
    for i in 0..=n {
        let offset=(i as f32*step).min(dim);
        let (a,b)=if vertical {
            let x=x0+offset;
            if i%2==0 {((x,y0),(x,y1))} else {((x,y1),(x,y0))}
        } else {
            let y=y0+offset;
            if i%2==0 {((x0,y),(x1,y))} else {((x1,y),(x0,y))}
        };
        if s.kind==ShapeKind::Circle {
            // Ellipse intersection with a vertical/horizontal scan line.
            let rx=(x1-x0)/2.0;let ry=(y1-y0)/2.0;
            if rx<=0.0||ry<=0.0 {continue;}
            let cx=(x0+x1)/2.0;let cy=(y0+y1)/2.0;
            let factor=if vertical {1.0-((a.0-cx)/rx).powi(2)} else {1.0-((a.1-cy)/ry).powi(2)};
            if factor<=0.0 {continue;}
            let half= factor.sqrt() * if vertical {ry} else {rx};
            let (p,q)=if vertical {((a.0,cy-half),(a.0,cy+half))} else {((cx-half,a.1),(cx+half,a.1))};
            if i%2==0 {stroke(plan,p,q,s,cursor,travel);} else {stroke(plan,q,p,s,cursor,travel);}
        } else {stroke(plan,a,b,s,cursor,travel);}
    }
}
fn outline(plan:&mut Plan,s:&Shape,cursor:&mut (f32,f32),travel:f32) {
    let (x0,y0,x1,y1)=s.extent();
    match s.kind {
        ShapeKind::Rectangle=>{
            for (a,b) in [((x0,y0),(x1,y0)),((x1,y0),(x1,y1)),((x1,y1),(x0,y1)),((x0,y1),(x0,y0))] {stroke(plan,a,b,s,cursor,travel);}
        }
        ShapeKind::Line=>stroke(plan,(s.x,s.y),(s.x+s.w,s.y+s.h),s,cursor,travel),
        ShapeKind::Circle=>{
            let cx=(x0+x1)/2.0;let cy=(y0+y1)/2.0;let rx=(x1-x0)/2.0;let ry=(y1-y0)/2.0;
            for i in 0..96 {let t=std::f32::consts::TAU*(i as f32)/96.0;let u=std::f32::consts::TAU*((i+1) as f32)/96.0;
                stroke(plan,(cx+rx*t.cos(),cy+ry*t.sin()),(cx+rx*u.cos(),cy+ry*u.sin()),s,cursor,travel);}
        }
    }
}
pub fn generate(p:&Project, max_power:u32)->Plan {
    let mut plan=Plan::default();
    let mut cursor=(0.0,0.0);
    let travel=p.travel_feed.max(1.0);
    for (idx,s) in p.shapes.iter().enumerate() {
        let (x0,y0,x1,y1)=s.extent();
        if x0<0.0||y0<0.0||x1>p.work_x||y1>p.work_y {plan.warnings.push(format!("Shape {} is outside machine work area",idx+1));continue;}
        if s.feed<=0.0||s.lines_per_mm<=0.0||s.passes==0||s.power>max_power {plan.warnings.push(format!("Shape {} has invalid process settings",idx+1));continue;}
        for _ in 0..s.passes.min(100) {
            if s.operation != Operation::Fill {outline(&mut plan,s,&mut cursor,travel);}
            if s.kind != ShapeKind::Line && s.operation != Operation::Outline {
                if s.fill != Fill::Vertical {fill_shape(&mut plan,s,&mut cursor,travel,false);}
                if s.fill != Fill::Horizontal {fill_shape(&mut plan,s,&mut cursor,travel,true);}
            }
        }
    }
    // Explicit M5 before every positioning move. No assumption that G0 extinguishes GRBL 0.9 laser output.
    plan.lines.push("G21".into());plan.lines.push("G90".into());plan.lines.push("M5".into());
    let mut last_laser=false; let mut last_power=0u32;
    for m in &plan.moves {
        if !m.laser {
            if last_laser {plan.lines.push("M5".into()); last_laser=false;}
            plan.lines.push(format!("G0 X{:.3} Y{:.3}",m.x,m.y));
        } else {
            if !last_laser||last_power!=m.power {plan.lines.push(format!("M3 S{}",m.power));last_laser=true;last_power=m.power;}
            plan.lines.push(format!("G1 X{:.3} Y{:.3} F{:.1}",m.x,m.y,m.feed));
        }
        plan.move_lines.push(plan.lines.len().saturating_sub(1));
    }
    plan.lines.push("M5".into());
    plan
}
// =============================================================================
// End of file: gcode.rs
// Location:    laser-cnc-controller/src/gcode.rs
// Created:     2026-10-12 18:37:43 UTC
// =============================================================================
