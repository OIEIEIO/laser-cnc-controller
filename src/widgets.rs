// =============================================================================
// File:     widgets.rs
// Location: laser-cnc-controller/src/widgets.rs
// Version:  v0.1.2
// Comments: Shared UI widgets.
//           - Numeric spin field: typed entry (commit on Enter / focus loss)
//             plus ▲▼ step buttons. No drag-to-change, no sliders.
//           - Painted CAD tool icons and icon buttons (no icon-font
//             dependency, crisp at any scale).
//           v0.1.1: Origin (job 0,0 marker tool) and Fence (work fence
//           on/off) icons.
//           v0.1.2 (app v0.2.1): Polyline, Text, Undo, Redo, Merge, Order
//           (burn order numbers) and FitContent (zoom to shapes) icons.
// =============================================================================
use eframe::egui::{self,Align,Color32,CornerRadius,Pos2,Rect,Sense,Stroke,Vec2};

const FIELD_H:f32=24.0;
const SPIN_W:f32=16.0;

/// Shared implementation. `mult` = true: ▲▼ multiply/divide by `step` instead of add/subtract.
fn spin(ui:&mut egui::Ui,id_salt:&str,value:f64,step:f64,mult:bool,min:f64,max:f64,decimals:usize,suffix:&str,width:f32)->Option<f64>{
  let id=ui.id().with(("spin",id_salt));
  let mut result=None;
  ui.horizontal(|ui|{
    ui.spacing_mut().item_spacing.x=0.0;
    // ---- Text entry ----
    let mut text=ui.data_mut(|d|d.get_temp::<String>(id)).unwrap_or_else(||format!("{:.*}",decimals,value));
    let edit=ui.add_sized(Vec2::new(width,FIELD_H),egui::TextEdit::singleline(&mut text).id(id).horizontal_align(Align::RIGHT)
      .margin(egui::Margin::symmetric(6,4)));
    if edit.has_focus(){ui.data_mut(|d|d.insert_temp(id,text.clone()));}
    if edit.lost_focus(){
      if let Ok(v)=text.trim().trim_end_matches(suffix.trim()).trim().parse::<f64>(){result=Some(v.clamp(min,max));}
      ui.data_mut(|d|d.remove::<String>(id));
    }
    // ---- ▲▼ buttons, stacked ----
    let (rect,_)=ui.allocate_exact_size(Vec2::new(SPIN_W,FIELD_H),Sense::hover());
    let up_rect=Rect::from_min_max(rect.min,Pos2::new(rect.max.x,rect.center().y));
    let dn_rect=Rect::from_min_max(Pos2::new(rect.min.x,rect.center().y),rect.max);
    let up=ui.interact(up_rect,id.with("up"),Sense::click());
    let dn=ui.interact(dn_rect,id.with("dn"),Sense::click());
    let visuals=ui.visuals().clone();
    for (r,resp,pointing_up) in [(up_rect,&up,true),(dn_rect,&dn,false)]{
      let fill=if resp.is_pointer_button_down_on(){visuals.widgets.active.weak_bg_fill}else if resp.hovered(){visuals.widgets.hovered.weak_bg_fill}else{visuals.widgets.inactive.weak_bg_fill};
      let cr=if pointing_up{CornerRadius{nw:0,ne:4,sw:0,se:0}}else{CornerRadius{nw:0,ne:0,sw:0,se:4}};
      ui.painter().rect_filled(r.shrink2(Vec2::new(0.0,0.5)),cr,fill);
      let c=r.center();let s=3.5;
      let pts=if pointing_up{vec![Pos2::new(c.x-s,c.y+s*0.55),Pos2::new(c.x+s,c.y+s*0.55),Pos2::new(c.x,c.y-s*0.65)]}
              else{vec![Pos2::new(c.x-s,c.y-s*0.55),Pos2::new(c.x+s,c.y-s*0.55),Pos2::new(c.x,c.y+s*0.65)]};
      ui.painter().add(egui::Shape::convex_polygon(pts,visuals.text_color(),Stroke::NONE));
    }
    let base=result.unwrap_or(value);
    if up.clicked(){result=Some((if mult{base*step}else{base+step}).clamp(min,max));}
    if dn.clicked(){result=Some((if mult{base/step}else{base-step}).clamp(min,max));}
    if !suffix.is_empty(){ui.add_space(5.0);ui.weak(suffix);}
  });
  // Snap away float noise from repeated stepping.
  result.map(|v|{let p=10f64.powi(decimals as i32);(v*p).round()/p})
}

/// f32 spin field. Returns true when the value changed.
pub fn num_f32(ui:&mut egui::Ui,id_salt:&str,value:&mut f32,step:f32,min:f32,max:f32,decimals:usize,suffix:&str)->bool{
  match spin(ui,id_salt,*value as f64,step as f64,false,min as f64,max as f64,decimals,suffix,72.0){
    Some(v) if (v as f32-*value).abs()>f32::EPSILON=>{*value=v as f32;true}
    _=>false
  }
}

/// f32 spin field whose ▲▼ multiply/divide by `factor` (used for zoom).
pub fn num_f32_mul(ui:&mut egui::Ui,id_salt:&str,value:&mut f32,factor:f32,min:f32,max:f32,decimals:usize,suffix:&str)->bool{
  match spin(ui,id_salt,*value as f64,factor as f64,true,min as f64,max as f64,decimals,suffix,60.0){
    Some(v) if (v as f32-*value).abs()>f32::EPSILON=>{*value=v as f32;true}
    _=>false
  }
}

/// u32 spin field. Returns true when the value changed.
pub fn num_u32(ui:&mut egui::Ui,id_salt:&str,value:&mut u32,step:u32,min:u32,max:u32,suffix:&str)->bool{
  match spin(ui,id_salt,*value as f64,step as f64,false,min as f64,max as f64,0,suffix,64.0){
    Some(v)=>{let v=v.round() as u32;if v!=*value{*value=v;true}else{false}}
    None=>false
  }
}

#[derive(Clone,Copy,PartialEq)]
pub enum Icon{Select,Pan,Move,Origin,Rectangle,Line,Circle,Duplicate,Delete,Clear,Grid,Snap,Fit,Fence,Polyline,Text,Undo,Redo,Merge,Order,FitContent}

/// Square icon button. `selected` draws it in the accent (active tool / toggle on).
pub fn icon_button(ui:&mut egui::Ui,icon:Icon,selected:bool,enabled:bool,tip:&str)->egui::Response{
  let size=Vec2::splat(30.0);
  let sense=if enabled{Sense::click()}else{Sense::hover()};
  let (rect,resp)=ui.allocate_exact_size(size,sense);
  let v=ui.visuals();
  let fill=if selected{v.selection.bg_fill}else if enabled&&resp.is_pointer_button_down_on(){v.widgets.active.weak_bg_fill}else if enabled&&resp.hovered(){v.widgets.hovered.weak_bg_fill}else{v.widgets.inactive.weak_bg_fill};
  let fg=if !enabled{Color32::from_gray(90)}else if selected{Color32::WHITE}else{Color32::from_gray(215)};
  let p=ui.painter();
  p.rect_filled(rect,CornerRadius::same(5),fill);
  if selected{p.rect_stroke(rect,CornerRadius::same(5),Stroke::new(1.0_f32,v.selection.stroke.color),egui::StrokeKind::Inside);}
  paint_icon(p,rect.shrink(7.0),icon,fg);
  resp.on_hover_text(tip)
}

fn paint_icon(p:&egui::Painter,r:Rect,icon:Icon,c:Color32){
  let s=Stroke::new(1.6_f32,c);
  let at=|x:f32,y:f32|Pos2::new(r.left()+x*r.width(),r.top()+y*r.height());
  let arrow_head=|tip:Pos2,dir:Vec2|{let d=dir.normalized();let n=Vec2::new(-d.y,d.x);let b=tip-d*4.0;
    p.add(egui::Shape::convex_polygon(vec![tip,b+n*3.0,b-n*3.0],c,Stroke::NONE));};
  match icon{
    Icon::Select=>{
      let pts=vec![at(0.22,0.05),at(0.22,0.86),at(0.42,0.67),at(0.56,0.97),at(0.70,0.90),at(0.56,0.61),at(0.82,0.61)];
      p.add(egui::Shape::closed_line(pts,s));
    }
    Icon::Pan=>{
      let c0=r.center();let h=r.width()*0.5;
      for d in [Vec2::X,-Vec2::X,Vec2::Y,-Vec2::Y]{p.line_segment([c0,c0+d*(h-1.0)],s);arrow_head(c0+d*h,d);}
    }
    Icon::Move=>{
      let a=Rect::from_min_max(at(0.0,0.55),at(0.42,0.97));
      p.extend(egui::Shape::dashed_line(&[a.left_top(),a.right_top(),a.right_bottom(),a.left_bottom(),a.left_top()],Stroke::new(1.2_f32,c),2.5,2.0));
      p.rect_stroke(Rect::from_min_max(at(0.58,0.03),at(1.0,0.45)),0.0,s,egui::StrokeKind::Inside);
      p.line_segment([at(0.30,0.67),at(0.62,0.38)],s);arrow_head(at(0.68,0.32),Vec2::new(1.0,-0.9));
    }
    Icon::Origin=>{
      let c0=r.center();
      p.line_segment([at(0.0,0.5),at(1.0,0.5)],s);p.line_segment([at(0.5,0.0),at(0.5,1.0)],s);
      p.circle_stroke(c0,r.width()*0.30,s);
    }
    Icon::Fence=>{
      let a=Rect::from_min_max(at(0.0,0.08),at(1.0,0.92));
      p.extend(egui::Shape::dashed_line(&[a.left_top(),a.right_top(),a.right_bottom(),a.left_bottom(),a.left_top()],Stroke::new(1.4_f32,c),3.0,2.0));
      p.circle_filled(a.left_bottom(),2.6,c);p.rect_filled(Rect::from_center_size(a.right_top(),Vec2::splat(4.5)),0.0,c);
    }
    Icon::Rectangle=>{p.rect_stroke(Rect::from_min_max(at(0.0,0.18),at(1.0,0.82)),0.0,s,egui::StrokeKind::Inside);}
    Icon::Line=>{p.line_segment([at(0.08,0.92),at(0.92,0.08)],s);p.circle_filled(at(0.08,0.92),2.4,c);p.circle_filled(at(0.92,0.08),2.4,c);}
    Icon::Circle=>{p.circle_stroke(r.center(),r.width()*0.48,s);p.circle_filled(r.center(),1.6,c);}
    Icon::Duplicate=>{
      p.rect_stroke(Rect::from_min_max(at(0.0,0.30),at(0.70,1.0)),0.0,s,egui::StrokeKind::Inside);
      p.rect_stroke(Rect::from_min_max(at(0.30,0.0),at(1.0,0.70)),0.0,s,egui::StrokeKind::Inside);
    }
    Icon::Delete=>{
      p.line_segment([at(0.05,0.18),at(0.95,0.18)],s);p.line_segment([at(0.36,0.18),at(0.40,0.04)],s);
      p.line_segment([at(0.40,0.04),at(0.60,0.04)],s);p.line_segment([at(0.60,0.04),at(0.64,0.18)],s);
      p.add(egui::Shape::line(vec![at(0.15,0.18),at(0.22,0.98),at(0.78,0.98),at(0.85,0.18)],s));
      p.line_segment([at(0.40,0.38),at(0.42,0.80)],Stroke::new(1.2_f32,c));p.line_segment([at(0.60,0.38),at(0.58,0.80)],Stroke::new(1.2_f32,c));
    }
    Icon::Clear=>{
      let a=r.shrink(1.0);
      p.extend(egui::Shape::dashed_line(&[a.left_top(),a.right_top(),a.right_bottom(),a.left_bottom(),a.left_top()],Stroke::new(1.2_f32,c),2.5,2.0));
      p.line_segment([at(0.25,0.25),at(0.75,0.75)],s);p.line_segment([at(0.75,0.25),at(0.25,0.75)],s);
    }
    Icon::Grid=>{
      for i in 0..4{let t=i as f32/3.0;p.line_segment([at(t,0.0),at(t,1.0)],Stroke::new(1.1_f32,c));p.line_segment([at(0.0,t),at(1.0,t)],Stroke::new(1.1_f32,c));}
    }
    Icon::Snap=>{
      for i in 0..3{let t=i as f32/2.0;let g=Stroke::new(1.0_f32,c.gamma_multiply(0.55));p.line_segment([at(t,0.0),at(t,1.0)],g);p.line_segment([at(0.0,t),at(1.0,t)],g);}
      p.circle_stroke(at(0.5,0.5),4.0,s);p.circle_filled(at(0.5,0.5),1.8,c);
    }
    Icon::Fit=>{
      let k=0.32;
      for (x,y,dx,dy) in [(0.0,0.0,1.0,1.0),(1.0,0.0,-1.0,1.0),(0.0,1.0,1.0,-1.0),(1.0,1.0,-1.0,-1.0)]{
        p.line_segment([at(x,y),at(x+dx*k,y)],s);p.line_segment([at(x,y),at(x,y+dy*k)],s);
      }
      p.rect_stroke(Rect::from_min_max(at(0.32,0.32),at(0.68,0.68)),0.0,Stroke::new(1.2_f32,c),egui::StrokeKind::Inside);
    }
    Icon::Polyline=>{
      p.add(egui::Shape::line(vec![at(0.05,0.90),at(0.30,0.25),at(0.62,0.70),at(0.95,0.10)],s));
      for (x,y) in [(0.05,0.90),(0.30,0.25),(0.62,0.70),(0.95,0.10)]{p.circle_filled(at(x,y),2.0,c);}
    }
    Icon::Text=>{
      p.line_segment([at(0.10,0.10),at(0.90,0.10)],s);p.line_segment([at(0.50,0.10),at(0.50,0.95)],s);
      p.line_segment([at(0.35,0.95),at(0.65,0.95)],Stroke::new(1.2_f32,c));
    }
    Icon::Undo|Icon::Redo=>{
      let flip=icon==Icon::Redo;
      let fx=|x:f32|if flip{1.0-x}else{x};
      let pts:Vec<Pos2>=(0..=10).map(|i|{let t=std::f32::consts::PI*(i as f32/10.0);at(fx(0.55+0.38*t.sin()),0.55-0.30*t.cos())}).collect();
      p.add(egui::Shape::line(pts,s));
      p.line_segment([at(fx(0.55),0.25),at(fx(0.18),0.25)],s);
      arrow_head(at(fx(0.08),0.25),Vec2::new(if flip{1.0}else{-1.0},0.0));
    }
    Icon::Merge=>{
      p.rect_stroke(Rect::from_min_max(at(0.0,0.55),at(1.0,0.95)),0.0,Stroke::new(1.2_f32,c.gamma_multiply(0.6)),egui::StrokeKind::Inside);
      p.rect_stroke(Rect::from_min_max(at(0.15,0.05),at(0.45,0.60)),0.0,Stroke::new(1.2_f32,c.gamma_multiply(0.6)),egui::StrokeKind::Inside);
      p.add(egui::Shape::closed_line(vec![at(0.0,0.95),at(1.0,0.95),at(1.0,0.55),at(0.45,0.55),at(0.45,0.05),at(0.15,0.05),at(0.15,0.55),at(0.0,0.55)],s));
    }
    Icon::Order=>{
      for (k,y) in [0.12f32,0.5,0.88].iter().enumerate(){
        p.text(at(0.12,*y),egui::Align2::CENTER_CENTER,format!("{}",k+1),egui::FontId::proportional(9.0),c);
        p.line_segment([at(0.35,*y),at(1.0,*y)],Stroke::new(1.4_f32,c));
      }
    }
    Icon::FitContent=>{
      let k=0.32;
      for (x,y,dx,dy) in [(0.0,0.0,1.0,1.0),(1.0,0.0,-1.0,1.0),(0.0,1.0,1.0,-1.0),(1.0,1.0,-1.0,-1.0)]{
        p.line_segment([at(x,y),at(x+dx*k,y)],s);p.line_segment([at(x,y),at(x,y+dy*k)],s);
      }
      p.circle_stroke(at(0.5,0.5),r.width()*0.16,Stroke::new(1.2_f32,c));
      p.rect_stroke(Rect::from_min_max(at(0.30,0.36),at(0.56,0.62)),0.0,Stroke::new(1.0_f32,c),egui::StrokeKind::Inside);
    }
  }
}
// =============================================================================
// End of file: widgets.rs
// Location:    laser-cnc-controller/src/widgets.rs
// Created:     2026-10-17 17:35:38 UTC
// =============================================================================
