// =============================================================================
// File:     canvas.rs
// Location: laser-cnc-controller/src/canvas.rs
// Version:  v0.1.7
// Comments: Mini-CAD canvas engine (millimetre world, Y up).
//           Tools: Select, Pan, Move, Rectangle, Line, Circle.
//           Mouse (FreeCAD OpenSCAD-style, adapted to 2D):
//             Left click    select / place point (click-to-start, click-to-finish)
//             Left drag     box select (empty), move (on shape), resize (handle)
//             Right drag    pan (any tool)        Shift+Right drag  zoom
//             Middle drag   zoom                  Wheel             zoom at cursor
//             Right click / Esc  cancel           Delete            delete selection
//           Box select: drag right = window (fully inside), drag left = crossing.
//           Canvas tool strip: tools, duplicate/delete/clear, grid on/off,
//           grid spacing (0.1/0.5/1/5/10 mm), snap on/off, zoom, fit.
//           Status bar: cursor X/Y (mm), tool hint, selection, zoom, grid, snap.
//           v0.1.1: Generate G-code removed (G-code regenerates on every edit).
//           Shape Properties dock slot at the end of the tool strip:
//           "Shape Properties [Float]" while docked (panel hidden),
//           "Shape Properties [Dock]" while floating. The app reads props_toggle.
//           v0.1.2:
//           - Origin tool: click (or drag-release) places the job origin
//             marker (job 0,0). With Select, drag the marker to move it.
//           - Work fence: rectangle 0..W by 0..H from the origin marker.
//             Tool strip: Fence on/off toggle and W / H fields. With Select,
//             drag the fence's top-right handle to resize it. ON = solid
//             orange, OFF = dashed grey.
//           - Toolpath drawing starts at the origin marker (Plan::origin).
//           - SimOverlay carries an optional path (live Job/Trace run) drawn
//             over the plan with done/current/remaining styling.
//           - Status bar shows origin position and fence state.
//           v0.1.3: head marker and the line to it are red while the laser is
//           on, yellow while it is off (SimOverlay::laser).
//           v0.1.4: Shape Properties dock slot removed from the tool strip
//           (the panel now docks in the app's right column).
//           v0.1.5: default grid spacing 5 mm.
//           v0.1.6 (app v0.2.1):
//           - Tools Polyline (click vertices; click the first vertex to close,
//             double-click or Enter to finish open, Backspace removes the
//             last vertex), Text (click to place) and Leader (set from Shape
//             Properties: click the point a note points to; a shape under the
//             click attaches the note to it).
//           - Merge (tool strip): selected closed shapes that share an edge or
//             overlap become one Region outline (src/paths.rs). Refused
//             shapes flash red for 3 s; the reason goes to the app notice.
//           - Layers: new shapes go on the active layer (Burn / Notes /
//             Guides); each layer can be hidden. Notes purple, guides dashed
//             grey. Hidden layers are not drawn or picked (Burn still burns).
//           - Burn order numbers (toggle) on Burn shapes, in list order.
//           - Notes attached to a shape move with it (drag, Move tool).
//             Moving a note itself keeps its leader point.
//           - Selected traces and bordered shapes show their burned extent
//             (dashed).
//           - Laser-off feed moves (overscan) drawn amber.
//           - Tool strip in groups (draw tools | undo/redo, duplicate,
//             delete, clear, merge, order | grid, snap | zoom, fit, fit
//             shapes | layers | fence). Groups wrap as units; docked =
//             two rows, floating = wraps to the window width.
//           - Fit shapes: zoom to all visible shapes and the origin marker.
//             Opening a project fits its shapes.
//           - toolbar() returns Events (changed, undo, redo, notice).
//           v0.1.7 (app v0.2.2): run countdown. During a live Job or Trace run
//           (machine or Simulate) a large timer sits in the canvas's top-right
//           corner: time remaining in large digits, a progress bar, elapsed
//           and total. Planned machine time from the run clock: it runs at the
//           Simulate speed in Simulate and stops while the controller reports
//           Hold or Door (shown as PAUSED, amber). Painted only, not clickable.
// =============================================================================
use eframe::egui::{self,Color32,CursorIcon,Key,PointerButton,Pos2,Rect,Sense,Stroke,Vec2};
use std::time::{Duration,Instant};
use crate::geometry::{self,Layer,Project,Pt,Shape,ShapeKind,TextFont};
use crate::gcode::{Move,Plan};
use crate::{font,paths};
use crate::widgets::{self,Icon};

#[derive(Clone,Copy,PartialEq,Debug)]
pub enum Tool{Select,Pan,Move,Origin,Rectangle,Line,Polyline,Circle,Text,Leader}

pub const GRID_SPACINGS:[f32;5]=[0.1,0.5,1.0,5.0,10.0];
const ZOOM_MIN:f32=0.2;
const ZOOM_MAX:f32=200.0;
const HANDLE_PX:f32=4.5;
const PICK_PX:f32=6.0;
const REFUSED_FOR:Duration=Duration::from_secs(3);

const C_SHAPE:Color32=Color32::from_rgb(175,178,186);
const C_HOVER:Color32=Color32::from_rgb(225,228,235);
const C_SELECT:Color32=Color32::from_rgb(90,170,255);
const C_PREVIEW:Color32=Color32::from_rgb(255,196,64);
const C_ORIGIN:Color32=Color32::from_rgb(80,220,230);
const C_FENCE:Color32=Color32::from_rgb(240,150,40);
const C_NOTE:Color32=Color32::from_rgb(176,150,240);
const C_GUIDE:Color32=Color32::from_rgb(128,132,140);
const C_REFUSED:Color32=Color32::from_rgb(240,80,80);
const C_ORDER:Color32=Color32::from_rgb(235,190,70);

/// Requests from the tool strip and canvas to the app.
#[derive(Default)]
pub struct Events{pub changed:bool,pub undo:bool,pub redo:bool,pub notice:Option<String>}

enum Drag{
  None,
  Box{start:Pos2},
  Move{start:(f32,f32),originals:Vec<(usize,Shape,bool)>},
  Handle{index:usize,handle:usize,original:Shape},
  Pan,
  Zoom{anchor:Pos2},
  Origin{original:(f32,f32)},
  FenceCorner{original:(f32,f32)},
}
#[derive(Clone,Copy,PartialEq)]
enum FitKind{Work,Shapes}

/// Simulation / live-run overlay passed in from the app.
/// `path` None: progress applies to the plan's moves (simulation).
/// `path` Some: a live run's own path (canvas coordinates) starting at `start`.
/// `timer` Some((elapsed, total, paused)): run clock and planned duration in seconds, for the countdown.
pub struct SimOverlay<'a>{pub active:bool,pub index:usize,pub head:(f32,f32),pub start:(f32,f32),pub path:Option<&'a [Move]>,pub laser:bool,pub timer:Option<(f32,f32,bool)>}

/// Seconds as HH:MM:SS (rounded up).
fn hms(seconds:f32)->String{let t=seconds.max(0.0).ceil() as u64;format!("{:02}:{:02}:{:02}",t/3600,(t/60)%60,t%60)}

pub struct CanvasView{
  pub tool:Tool, pub grid:bool, pub grid_spacing:f32, pub snap:bool, pub zoom:f32,
  pub active_layer:Layer, pub hidden:[bool;3], pub show_order:bool,
  pan:Vec2, fit_pending:Option<FitKind>, last_rect:Rect,
  draw_start:Option<(f32,f32)>, poly_pts:Vec<Pt>, move_base:Option<(f32,f32)>, drag:Drag,
  leader_for:Option<usize>, refused:Vec<usize>, refused_at:Option<Instant>,
}
impl Default for CanvasView{
  fn default()->Self{Self{tool:Tool::Select,grid:true,grid_spacing:5.0,snap:true,zoom:2.0,
    active_layer:Layer::Burn,hidden:[false;3],show_order:false,pan:Vec2::new(34.0,-28.0),
    fit_pending:Some(FitKind::Work),last_rect:Rect::NOTHING,draw_start:None,poly_pts:vec![],move_base:None,drag:Drag::None,
    leader_for:None,refused:vec![],refused_at:None}}
}

/// Notes attached (by uid) to any shape in `moved` that are not themselves in `moved`.
pub fn followers(project:&Project,moved:&[usize])->Vec<usize>{
  let uids:Vec<&str>=moved.iter().filter_map(|&i|project.shapes.get(i)).map(|s|s.uid.as_str()).filter(|u|!u.is_empty()).collect();
  project.shapes.iter().enumerate()
    .filter(|(i,s)|!moved.contains(i)&&s.attach.as_deref().is_some_and(|a|uids.contains(&a)))
    .map(|(i,_)|i).collect()
}
/// Remove shapes by index; notes attached to them are detached.
pub fn delete_shapes(project:&mut Project,idx:&[usize]){
  let mut idx=idx.to_vec();idx.sort_unstable();idx.dedup();
  let gone:Vec<String>=idx.iter().filter_map(|&i|project.shapes.get(i)).map(|s|s.uid.clone()).collect();
  for i in idx.into_iter().rev(){if i<project.shapes.len(){project.shapes.remove(i);}}
  for s in &mut project.shapes{if s.attach.as_ref().is_some_and(|a|gone.contains(a)){s.attach=None;}}
}
/// Move a shape. A note moved directly keeps its leader point; followers move whole.
fn translated(original:&Shape,dx:f32,dy:f32,keep_leader:bool)->Shape{
  let mut s=original.clone();s.translate(dx,dy);
  if keep_leader{s.leader=original.leader;}
  s
}

impl CanvasView{
  pub fn request_fit(&mut self){self.fit_pending=Some(FitKind::Work);}
  pub fn request_fit_shapes(&mut self){self.fit_pending=Some(FitKind::Shapes);}
  /// Leader tool for the note at `index` (next canvas click sets the leader point).
  pub fn begin_leader(&mut self,index:usize){self.cancel_soft();self.tool=Tool::Leader;self.leader_for=Some(index);}
  fn layer_visible(&self,l:Layer)->bool{!self.hidden[l.index()]}

  // ---------------------------------------------------------------- transforms
  fn to_screen(&self,rect:Rect,x:f32,y:f32)->Pos2{Pos2::new(rect.left()+self.pan.x+x*self.zoom,rect.bottom()+self.pan.y-y*self.zoom)}
  fn to_world(&self,rect:Rect,p:Pos2)->(f32,f32){((p.x-rect.left()-self.pan.x)/self.zoom,(rect.bottom()+self.pan.y-p.y)/self.zoom)}
  fn snap_pt(&self,p:(f32,f32))->(f32,f32){
    if !self.snap{return p;}
    let s=self.grid_spacing;((p.0/s).round()*s,(p.1/s).round()*s)
  }
  fn zoom_about(&mut self,rect:Rect,p:Pos2,new_zoom:f32){
    let w=self.to_world(rect,p);
    self.zoom=new_zoom.clamp(ZOOM_MIN,ZOOM_MAX);
    self.pan.x=p.x-rect.left()-w.0*self.zoom;
    self.pan.y=p.y-rect.bottom()+w.1*self.zoom;
  }
  fn fit_box(&mut self,rect:Rect,b:(f32,f32,f32,f32)){
    let m=40.0;
    let (wx,wy)=((b.2-b.0).max(1.0),(b.3-b.1).max(1.0));
    let (cx,cy)=((b.0+b.2)/2.0,(b.1+b.3)/2.0);
    self.zoom=((rect.width()-2.0*m)/wx).min((rect.height()-2.0*m)/wy).clamp(ZOOM_MIN,ZOOM_MAX);
    self.pan=Vec2::new(rect.width()/2.0-cx*self.zoom,-rect.height()/2.0+cy*self.zoom);
  }
  fn fit(&mut self,rect:Rect,project:&Project,kind:FitKind){
    let work=(0.0,0.0,project.work_x.max(1.0),project.work_y.max(1.0));
    let b=if kind==FitKind::Shapes{self.content_box(project).unwrap_or(work)}else{work};
    self.fit_box(rect,b);
  }
  /// Extent of all visible shapes (with leader points) and the origin marker.
  fn content_box(&self,project:&Project)->Option<(f32,f32,f32,f32)>{
    fn grow(b:&mut Option<(f32,f32,f32,f32)>,x0:f32,y0:f32,x1:f32,y1:f32){*b=Some(match *b{Some(o)=>(o.0.min(x0),o.1.min(y0),o.2.max(x1),o.3.max(y1)),None=>(x0,y0,x1,y1)});}
    let mut b:Option<(f32,f32,f32,f32)>=None;
    for s in project.shapes.iter().filter(|s|self.layer_visible(s.layer)){
      let (x0,y0,x1,y1)=s.extent();grow(&mut b,x0,y0,x1,y1);
      if let Some(l)=s.leader{grow(&mut b,l.0,l.1,l.0,l.1);}
    }
    b?;
    let (ox,oy)=project.origin;grow(&mut b,ox,oy,ox,oy);
    b
  }
  fn set_tool(&mut self,tool:Tool,project:&mut Project){self.cancel(project);self.tool=tool;if tool!=Tool::Leader{self.leader_for=None;}}
  fn cancel_soft(&mut self){self.draw_start=None;self.move_base=None;self.poly_pts.clear();}
  /// Abort any operation in progress; restore shapes changed by an unfinished drag.
  fn cancel(&mut self,project:&mut Project){
    match std::mem::replace(&mut self.drag,Drag::None){
      Drag::Move{originals,..}=>{for (i,o,_) in originals{if let Some(s)=project.shapes.get_mut(i){*s=o;}}}
      Drag::Handle{index,original,..}=>{if let Some(s)=project.shapes.get_mut(index){*s=original;}}
      Drag::Origin{original}=>project.origin=original,
      Drag::FenceCorner{original}=>{project.fence.w=original.0;project.fence.h=original.1;}
      _=>{}
    }
    self.cancel_soft();
    if self.tool==Tool::Leader{self.tool=Tool::Select;self.leader_for=None;}
  }
  /// True while an edit is in progress (drag, drawing, Move base point, polyline vertices).
  pub fn busy(&self)->bool{self.draw_start.is_some()||self.move_base.is_some()||!self.poly_pts.is_empty()||!matches!(self.drag,Drag::None)}

  // ---------------------------------------------------------------- geometry helpers
  fn hit_shape(&self,s:&Shape,w:(f32,f32),tol:f32)->bool{
    let seg=|a:Pt,b:Pt|{let (dx,dy)=(b.0-a.0,b.1-a.1);let l2=dx*dx+dy*dy;
      let t=if l2>0.0{(((w.0-a.0)*dx+(w.1-a.1)*dy)/l2).clamp(0.0,1.0)}else{0.0};
      let (px,py)=(a.0+t*dx,a.1+t*dy);((w.0-px).powi(2)+(w.1-py).powi(2)).sqrt()};
    let near_path=|pts:&[Pt],closed:bool,t:f32|{let n=pts.len();if n<2{return false;}
      (0..if closed{n}else{n-1}).any(|i|seg(pts[i],pts[(i+1)%n])<=t)};
    let inside=|rings:&[&Vec<Pt>]|{let mut c=false;for r in rings{let n=r.len();for k in 0..n{let (a,b)=(r[k],r[(k+1)%n]);
      if (a.1>w.1)!=(b.1>w.1)&&w.0<a.0+(w.1-a.1)*(b.0-a.0)/(b.1-a.1){c=!c;}}}c};
    let (x0,y0,x1,y1)=s.extent();
    match s.kind{
      ShapeKind::Rectangle=>w.0>=x0-tol&&w.0<=x1+tol&&w.1>=y0-tol&&w.1<=y1+tol,
      ShapeKind::Circle=>{let (cx,cy,rx,ry)=((x0+x1)/2.0,(y0+y1)/2.0,(x1-x0)/2.0+tol,(y1-y0)/2.0+tol);
        ((w.0-cx)/rx).powi(2)+((w.1-cy)/ry).powi(2)<=1.0}
      ShapeKind::Line=>near_path(&s.path(),false,tol.max(s.width/2.0)),
      ShapeKind::Polyline=>near_path(&s.points,s.closed,tol.max(s.width/2.0))||(s.closed&&inside(&[&s.points])),
      ShapeKind::Region=>{let rings:Vec<&Vec<Pt>>=s.polys.iter().flat_map(|p|std::iter::once(&p.outer).chain(p.holes.iter())).collect();
        inside(&rings)||rings.iter().any(|r|near_path(r,true,tol))}
      ShapeKind::Text=>{(w.0>=x0-tol&&w.0<=x1+tol&&w.1>=y0-tol&&w.1<=y1+tol)||s.leader.is_some_and(|l|seg(Self::leader_anchor(s),l)<=tol)}
    }
  }
  fn hit(&self,project:&Project,w:(f32,f32))->Option<usize>{
    let tol=PICK_PX/self.zoom;
    project.shapes.iter().enumerate().rev().find(|(_,s)|self.layer_visible(s.layer)&&self.hit_shape(s,w,tol)).map(|(i,_)|i)
  }
  /// Point of a note where its leader starts: the point of the (slightly enlarged) text box nearest
  /// the leader point, so the leader never crosses the text.
  fn leader_anchor(s:&Shape)->Pt{
    let (x0,y0,x1,y1)=s.extent();let m=s.h*0.3;
    match s.leader{
      Some(l)=>{let p=(l.0.clamp(x0-m,x1+m),l.1.clamp(y0-m,y1+m));if p==l{(x0-m,(y0+y1)/2.0)}else{p}}
      None=>(x0-m,(y0+y1)/2.0),
    }
  }
  fn near(&self,rect:Rect,w:(f32,f32),p:Pos2,px:f32)->bool{let q=self.to_screen(rect,w.0,w.1);(q.x-p.x).abs()<=px&&(q.y-p.y).abs()<=px}
  fn fence_corner(project:&Project)->(f32,f32){(project.origin.0+project.fence.w,project.origin.1+project.fence.h)}
  fn handles(s:&Shape)->Vec<(f32,f32)>{
    let (x0,y0,x1,y1)=s.extent();let (xm,ym)=((x0+x1)/2.0,(y0+y1)/2.0);
    match s.kind{
      ShapeKind::Line=>vec![(s.x,s.y),(s.x+s.w,s.y+s.h)],
      ShapeKind::Rectangle=>vec![(x0,y0),(xm,y0),(x1,y0),(x1,ym),(x1,y1),(xm,y1),(x0,y1),(x0,ym)],
      ShapeKind::Circle=>vec![(xm,y0),(x1,ym),(xm,y1),(x0,ym)],
      ShapeKind::Polyline=>s.points.clone(),
      ShapeKind::Region|ShapeKind::Text=>vec![],
    }
  }
  fn handle_at(&self,rect:Rect,s:&Shape,p:Pos2)->Option<usize>{
    Self::handles(s).iter().position(|h|{let q=self.to_screen(rect,h.0,h.1);(q.x-p.x).abs()<=HANDLE_PX+2.0&&(q.y-p.y).abs()<=HANDLE_PX+2.0})
  }
  fn apply_handle(original:&Shape,handle:usize,p:(f32,f32))->Shape{
    let mut s=original.clone();
    let (mut x0,mut y0,mut x1,mut y1)=original.extent();
    match s.kind{
      ShapeKind::Line=>{
        let (ex,ey)=(original.x+original.w,original.y+original.h);
        if handle==0{s.x=p.0;s.y=p.1;s.w=ex-p.0;s.h=ey-p.1;}else{s.w=p.0-original.x;s.h=p.1-original.y;}
      }
      ShapeKind::Rectangle=>{
        match handle{0=>{x0=p.0;y0=p.1}1=>{y0=p.1}2=>{x1=p.0;y0=p.1}3=>{x1=p.0}4=>{x1=p.0;y1=p.1}5=>{y1=p.1}6=>{x0=p.0;y1=p.1}_=>{x0=p.0}}
        s.x=x0.min(x1);s.y=y0.min(y1);s.w=(x1-x0).abs();s.h=(y1-y0).abs();
      }
      ShapeKind::Circle=>{
        let (cx,cy)=((x0+x1)/2.0,(y0+y1)/2.0);
        let r=(if handle==0||handle==2{(p.1-cy).abs()}else{(p.0-cx).abs()}).max(0.001);
        s.x=cx-r;s.y=cy-r;s.w=2.0*r;s.h=2.0*r;
      }
      ShapeKind::Polyline=>{if let Some(v)=s.points.get_mut(handle){*v=p;}s.sync_box();}
      ShapeKind::Region|ShapeKind::Text=>{}
    }
    s
  }
  fn path_shape(&self,painter:&egui::Painter,rect:Rect,pts:&[Pt],closed:bool,dx:f32,dy:f32,stroke:Stroke,dashed:bool){
    if pts.len()<2{return;}
    let mut v:Vec<Pos2>=pts.iter().map(|p|self.to_screen(rect,p.0+dx,p.1+dy)).collect();
    if dashed{if closed{v.push(v[0]);}painter.extend(egui::Shape::dashed_line(&v,stroke,6.0,4.0));}
    else if closed{painter.add(egui::Shape::closed_line(v,stroke));}else{painter.add(egui::Shape::line(v,stroke));}
  }
  fn paint_shape(&self,painter:&egui::Painter,rect:Rect,s:&Shape,dx:f32,dy:f32,stroke:Stroke){
    let dashed=s.layer==Layer::Guides;
    let (x0,y0,x1,y1)=s.extent();
    match s.kind{
      ShapeKind::Rectangle=>self.path_shape(painter,rect,&[(x0,y0),(x1,y0),(x1,y1),(x0,y1)],true,dx,dy,stroke,dashed),
      ShapeKind::Line=>self.path_shape(painter,rect,&s.path(),false,dx,dy,stroke,dashed),
      ShapeKind::Circle=>{
        let (cx,cy,rx,ry)=((x0+x1)/2.0,(y0+y1)/2.0,(x1-x0)/2.0,(y1-y0)/2.0);
        let pts:Vec<Pt>=(0..96).map(|i|{let t=std::f32::consts::TAU*i as f32/96.0;(cx+rx*t.cos(),cy+ry*t.sin())}).collect();
        self.path_shape(painter,rect,&pts,true,dx,dy,stroke,dashed);
      }
      ShapeKind::Polyline=>self.path_shape(painter,rect,&s.points,s.closed,dx,dy,stroke,dashed),
      ShapeKind::Region=>{for p in &s.polys{self.path_shape(painter,rect,&p.outer,true,dx,dy,stroke,dashed);for h in &p.holes{self.path_shape(painter,rect,h,true,dx,dy,stroke,dashed);}}}
      ShapeKind::Text=>{
        if s.layer==Layer::Burn{
          match s.font{
            TextFont::Stroke=>{for p in font::stroke_text(&s.text,s.x,s.y,s.h){self.path_shape(painter,rect,&p,false,dx,dy,stroke,false);}}
            TextFont::Outline=>{for r in font::outline_rings(&s.text,s.x,s.y,s.h){self.path_shape(painter,rect,&r,true,dx,dy,stroke,false);}}
          }
        }else{
          let cap_px=s.h*self.zoom;
          if cap_px>=3.0{
            let q=self.to_screen(rect,s.x+dx,s.y+dy);
            painter.text(q+Vec2::new(0.0,cap_px*0.28),egui::Align2::LEFT_BOTTOM,&s.text,egui::FontId::proportional(font::screen_font_size(cap_px)),stroke.color);
          }else{
            let r=Rect::from_two_pos(self.to_screen(rect,x0+dx,y0+dy),self.to_screen(rect,x1+dx,y1+dy));
            painter.rect_filled(r,0.0,stroke.color.gamma_multiply(0.35));
          }
        }
        if let Some(l)=s.leader{
          let a=Self::leader_anchor(s);
          let (pa,pb)=(self.to_screen(rect,a.0+dx,a.1+dy),self.to_screen(rect,l.0,l.1));
          painter.line_segment([pa,pb],Stroke::new(1.0_f32,stroke.color));
          painter.circle_filled(pb,2.5,stroke.color);
        }
      }
    }
  }
  /// Create a shape from two clicked points with the active drawing tool.
  fn finish_draw(&mut self,project:&mut Project,selected:&mut Vec<usize>,end:(f32,f32))->bool{
    let Some(start)=self.draw_start.take() else {return false;};
    let (dx,dy)=(end.0-start.0,end.1-start.1);
    let mut shape=match self.tool{
      Tool::Rectangle=>{
        if dx.abs()<1e-4||dy.abs()<1e-4{return false;}
        Shape::rectangle(start.0.min(end.0),start.1.min(end.1),dx.abs(),dy.abs())
      }
      Tool::Line=>{if (dx*dx+dy*dy).sqrt()<1e-4{return false;}Shape::line(start,end)}
      Tool::Circle=>{let r=(dx*dx+dy*dy).sqrt();if r<1e-4{return false;}Shape::circle(start,r)}
      _=>return false,
    };
    shape.layer=self.active_layer;
    project.shapes.push(shape);
    *selected=vec![project.shapes.len()-1];
    true
  }
  /// Finish the polyline being drawn (needs 2 distinct points, 3 when closed).
  fn finish_poly(&mut self,project:&mut Project,selected:&mut Vec<usize>,closed:bool)->bool{
    let mut pts=std::mem::take(&mut self.poly_pts);
    pts.dedup_by(|a,b|(a.0-b.0).abs()<1e-5&&(a.1-b.1).abs()<1e-5);
    if pts.len()<2||(closed&&pts.len()<3){return false;}
    let mut s=Shape::polyline(pts,closed);s.layer=self.active_layer;
    project.shapes.push(s);*selected=vec![project.shapes.len()-1];
    true
  }
  /// Merge the selected closed shapes into one Region (settings from the first selected shape).
  fn merge(&mut self,project:&mut Project,selected:&mut Vec<usize>,ev:&mut Events){
    let mut idx=selected.clone();idx.sort_unstable();idx.dedup();idx.retain(|&i|i<project.shapes.len());
    if idx.len()<2{ev.notice=Some("Select two or more touching closed shapes, then Merge".into());return;}
    let layer=project.shapes[idx[0]].layer;
    if idx.iter().any(|&i|project.shapes[i].layer!=layer){
      self.refused=idx.iter().copied().filter(|&i|project.shapes[i].layer!=layer).collect();self.refused_at=Some(Instant::now());
      ev.notice=Some("Not merged: the shapes are on different layers".into());return;
    }
    let shapes:Vec<&Shape>=idx.iter().map(|&i|&project.shapes[i]).collect();
    match paths::merge(&shapes){
      Ok(polys)=>{
        let src=selected.first().copied().filter(|i|*i<project.shapes.len()).unwrap_or(idx[0]);
        let mut m=project.shapes[src].clone();
        m.kind=ShapeKind::Region;m.polys=polys;m.points.clear();m.closed=false;m.text.clear();m.leader=None;m.attach=None;
        m.uid=geometry::new_id();m.sync_box();
        let old:Vec<String>=idx.iter().map(|&i|project.shapes[i].uid.clone()).collect();
        for &i in idx.iter().rev(){project.shapes.remove(i);}
        let at=idx[0].min(project.shapes.len());
        let uid=m.uid.clone();
        project.shapes.insert(at,m);
        for s in &mut project.shapes{if s.attach.as_ref().is_some_and(|a|old.contains(a)){s.attach=Some(uid.clone());}}
        *selected=vec![at];
        ev.changed=true;ev.notice=Some(format!("Merged {} shapes into one outline",idx.len()));
      }
      Err((msg,bad))=>{self.refused=bad.iter().map(|&k|idx[k]).collect();self.refused_at=Some(Instant::now());ev.notice=Some(msg);}
    }
  }

  // ---------------------------------------------------------------- tool strip
  fn group_draw(&mut self,ui:&mut egui::Ui,project:&mut Project){
    ui.horizontal(|ui|{
      for (tool,icon,tip) in [(Tool::Select,Icon::Select,"Select"),(Tool::Pan,Icon::Pan,"Pan"),(Tool::Move,Icon::Move,"Move (click base point, click destination)"),
                              (Tool::Origin,Icon::Origin,"Job origin: click to place job 0,0 (the spot you zero on the machine)")]{
        if widgets::icon_button(ui,icon,self.tool==tool,true,tip).clicked(){self.set_tool(tool,project);}
      }
      ui.separator();
      for (tool,icon,tip) in [(Tool::Rectangle,Icon::Rectangle,"Rectangle (click corner, click opposite corner)"),
                              (Tool::Line,Icon::Line,"Line / trace (click start, click end). Width and fill in Shape Properties."),
                              (Tool::Polyline,Icon::Polyline,"Polyline (click vertices; click the first vertex to close; double-click or Enter to finish open; Backspace removes the last vertex)"),
                              (Tool::Circle,Icon::Circle,"Circle (click centre, click radius)"),
                              (Tool::Text,Icon::Text,"Text (click to place; edit in Shape Properties). Notes layer: shown only. Burn layer: burned.")]{
        if widgets::icon_button(ui,icon,self.tool==tool,true,tip).clicked(){self.set_tool(tool,project);}
      }
    });
  }
  fn group_edit(&mut self,ui:&mut egui::Ui,project:&mut Project,selected:&mut Vec<usize>,can_undo:bool,can_redo:bool,ev:&mut Events){
    ui.horizontal(|ui|{
      if widgets::icon_button(ui,Icon::Undo,false,can_undo,"Undo (Ctrl+Z)").clicked(){self.cancel(project);ev.undo=true;}
      if widgets::icon_button(ui,Icon::Redo,false,can_redo,"Redo (Ctrl+Y / Ctrl+Shift+Z)").clicked(){self.cancel(project);ev.redo=true;}
      ui.separator();
      let has_sel=!selected.is_empty();
      if widgets::icon_button(ui,Icon::Duplicate,false,has_sel,"Duplicate selected").clicked(){
        let off=if self.snap{self.grid_spacing.max(1.0)}else{2.0};
        let mut new_sel=vec![];
        for &i in selected.iter(){if let Some(s)=project.shapes.get(i).cloned(){let mut d=s;d.translate(off,off);d.uid=geometry::new_id();project.shapes.push(d);new_sel.push(project.shapes.len()-1);}}
        *selected=new_sel;ev.changed=true;
      }
      if widgets::icon_button(ui,Icon::Delete,false,has_sel,"Delete selected (Delete)").clicked(){
        self.cancel(project);delete_shapes(project,selected);selected.clear();ev.changed=true;
      }
      if widgets::icon_button(ui,Icon::Clear,false,!project.shapes.is_empty(),"Clear all shapes (all layers)").clicked(){
        self.cancel(project);project.shapes.clear();selected.clear();ev.changed=true;
      }
      ui.separator();
      if widgets::icon_button(ui,Icon::Merge,false,selected.len()>=2,"Merge: selected closed shapes that share an edge or overlap become one outline with one fill. Corner contact or a gap does not merge. Undo restores the pieces.").clicked(){
        self.cancel(project);self.merge(project,selected,ev);
      }
      if widgets::icon_button(ui,Icon::Order,self.show_order,true,"Show burn order numbers (change the order in Shape Properties)").clicked(){self.show_order=!self.show_order;}
    });
  }
  fn group_view(&mut self,ui:&mut egui::Ui){
    ui.horizontal(|ui|{
      if widgets::icon_button(ui,Icon::Grid,self.grid,true,"Grid on/off").clicked(){self.grid=!self.grid;}
      egui::ComboBox::from_id_salt("grid_spacing").width(78.0).selected_text(format!("{} mm",self.grid_spacing))
        .show_ui(ui,|ui|{for s in GRID_SPACINGS{ui.selectable_value(&mut self.grid_spacing,s,format!("{s} mm"));}})
        .response.on_hover_text("Grid spacing (also the snap increment)");
      if widgets::icon_button(ui,Icon::Snap,self.snap,true,"Snap to grid on/off").clicked(){self.snap=!self.snap;}
    });
    ui.separator();
    ui.horizontal(|ui|{
      ui.label("Zoom");
      let mut z=self.zoom;
      if widgets::num_f32_mul(ui,"canvas_zoom",&mut z,1.25,ZOOM_MIN,ZOOM_MAX,2,"px/mm"){
        if self.last_rect.is_positive(){let c=self.last_rect.center();self.zoom_about(self.last_rect,c,z);}else{self.zoom=z;}
      }
      if widgets::icon_button(ui,Icon::Fit,false,true,"Fit work area").clicked(){self.fit_pending=Some(FitKind::Work);}
      if widgets::icon_button(ui,Icon::FitContent,false,true,"Fit shapes (all visible shapes and the job origin)").clicked(){self.fit_pending=Some(FitKind::Shapes);}
    });
  }
  fn group_layers(&mut self,ui:&mut egui::Ui){
    ui.horizontal(|ui|{
      ui.label("Layer");
      egui::ComboBox::from_id_salt("active_layer").width(76.0).selected_text(self.active_layer.name()).show_ui(ui,|ui|{
        ui.selectable_value(&mut self.active_layer,Layer::Burn,"Burn").on_hover_text("Generates G-code");
        ui.selectable_value(&mut self.active_layer,Layer::Notes,"Notes").on_hover_text("Shown on the canvas and in Simulate, never burned");
        ui.selectable_value(&mut self.active_layer,Layer::Guides,"Guides").on_hover_text("Reference outlines (tape edge, clearance), never burned");
      }).response.on_hover_text("Layer for new shapes");
      let hidden_n=self.hidden.iter().filter(|h|**h).count();
      ui.menu_button(if hidden_n>0{format!("Show ({hidden_n} hidden)")}else{"Show".to_string()},|ui|{
        for l in Layer::ALL{let mut vis=!self.hidden[l.index()];if ui.checkbox(&mut vis,l.name()).changed(){self.hidden[l.index()]=!vis;}}
        ui.weak("Hidden layers are not drawn or picked.\nA hidden Burn layer still burns.");
      });
    });
  }
  fn group_fence(&mut self,ui:&mut egui::Ui,project:&mut Project,ev:&mut Events){
    ui.horizontal(|ui|{
      if widgets::icon_button(ui,Icon::Fence,project.fence.on,true,"Work fence on/off: program moves checked against it, jogs outside it refused once zero is set").clicked(){
        project.fence.on=!project.fence.on;ev.changed=true;
      }
      ui.label("W");
      ev.changed|=widgets::num_f32(ui,"fence_w",&mut project.fence.w,self.grid_spacing.max(1.0),1.0,3000.0,1,"");
      ui.label("H");
      ev.changed|=widgets::num_f32(ui,"fence_h",&mut project.fence.h,self.grid_spacing.max(1.0),1.0,3000.0,1,"mm");
    }).response.on_hover_text("Work fence size from the job origin");
  }
  /// Canvas tool strip. `docked`: two rows (draw / edit, view / layers / fence); floating: one flow
  /// that wraps to the window width. Groups never split.
  pub fn toolbar(&mut self,ui:&mut egui::Ui,project:&mut Project,selected:&mut Vec<usize>,docked:bool,can_undo:bool,can_redo:bool)->Events{
    let mut ev=Events::default();
    ui.spacing_mut().item_spacing=Vec2::new(4.0,4.0);
    if docked{
      ui.horizontal_wrapped(|ui|{self.group_draw(ui,project);ui.separator();self.group_edit(ui,project,selected,can_undo,can_redo,&mut ev);});
      ui.horizontal_wrapped(|ui|{self.group_view(ui);ui.separator();self.group_layers(ui);ui.separator();self.group_fence(ui,project,&mut ev);});
    }else{
      ui.horizontal_wrapped(|ui|{
        self.group_draw(ui,project);ui.separator();self.group_edit(ui,project,selected,can_undo,can_redo,&mut ev);ui.separator();
        self.group_view(ui);ui.separator();self.group_layers(ui);ui.separator();self.group_fence(ui,project,&mut ev);
      });
    }
    ev
  }

  // ---------------------------------------------------------------- canvas
  /// Draws the canvas and handles interaction.
  pub fn show(&mut self,ui:&mut egui::Ui,project:&mut Project,selected:&mut Vec<usize>,plan:&Plan,sim:&SimOverlay<'_>)->Events{
    let mut ev=Events::default();
    selected.retain(|&i|i<project.shapes.len());
    if self.leader_for.is_some_and(|i|i>=project.shapes.len()){self.leader_for=None;if self.tool==Tool::Leader{self.tool=Tool::Select;}}
    if self.refused_at.is_some_and(|t|t.elapsed()>REFUSED_FOR){self.refused.clear();self.refused_at=None;}
    let status_h=24.0;
    let avail=ui.available_size();
    let (resp,painter)=ui.allocate_painter(Vec2::new(avail.x.max(100.0),(avail.y-status_h).max(100.0)),Sense::click_and_drag());
    let rect=resp.rect;
    // Fit once the canvas size is stable (first frames can report a provisional window size).
    if let Some(kind)=self.fit_pending{self.fit(rect,project,kind);if (rect.size()-self.last_rect.size()).length()<0.5{self.fit_pending=None;}}
    self.last_rect=rect;
    let pointer=resp.interact_pointer_pos().or(resp.hover_pos());
    let world=pointer.map(|p|self.to_world(rect,p));
    let snapped=world.map(|w|self.snap_pt(w));
    let mods=ui.input(|i|i.modifiers);
    let press=ui.input(|i|i.pointer.press_origin());

    // ------------------------------------------------ view navigation
    if resp.hovered(){
      let dy=ui.input(|i|i.smooth_scroll_delta.y);
      if dy!=0.0{if let Some(p)=pointer{self.zoom_about(rect,p,self.zoom*(dy*0.0025).exp());}}
    }
    if resp.drag_started_by(PointerButton::Secondary){
      self.drag=if mods.shift{Drag::Zoom{anchor:press.unwrap_or(rect.center())}}else{Drag::Pan};
    }
    if resp.drag_started_by(PointerButton::Middle){self.drag=Drag::Zoom{anchor:press.unwrap_or(rect.center())};}

    // ------------------------------------------------ primary drag start
    if resp.drag_started_by(PointerButton::Primary){
      let origin=press.unwrap_or(rect.center());
      let w=self.to_world(rect,origin);
      match self.tool{
        Tool::Pan=>self.drag=Drag::Pan,
        Tool::Select if self.near(rect,project.origin,origin,HANDLE_PX+4.0)=>{self.drag=Drag::Origin{original:project.origin};}
        Tool::Select if self.near(rect,Self::fence_corner(project),origin,HANDLE_PX+3.0)=>{self.drag=Drag::FenceCorner{original:(project.fence.w,project.fence.h)};}
        Tool::Origin|Tool::Polyline|Tool::Text|Tool::Leader=>{}
        Tool::Select|Tool::Move=>{
          let handle=if self.tool==Tool::Select&&selected.len()==1{
            project.shapes.get(selected[0]).and_then(|s|self.handle_at(rect,s,origin)).map(|h|(selected[0],h))}else{None};
          if let Some((index,handle))=handle{
            self.drag=Drag::Handle{index,handle,original:project.shapes[index].clone()};
          }else if let Some(i)=self.hit(project,w){
            if !selected.contains(&i){if !mods.shift{selected.clear();}selected.push(i);}
            self.move_base=None;
            // Flag: keep the leader point (a note moved without the shape it points to).
            let uids:Vec<String>=selected.iter().map(|&i|project.shapes[i].uid.clone()).collect();
            let mut originals:Vec<(usize,Shape,bool)>=selected.iter().map(|&i|{let s=&project.shapes[i];
              (i,s.clone(),!s.attach.as_ref().is_some_and(|a|uids.contains(a)))}).collect();
            for f in followers(project,selected){originals.push((f,project.shapes[f].clone(),false));}
            self.drag=Drag::Move{start:self.snap_pt(w),originals};
          }else if self.tool==Tool::Select{
            self.drag=Drag::Box{start:origin};
          }
        }
        Tool::Rectangle|Tool::Line|Tool::Circle=>{if self.draw_start.is_none(){self.draw_start=Some(self.snap_pt(w));}}
      }
    }

    // ------------------------------------------------ drag in progress
    if resp.dragged(){
      let delta=resp.drag_delta();
      match &self.drag{
        Drag::Pan=>self.pan+=delta,
        Drag::Zoom{anchor}=>{let a=*anchor;self.zoom_about(rect,a,self.zoom*(-delta.y*0.01).exp());}
        Drag::Move{start,originals}=>{if let Some(c)=snapped{
          let (dx,dy)=(c.0-start.0,c.1-start.1);
          for (i,o,keep) in originals{if let Some(s)=project.shapes.get_mut(*i){*s=translated(o,dx,dy,*keep);}}
        }}
        Drag::Handle{index,handle,original}=>{if let Some(c)=snapped{
          let s=Self::apply_handle(original,*handle,c);if let Some(t)=project.shapes.get_mut(*index){*t=s;}
        }}
        Drag::Origin{..}=>{if let Some(c)=snapped{project.origin=c;}}
        Drag::FenceCorner{..}=>{if let Some(c)=snapped{project.fence.w=(c.0-project.origin.0).max(1.0);project.fence.h=(c.1-project.origin.1).max(1.0);}}
        _=>{}
      }
    }

    // ------------------------------------------------ drag end
    if resp.drag_stopped(){
      match std::mem::replace(&mut self.drag,Drag::None){
        Drag::Box{start}=>{if let Some(end)=pointer{
          let (a,b)=(self.to_world(rect,start),self.to_world(rect,end));
          let (bx0,by0,bx1,by1)=(a.0.min(b.0),a.1.min(b.1),a.0.max(b.0),a.1.max(b.1));
          let window=end.x>=start.x;
          if !mods.shift{selected.clear();}
          for (i,s) in project.shapes.iter().enumerate(){
            if !self.layer_visible(s.layer){continue;}
            let (x0,y0,x1,y1)=s.extent();
            let pick=if window{x0>=bx0&&x1<=bx1&&y0>=by0&&y1<=by1}else{x0<=bx1&&x1>=bx0&&y0<=by1&&y1>=by0};
            if pick&&!selected.contains(&i){selected.push(i);}
          }
        }}
        Drag::Move{..}|Drag::Handle{..}|Drag::Origin{..}|Drag::FenceCorner{..}=>ev.changed=true,
        _=>{}
      }
      if resp.drag_stopped_by(PointerButton::Primary)&&matches!(self.tool,Tool::Rectangle|Tool::Line|Tool::Circle){
        if let Some(e)=snapped{ev.changed|=self.finish_draw(project,selected,e);}
      }
      if resp.drag_stopped_by(PointerButton::Primary)&&self.tool==Tool::Origin{
        if let Some(e)=snapped{project.origin=e;ev.changed=true;}
      }
    }

    // ------------------------------------------------ clicks
    if resp.clicked(){if let (Some(w),Some(s),Some(p))=(world,snapped,pointer){
      match self.tool{
        Tool::Select=>{
          match self.hit(project,w){
            Some(i)=>{if mods.shift{if let Some(p)=selected.iter().position(|&j|j==i){selected.remove(p);}else{selected.push(i);}}else{*selected=vec![i];}}
            None=>{if !mods.shift{selected.clear();}}
          }
        }
        Tool::Pan=>{}
        Tool::Origin=>{project.origin=s;ev.changed=true;}
        Tool::Move=>{
          if selected.is_empty(){if let Some(i)=self.hit(project,w){*selected=vec![i];}}
          else if let Some(base)=self.move_base.take(){
            let (dx,dy)=(s.0-base.0,s.1-base.1);
            let foll=followers(project,selected);
            let uids:Vec<String>=selected.iter().filter_map(|&i|project.shapes.get(i)).map(|s|s.uid.clone()).collect();
            for &i in selected.iter(){if let Some(sh)=project.shapes.get_mut(i){let keep=!sh.attach.as_ref().is_some_and(|a|uids.contains(a));*sh=translated(sh,dx,dy,keep);}}
            for i in foll{if let Some(sh)=project.shapes.get_mut(i){sh.translate(dx,dy);}}
            ev.changed=true;
          }else{self.move_base=Some(s);}
        }
        Tool::Rectangle|Tool::Line|Tool::Circle=>{
          if self.draw_start.is_none(){self.draw_start=Some(s);}else{ev.changed|=self.finish_draw(project,selected,s);}
        }
        Tool::Polyline=>{
          if self.poly_pts.len()>=3&&self.near(rect,self.poly_pts[0],p,HANDLE_PX+3.0){ev.changed|=self.finish_poly(project,selected,true);}
          else if self.poly_pts.last()!=Some(&s){self.poly_pts.push(s);}
        }
        Tool::Text=>{
          let text=match self.active_layer{Layer::Burn=>"S01",Layer::Notes=>"Note",Layer::Guides=>"Guide"};
          project.shapes.push(Shape::text(s,text,3.0,self.active_layer));
          *selected=vec![project.shapes.len()-1];
          self.tool=Tool::Select;ev.changed=true;
          ev.notice=Some("Text placed: edit it in Shape Properties".into());
        }
        Tool::Leader=>{
          if let Some(i)=self.leader_for.take(){
            let target=self.hit(project,w).filter(|&j|j!=i).and_then(|j|project.shapes.get(j)).map(|t|t.uid.clone()).filter(|u|!u.is_empty());
            if let Some(note)=project.shapes.get_mut(i){note.leader=Some(s);note.attach=target.clone();}
            ev.notice=Some(if target.is_some(){"Leader set; the note is attached to that shape and moves with it".into()}else{"Leader set (no shape under the point: not attached)".into()});
            *selected=vec![i];ev.changed=true;
          }
          self.tool=Tool::Select;
        }
      }
    }}
    if resp.double_clicked()&&self.tool==Tool::Polyline&&self.poly_pts.len()>=2{ev.changed|=self.finish_poly(project,selected,false);}
    if resp.secondary_clicked(){self.cancel(project);}

    // ------------------------------------------------ keyboard (only when no text field has focus)
    if ui.ctx().memory(|m|m.focused().is_none()){
      if ui.input(|i|i.key_pressed(Key::Escape)){
        if self.busy()||self.tool==Tool::Leader{self.cancel(project);}else if !selected.is_empty(){selected.clear();}else{self.tool=Tool::Select;}
      }
      if self.tool==Tool::Polyline&&!self.poly_pts.is_empty(){
        if ui.input(|i|i.key_pressed(Key::Enter)){ev.changed|=self.finish_poly(project,selected,false);}
        if ui.input(|i|i.key_pressed(Key::Backspace)){self.poly_pts.pop();}
      }
      if ui.input(|i|i.key_pressed(Key::Delete))&&!selected.is_empty(){
        self.cancel(project);delete_shapes(project,selected);selected.clear();ev.changed=true;
      }
    }

    // ------------------------------------------------ cursor
    if resp.hovered()||!matches!(self.drag,Drag::None){
      let icon=match (&self.drag,self.tool){
        (Drag::Pan,_)=>CursorIcon::Grabbing,
        (Drag::Zoom{..},_)=>CursorIcon::ZoomIn,
        (Drag::Move{..},_)=>CursorIcon::Move,
        (Drag::Handle{..}|Drag::Origin{..}|Drag::FenceCorner{..},_)=>CursorIcon::Crosshair,
        (_,Tool::Pan)=>CursorIcon::Grab,
        (_,Tool::Move)=>CursorIcon::Move,
        (_,Tool::Rectangle|Tool::Line|Tool::Polyline|Tool::Circle|Tool::Origin|Tool::Text|Tool::Leader)=>CursorIcon::Crosshair,
        (_,Tool::Select)=>{
          let on_marker=pointer.is_some_and(|p|self.near(rect,project.origin,p,HANDLE_PX+4.0)||self.near(rect,Self::fence_corner(project),p,HANDLE_PX+3.0));
          let on_handle=on_marker||selected.len()==1&&pointer.and_then(|p|project.shapes.get(selected[0]).and_then(|s|self.handle_at(rect,s,p))).is_some();
          if on_handle{CursorIcon::Crosshair}else if world.and_then(|w|self.hit(project,w)).is_some(){CursorIcon::PointingHand}else{CursorIcon::Default}
        }
      };
      ui.ctx().set_cursor_icon(icon);
    }

    // ================================================ painting
    painter.rect_filled(rect,0.0,Color32::BLACK);
    // ---- grid: minor at spacing, major every 10 ----
    if self.grid{
      let (wx0,wy1)=self.to_world(rect,rect.left_top());
      let (wx1,wy0)=self.to_world(rect,rect.right_bottom());
      let s=self.grid_spacing;
      for (step,major) in [(s,false),(s*10.0,true)]{
        if step*self.zoom<6.0{continue;}
        let color=if major{Color32::from_gray(52)}else{Color32::from_gray(27)};
        let (k0,k1)=((wx0/step).ceil() as i64,(wx1/step).floor() as i64);
        if k1-k0<4000{for k in k0..=k1{if !major&&k%10==0{continue;}let x=self.to_screen(rect,k as f32*step,0.0).x;
          painter.line_segment([Pos2::new(x,rect.top()),Pos2::new(x,rect.bottom())],Stroke::new(1.0_f32,color));}}
        let (k0,k1)=((wy0/step).ceil() as i64,(wy1/step).floor() as i64);
        if k1-k0<4000{for k in k0..=k1{if !major&&k%10==0{continue;}let y=self.to_screen(rect,0.0,k as f32*step).y;
          painter.line_segment([Pos2::new(rect.left(),y),Pos2::new(rect.right(),y)],Stroke::new(1.0_f32,color));}}
      }
    }
    // ---- work area and origin ----
    let work=Rect::from_two_pos(self.to_screen(rect,0.0,0.0),self.to_screen(rect,project.work_x,project.work_y));
    painter.rect_stroke(work,0.0,Stroke::new(1.5_f32,Color32::from_gray(115)),egui::StrokeKind::Middle);
    painter.text(work.left_top()+Vec2::new(4.0,-4.0),egui::Align2::LEFT_BOTTOM,format!("{} × {} mm",project.work_x,project.work_y),egui::FontId::proportional(11.0),Color32::from_gray(120));
    let o=self.to_screen(rect,0.0,0.0);
    painter.arrow(o,Vec2::new(28.0,0.0),Stroke::new(1.6_f32,Color32::from_rgb(220,80,80)));
    painter.arrow(o,Vec2::new(0.0,-28.0),Stroke::new(1.6_f32,Color32::from_rgb(90,200,110)));
    // ---- work fence (from the origin marker) ----
    let fr=Rect::from_two_pos(self.to_screen(rect,project.origin.0,project.origin.1),self.to_screen(rect,Self::fence_corner(project).0,Self::fence_corner(project).1));
    if project.fence.on{
      painter.rect_filled(fr,0.0,Color32::from_rgba_unmultiplied(240,150,40,14));
      painter.rect_stroke(fr,0.0,Stroke::new(1.6_f32,C_FENCE),egui::StrokeKind::Middle);
      painter.text(fr.left_top()+Vec2::new(4.0,-4.0),egui::Align2::LEFT_BOTTOM,format!("fence {} × {} mm",project.fence.w,project.fence.h),egui::FontId::proportional(11.0),C_FENCE);
    }else{
      painter.extend(egui::Shape::dashed_line(&[fr.left_top(),fr.right_top(),fr.right_bottom(),fr.left_bottom(),fr.left_top()],Stroke::new(1.0_f32,Color32::from_gray(90)),6.0,5.0));
    }
    // ---- toolpath + simulation (plan starts at the origin marker) ----
    let sim_plan=sim.active&&sim.path.is_none();
    let mut prev=plan.origin;
    let head=self.to_screen(rect,sim.head.0,sim.head.1);
    let head_col=if sim.laser{Color32::from_rgb(255,60,50)}else{Color32::YELLOW};
    for (i,m) in plan.moves.iter().enumerate(){
      let (a,b)=(self.to_screen(rect,prev.0,prev.1),self.to_screen(rect,m.x,m.y));
      let base=if m.laser{Color32::from_rgb(105,105,105)}else if !m.rapid{Color32::from_rgb(120,96,40)}else{Color32::from_rgb(60,66,78)};
      if sim_plan&&i<sim.index{painter.line_segment([a,b],Stroke::new(if m.laser{1.7_f32}else{1.0},Color32::from_rgb(77,211,128)));}
      else if sim_plan&&i==sim.index{painter.line_segment([a,b],Stroke::new(1.2_f32,base));painter.line_segment([a,head],Stroke::new(2.0_f32,head_col));}
      else{painter.line_segment([a,b],Stroke::new(if m.laser{1.0_f32}else{0.6},base));}
      prev=(m.x,m.y);
    }
    // ---- live run path (Job or Trace) ----
    if let (true,Some(path))=(sim.active,&sim.path){
      let mut prev=sim.start;
      for (i,m) in path.iter().enumerate(){
        let (a,b)=(self.to_screen(rect,prev.0,prev.1),self.to_screen(rect,m.x,m.y));
        if i<sim.index{painter.line_segment([a,b],Stroke::new(if m.laser{1.9_f32}else{1.0},Color32::from_rgb(77,211,128)));}
        else if i==sim.index{painter.line_segment([a,head],Stroke::new(2.0_f32,head_col));}
        prev=(m.x,m.y);
      }
    }
    if sim.active&&(!plan.moves.is_empty()||sim.path.is_some()){
      painter.line_segment([head+Vec2::new(-8.0,-8.0),head+Vec2::new(8.0,8.0)],Stroke::new(2.2_f32,head_col));
      painter.line_segment([head+Vec2::new(-8.0,8.0),head+Vec2::new(8.0,-8.0)],Stroke::new(2.2_f32,head_col));
    }
    // ---- shapes ----
    let hovered=if matches!(self.drag,Drag::None){world.and_then(|w|self.hit(project,w))}else{None};
    let moving:Vec<usize>=if let Drag::Move{originals,..}=&self.drag{originals.iter().map(|o|o.0).collect()}else{vec![]};
    let mut order=0usize;
    for (i,s) in project.shapes.iter().enumerate(){
      if s.layer==Layer::Burn{order+=1;}
      if !self.layer_visible(s.layer){continue;}
      let base=match s.layer{Layer::Burn=>C_SHAPE,Layer::Notes=>C_NOTE,Layer::Guides=>C_GUIDE};
      let color=if self.refused.contains(&i){C_REFUSED}else if selected.contains(&i){C_SELECT}else if hovered==Some(i){C_HOVER}else{base};
      self.paint_shape(&painter,rect,s,0.0,0.0,Stroke::new(if selected.contains(&i)||self.refused.contains(&i){2.0_f32}else{1.5},color));
      // Burned extent of selected traces / bordered shapes.
      if selected.contains(&i)&&!moving.contains(&i){
        for r in paths::display_rings(s){self.path_shape(&painter,rect,&r,true,0.0,0.0,Stroke::new(1.0_f32,C_SELECT.gamma_multiply(0.6)),true);}
      }
      if self.show_order&&s.layer==Layer::Burn{
        let (x0,_,_,y1)=s.extent();
        let q=self.to_screen(rect,x0,y1);
        let r=Rect::from_min_size(q+Vec2::new(-2.0,-17.0),Vec2::new(8.0+7.0*order.to_string().len() as f32,15.0));
        painter.rect_filled(r,3.0,Color32::from_rgba_unmultiplied(20,20,24,220));
        painter.text(r.center(),egui::Align2::CENTER_CENTER,order.to_string(),egui::FontId::proportional(11.0),C_ORDER);
      }
    }
    // ---- Move tool preview (click-click) ----
    if let (Some(base),Some(c))=(self.move_base,snapped){
      let (dx,dy)=(c.0-base.0,c.1-base.1);
      for &i in selected.iter(){if let Some(s)=project.shapes.get(i){self.paint_shape(&painter,rect,s,dx,dy,Stroke::new(1.2_f32,C_PREVIEW));}}
      painter.line_segment([self.to_screen(rect,base.0,base.1),self.to_screen(rect,c.0,c.1)],Stroke::new(1.0_f32,C_PREVIEW));
    }
    // ---- job origin marker ----
    let o=self.to_screen(rect,project.origin.0,project.origin.1);
    painter.line_segment([o-Vec2::new(14.0,0.0),o+Vec2::new(14.0,0.0)],Stroke::new(1.6_f32,C_ORIGIN));
    painter.line_segment([o-Vec2::new(0.0,14.0),o+Vec2::new(0.0,14.0)],Stroke::new(1.6_f32,C_ORIGIN));
    painter.circle_stroke(o,6.0,Stroke::new(1.6_f32,C_ORIGIN));
    painter.text(o+Vec2::new(9.0,9.0),egui::Align2::LEFT_TOP,"0,0 job",egui::FontId::proportional(11.0),C_ORIGIN);
    // ---- fence resize handle (Select tool) ----
    if self.tool==Tool::Select{
      let q=fr.right_top();let r=Rect::from_center_size(q,Vec2::splat(HANDLE_PX*2.0));
      let col=if project.fence.on{C_FENCE}else{Color32::from_gray(120)};
      painter.rect_filled(r,1.0,Color32::from_rgb(16,22,30));painter.rect_stroke(r,1.0,Stroke::new(1.2_f32,col),egui::StrokeKind::Inside);
    }
    // ---- handles ----
    if self.tool==Tool::Select&&selected.len()==1{if let Some(s)=project.shapes.get(selected[0]){
      let hot=pointer.and_then(|p|self.handle_at(rect,s,p));
      for (k,h) in Self::handles(s).iter().enumerate(){
        let q=self.to_screen(rect,h.0,h.1);let r=Rect::from_center_size(q,Vec2::splat(HANDLE_PX*2.0));
        painter.rect_filled(r,1.0,if hot==Some(k){C_SELECT}else{Color32::from_rgb(16,22,30)});
        painter.rect_stroke(r,1.0,Stroke::new(1.2_f32,C_SELECT),egui::StrokeKind::Inside);
      }
    }}
    // ---- drawing preview with live dimensions ----
    if let (Some(a),Some(b))=(self.draw_start,snapped){
      let (dx,dy)=(b.0-a.0,b.1-a.1);
      let (ghost,label)=match self.tool{
        Tool::Line=>(Shape::line(a,b),format!("L {:.2} mm   A {:.1}°",(dx*dx+dy*dy).sqrt(),dy.atan2(dx).to_degrees())),
        Tool::Circle=>{let r=(dx*dx+dy*dy).sqrt();
          painter.line_segment([self.to_screen(rect,a.0,a.1),self.to_screen(rect,b.0,b.1)],Stroke::new(1.0_f32,C_PREVIEW.gamma_multiply(0.6)));
          (Shape::circle(a,r),format!("Ø {:.2} mm",2.0*r))}
        _=>(Shape::rectangle(a.0.min(b.0),a.1.min(b.1),dx.abs(),dy.abs()),format!("{:.2} × {:.2} mm",dx.abs(),dy.abs())),
      };
      self.paint_shape(&painter,rect,&ghost,0.0,0.0,Stroke::new(1.4_f32,C_PREVIEW));
      let q=self.to_screen(rect,b.0,b.1);
      painter.text(q+Vec2::new(12.0,-12.0),egui::Align2::LEFT_BOTTOM,label,egui::FontId::proportional(12.0),C_PREVIEW);
    }
    // ---- polyline in progress ----
    if !self.poly_pts.is_empty(){
      let mut pts=self.poly_pts.clone();
      if let Some(c)=snapped{pts.push(c);}
      self.path_shape(&painter,rect,&pts,false,0.0,0.0,Stroke::new(1.4_f32,C_PREVIEW),false);
      let f=self.to_screen(rect,self.poly_pts[0].0,self.poly_pts[0].1);
      let closing=self.poly_pts.len()>=3&&pointer.is_some_and(|p|self.near(rect,self.poly_pts[0],p,HANDLE_PX+3.0));
      painter.circle_stroke(f,if closing{7.0}else{4.0},Stroke::new(1.4_f32,C_PREVIEW));
      if let (Some(&l),Some(c))=(self.poly_pts.last(),snapped){
        let (dx,dy)=(c.0-l.0,c.1-l.1);
        painter.text(self.to_screen(rect,c.0,c.1)+Vec2::new(12.0,-12.0),egui::Align2::LEFT_BOTTOM,
          format!("L {:.2} mm   A {:.1}°{}",(dx*dx+dy*dy).sqrt(),dy.atan2(dx).to_degrees(),if closing{"   close"}else{""}),egui::FontId::proportional(12.0),C_PREVIEW);
      }
    }
    // ---- leader in progress ----
    if let (Tool::Leader,Some(i),Some(c))=(self.tool,self.leader_for,snapped){if let Some(s)=project.shapes.get(i){
      let mut t=s.clone();t.leader=Some(c);
      let a=Self::leader_anchor(&t);
      painter.line_segment([self.to_screen(rect,a.0,a.1),self.to_screen(rect,c.0,c.1)],Stroke::new(1.2_f32,C_PREVIEW));
    }}
    // ---- box select ----
    if let (Drag::Box{start},Some(end))=(&self.drag,pointer){
      let r=Rect::from_two_pos(*start,end);
      if end.x>=start.x{
        painter.rect_filled(r,0.0,Color32::from_rgba_unmultiplied(60,130,220,28));
        painter.rect_stroke(r,0.0,Stroke::new(1.0_f32,Color32::from_rgb(90,160,240)),egui::StrokeKind::Inside);
      }else{
        painter.rect_filled(r,0.0,Color32::from_rgba_unmultiplied(70,200,110,24));
        painter.extend(egui::Shape::dashed_line(&[r.left_top(),r.right_top(),r.right_bottom(),r.left_bottom(),r.left_top()],Stroke::new(1.0_f32,Color32::from_rgb(100,210,130)),5.0,4.0));
      }
    }
    // ---- snapped cursor marker ----
    if matches!(self.tool,Tool::Rectangle|Tool::Line|Tool::Polyline|Tool::Circle|Tool::Move|Tool::Text|Tool::Leader)&&resp.hovered(){if let Some(c)=snapped{
      let q=self.to_screen(rect,c.0,c.1);
      painter.line_segment([q-Vec2::new(7.0,0.0),q+Vec2::new(7.0,0.0)],Stroke::new(1.0_f32,C_PREVIEW));
      painter.line_segment([q-Vec2::new(0.0,7.0),q+Vec2::new(0.0,7.0)],Stroke::new(1.0_f32,C_PREVIEW));
    }}

    // ---- run countdown (top-right corner) ----
    if let Some((elapsed,total,paused))=sim.timer{
      let (w,h)=(300.0_f32,124.0_f32);
      let r=Rect::from_min_size(Pos2::new(rect.right()-w-16.0,rect.top()+16.0),Vec2::new(w,h));
      let accent=if paused{C_ORDER}else{Color32::WHITE};
      painter.rect_filled(r,8.0,Color32::from_rgba_unmultiplied(12,14,18,210));
      painter.rect_stroke(r,8.0,Stroke::new(1.0_f32,if paused{C_ORDER}else{Color32::from_gray(70)}),egui::StrokeKind::Inside);
      painter.text(r.left_top()+Vec2::new(14.0,10.0),egui::Align2::LEFT_TOP,if paused{"PAUSED · time remaining"}else{"Time remaining"},egui::FontId::proportional(13.0),if paused{C_ORDER}else{Color32::from_gray(170)});
      painter.text(Pos2::new(r.center().x,r.top()+60.0),egui::Align2::CENTER_CENTER,hms(total-elapsed),egui::FontId::monospace(46.0),accent);
      let bar=Rect::from_min_max(Pos2::new(r.left()+14.0,r.bottom()-34.0),Pos2::new(r.right()-14.0,r.bottom()-28.0));
      let frac=if total>0.0{(elapsed/total).clamp(0.0,1.0)}else{1.0};
      painter.rect_filled(bar,3.0,Color32::from_gray(45));
      painter.rect_filled(Rect::from_min_max(bar.min,Pos2::new(bar.left()+bar.width()*frac,bar.bottom())),3.0,Color32::from_rgb(77,211,128));
      painter.text(Pos2::new(r.left()+14.0,r.bottom()-10.0),egui::Align2::LEFT_BOTTOM,format!("elapsed {}",hms(elapsed)),egui::FontId::proportional(12.0),Color32::from_gray(160));
      painter.text(Pos2::new(r.right()-14.0,r.bottom()-10.0),egui::Align2::RIGHT_BOTTOM,format!("total {}  ·  {:.0} %",hms(total),frac*100.0),egui::FontId::proportional(12.0),Color32::from_gray(160));
    }

    // ================================================ status bar
    let hint=match self.tool{
      Tool::Select=>"Click: select · Shift+click: add/remove · Drag empty: box select (rightward = window, leftward = crossing) · Drag shape: move · Drag handle: resize",
      Tool::Pan=>"Drag to pan",
      Tool::Move=>if selected.is_empty(){"Click a shape to select it"}else if self.move_base.is_none(){"Click base point"}else{"Click destination"},
      Tool::Origin=>"Click to place the job origin (job 0,0). The program runs relative to it.",
      Tool::Rectangle=>if self.draw_start.is_none(){"Click first corner"}else{"Click opposite corner"},
      Tool::Line=>if self.draw_start.is_none(){"Click start point"}else{"Click end point"},
      Tool::Polyline=>if self.poly_pts.is_empty(){"Click the first vertex"}else{"Click next vertex · click the first vertex to close · double-click or Enter: finish open · Backspace: remove last · Esc: cancel"},
      Tool::Circle=>if self.draw_start.is_none(){"Click centre"}else{"Click radius point"},
      Tool::Text=>"Click to place the text (lower-left corner)",
      Tool::Leader=>"Click the point the leader points to (a shape under the click attaches the note) · Esc: cancel",
    };
    ui.horizontal(|ui|{
      ui.spacing_mut().item_spacing.x=10.0;
      let (x,y)=snapped.unwrap_or((0.0,0.0));
      ui.monospace(if world.is_some(){format!("X {:>9.3}  Y {:>9.3} mm",x,y)}else{"X       ---  Y       --- mm".to_string()});
      ui.separator();
      ui.weak(format!("Sel {}",selected.len()));
      ui.weak(format!("{:.2} px/mm",self.zoom));
      ui.weak(format!("Grid {} mm",self.grid_spacing));
      ui.weak(format!("Snap {}",if self.snap{"ON"}else{"OFF"}));
      ui.weak(format!("Layer {}",self.active_layer.name()));
      ui.separator();
      ui.colored_label(C_ORIGIN,format!("Origin X {:.3} Y {:.3}",project.origin.0,project.origin.1));
      ui.weak(if project.fence.on{format!("Fence ON {} × {}",project.fence.w,project.fence.h)}else{"Fence OFF".to_string()});
      ui.separator();
      ui.strong(format!("{:?}",self.tool));
      ui.add(egui::Label::new(egui::RichText::new(hint).weak()).truncate());
    });
    ev
  }
}
// =============================================================================
// End of file: canvas.rs
// Location:    laser-cnc-controller/src/canvas.rs
// Created:     2026-10-17 18:43:10 UTC
// =============================================================================
