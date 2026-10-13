// =============================================================================
// File:     canvas.rs
// Location: laser-cnc-controller/src/canvas.rs
// Version:  v0.1.5
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
// =============================================================================
use eframe::egui::{self,Color32,CursorIcon,Key,PointerButton,Pos2,Rect,Sense,Stroke,Vec2};
use crate::geometry::{Operation,Project,Shape,ShapeKind};
use crate::gcode::{Move,Plan};
use crate::widgets::{self,Icon};

#[derive(Clone,Copy,PartialEq,Debug)]
pub enum Tool{Select,Pan,Move,Origin,Rectangle,Line,Circle}

pub const GRID_SPACINGS:[f32;5]=[0.1,0.5,1.0,5.0,10.0];
const ZOOM_MIN:f32=0.2;
const ZOOM_MAX:f32=200.0;
const HANDLE_PX:f32=4.5;
const PICK_PX:f32=6.0;

const C_SHAPE:Color32=Color32::from_rgb(175,178,186);
const C_HOVER:Color32=Color32::from_rgb(225,228,235);
const C_SELECT:Color32=Color32::from_rgb(90,170,255);
const C_PREVIEW:Color32=Color32::from_rgb(255,196,64);
const C_ORIGIN:Color32=Color32::from_rgb(80,220,230);
const C_FENCE:Color32=Color32::from_rgb(240,150,40);

enum Drag{
  None,
  Box{start:Pos2},
  Move{start:(f32,f32),originals:Vec<(usize,Shape)>},
  Handle{index:usize,handle:usize,original:Shape},
  Pan,
  Zoom{anchor:Pos2},
  Origin{original:(f32,f32)},
  FenceCorner{original:(f32,f32)},
}

/// Simulation / live-run overlay passed in from the app.
/// `path` None: progress applies to the plan's moves (simulation).
/// `path` Some: a live run's own path (canvas coordinates) starting at `start`.
pub struct SimOverlay<'a>{pub active:bool,pub index:usize,pub head:(f32,f32),pub start:(f32,f32),pub path:Option<&'a [Move]>,pub laser:bool}

pub struct CanvasView{
  pub tool:Tool, pub grid:bool, pub grid_spacing:f32, pub snap:bool, pub zoom:f32,
  pan:Vec2, fit_pending:bool, last_rect:Rect,
  draw_start:Option<(f32,f32)>, move_base:Option<(f32,f32)>, drag:Drag,
}
impl Default for CanvasView{
  fn default()->Self{Self{tool:Tool::Select,grid:true,grid_spacing:5.0,snap:true,zoom:2.0,pan:Vec2::new(34.0,-28.0),
    fit_pending:true,last_rect:Rect::NOTHING,draw_start:None,move_base:None,drag:Drag::None}}
}

impl CanvasView{
  pub fn request_fit(&mut self){self.fit_pending=true;}

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
  fn fit(&mut self,rect:Rect,project:&Project){
    let m=40.0;
    let (wx,wy)=(project.work_x.max(1.0),project.work_y.max(1.0));
    self.zoom=((rect.width()-2.0*m)/wx).min((rect.height()-2.0*m)/wy).clamp(ZOOM_MIN,ZOOM_MAX);
    self.pan=Vec2::new((rect.width()-wx*self.zoom)/2.0,-(rect.height()-wy*self.zoom)/2.0);
  }
  fn set_tool(&mut self,tool:Tool,project:&mut Project){self.cancel(project);self.tool=tool;}
  /// Abort any operation in progress; restore shapes changed by an unfinished drag.
  fn cancel(&mut self,project:&mut Project){
    match std::mem::replace(&mut self.drag,Drag::None){
      Drag::Move{originals,..}=>{for (i,o) in originals{if let Some(s)=project.shapes.get_mut(i){*s=o;}}}
      Drag::Handle{index,original,..}=>{if let Some(s)=project.shapes.get_mut(index){*s=original;}}
      Drag::Origin{original}=>project.origin=original,
      Drag::FenceCorner{original}=>{project.fence.w=original.0;project.fence.h=original.1;}
      _=>{}
    }
    self.draw_start=None;self.move_base=None;
  }
  fn busy(&self)->bool{self.draw_start.is_some()||self.move_base.is_some()||!matches!(self.drag,Drag::None)}

  // ---------------------------------------------------------------- geometry helpers
  fn hit(&self,project:&Project,w:(f32,f32))->Option<usize>{
    let tol=PICK_PX/self.zoom;
    project.shapes.iter().enumerate().rev().find(|(_,s)|{
      let (x0,y0,x1,y1)=s.extent();
      match s.kind{
        ShapeKind::Rectangle=>w.0>=x0-tol&&w.0<=x1+tol&&w.1>=y0-tol&&w.1<=y1+tol,
        ShapeKind::Circle=>{let (cx,cy,rx,ry)=((x0+x1)/2.0,(y0+y1)/2.0,(x1-x0)/2.0+tol,(y1-y0)/2.0+tol);
          ((w.0-cx)/rx).powi(2)+((w.1-cy)/ry).powi(2)<=1.0}
        ShapeKind::Line=>{let (ax,ay,bx,by)=(s.x,s.y,s.x+s.w,s.y+s.h);let (dx,dy)=(bx-ax,by-ay);
          let l2=dx*dx+dy*dy;let t=if l2>0.0{(((w.0-ax)*dx+(w.1-ay)*dy)/l2).clamp(0.0,1.0)}else{0.0};
          let (px,py)=(ax+t*dx,ay+t*dy);((w.0-px).powi(2)+(w.1-py).powi(2)).sqrt()<=tol}
      }
    }).map(|(i,_)|i)
  }
  fn near(&self,rect:Rect,w:(f32,f32),p:Pos2,px:f32)->bool{let q=self.to_screen(rect,w.0,w.1);(q.x-p.x).abs()<=px&&(q.y-p.y).abs()<=px}
  fn fence_corner(project:&Project)->(f32,f32){(project.origin.0+project.fence.w,project.origin.1+project.fence.h)}
  fn handles(s:&Shape)->Vec<(f32,f32)>{
    let (x0,y0,x1,y1)=s.extent();let (xm,ym)=((x0+x1)/2.0,(y0+y1)/2.0);
    match s.kind{
      ShapeKind::Line=>vec![(s.x,s.y),(s.x+s.w,s.y+s.h)],
      ShapeKind::Rectangle=>vec![(x0,y0),(xm,y0),(x1,y0),(x1,ym),(x1,y1),(xm,y1),(x0,y1),(x0,ym)],
      ShapeKind::Circle=>vec![(xm,y0),(x1,ym),(xm,y1),(x0,ym)],
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
    }
    s
  }
  fn paint_shape(&self,painter:&egui::Painter,rect:Rect,s:&Shape,dx:f32,dy:f32,stroke:Stroke){
    let (x0,y0,x1,y1)=s.extent();
    match s.kind{
      ShapeKind::Rectangle=>{painter.rect_stroke(Rect::from_two_pos(self.to_screen(rect,x0+dx,y0+dy),self.to_screen(rect,x1+dx,y1+dy)),0.0,stroke,egui::StrokeKind::Middle);}
      ShapeKind::Line=>{painter.line_segment([self.to_screen(rect,s.x+dx,s.y+dy),self.to_screen(rect,s.x+s.w+dx,s.y+s.h+dy)],stroke);}
      ShapeKind::Circle=>{
        let (cx,cy,rx,ry)=((x0+x1)/2.0+dx,(y0+y1)/2.0+dy,(x1-x0)/2.0,(y1-y0)/2.0);
        let pts:Vec<Pos2>=(0..96).map(|i|{let t=std::f32::consts::TAU*i as f32/96.0;self.to_screen(rect,cx+rx*t.cos(),cy+ry*t.sin())}).collect();
        painter.add(egui::Shape::closed_line(pts,stroke));
      }
    }
  }
  /// Create a shape from two clicked points with the active drawing tool.
  fn finish_draw(&mut self,project:&mut Project,selected:&mut Vec<usize>,end:(f32,f32))->bool{
    let Some(start)=self.draw_start.take() else {return false;};
    let (dx,dy)=(end.0-start.0,end.1-start.1);
    let shape=match self.tool{
      Tool::Rectangle=>{
        if dx.abs()<1e-4||dy.abs()<1e-4{return false;}
        Shape::rectangle(start.0.min(end.0),start.1.min(end.1),dx.abs(),dy.abs())
      }
      Tool::Line=>{
        if (dx*dx+dy*dy).sqrt()<1e-4{return false;}
        let mut s=Shape::rectangle(start.0,start.1,dx,dy);s.kind=ShapeKind::Line;s.operation=Operation::Outline;s
      }
      Tool::Circle=>{
        let r=(dx*dx+dy*dy).sqrt();if r<1e-4{return false;}
        let mut s=Shape::rectangle(start.0-r,start.1-r,2.0*r,2.0*r);s.kind=ShapeKind::Circle;s
      }
      _=>return false,
    };
    project.shapes.push(shape);
    *selected=vec![project.shapes.len()-1];
    true
  }

  // ---------------------------------------------------------------- tool strip
  /// Canvas tool strip. Returns true when geometry changed or G-code regeneration was requested.
  pub fn toolbar(&mut self,ui:&mut egui::Ui,project:&mut Project,selected:&mut Vec<usize>)->bool{
    let mut changed=false;
    ui.horizontal_wrapped(|ui|{
      ui.spacing_mut().item_spacing=Vec2::new(4.0,4.0);
      for (tool,icon,tip) in [(Tool::Select,Icon::Select,"Select"),(Tool::Pan,Icon::Pan,"Pan"),(Tool::Move,Icon::Move,"Move (click base point, click destination)"),
                              (Tool::Origin,Icon::Origin,"Job origin: click to place job 0,0 (the spot you zero on the machine)"),
                              (Tool::Rectangle,Icon::Rectangle,"Rectangle (click corner, click opposite corner)"),
                              (Tool::Line,Icon::Line,"Line (click start, click end)"),(Tool::Circle,Icon::Circle,"Circle (click centre, click radius)")]{
        if tool==Tool::Rectangle{ui.separator();}
        if widgets::icon_button(ui,icon,self.tool==tool,true,tip).clicked(){self.set_tool(tool,project);}
      }
      ui.separator();
      let has_sel=!selected.is_empty();
      if widgets::icon_button(ui,Icon::Duplicate,false,has_sel,"Duplicate selected").clicked(){
        let off=if self.snap{self.grid_spacing.max(1.0)}else{2.0};
        let mut new_sel=vec![];
        for &i in selected.iter(){if let Some(s)=project.shapes.get(i).cloned(){let mut d=s;d.x+=off;d.y+=off;project.shapes.push(d);new_sel.push(project.shapes.len()-1);}}
        *selected=new_sel;changed=true;
      }
      if widgets::icon_button(ui,Icon::Delete,false,has_sel,"Delete selected (Delete)").clicked(){
        self.cancel(project);
        let mut idx=selected.clone();idx.sort_unstable();idx.dedup();
        for i in idx.into_iter().rev(){if i<project.shapes.len(){project.shapes.remove(i);}}
        selected.clear();changed=true;
      }
      if widgets::icon_button(ui,Icon::Clear,false,!project.shapes.is_empty(),"Clear all shapes").clicked(){
        self.cancel(project);project.shapes.clear();selected.clear();changed=true;
      }
      ui.separator();
      if widgets::icon_button(ui,Icon::Grid,self.grid,true,"Grid on/off").clicked(){self.grid=!self.grid;}
      egui::ComboBox::from_id_salt("grid_spacing").width(78.0).selected_text(format!("{} mm",self.grid_spacing))
        .show_ui(ui,|ui|{for s in GRID_SPACINGS{ui.selectable_value(&mut self.grid_spacing,s,format!("{s} mm"));}})
        .response.on_hover_text("Grid spacing (also the snap increment)");
      if widgets::icon_button(ui,Icon::Snap,self.snap,true,"Snap to grid on/off").clicked(){self.snap=!self.snap;}
      ui.separator();
      ui.label("Zoom");
      let mut z=self.zoom;
      if widgets::num_f32_mul(ui,"canvas_zoom",&mut z,1.25,ZOOM_MIN,ZOOM_MAX,2,"px/mm"){
        if self.last_rect.is_positive(){let c=self.last_rect.center();self.zoom_about(self.last_rect,c,z);}else{self.zoom=z;}
      }
      if widgets::icon_button(ui,Icon::Fit,false,true,"Fit work area").clicked(){self.fit_pending=true;}
      ui.separator();
      // Work fence: on/off and size (from the origin marker).
      if widgets::icon_button(ui,Icon::Fence,project.fence.on,true,"Work fence on/off: program moves checked against it, jogs outside it refused once zero is set").clicked(){
        project.fence.on=!project.fence.on;changed=true;
      }
      ui.label("W");
      changed|=widgets::num_f32(ui,"fence_w",&mut project.fence.w,self.grid_spacing.max(1.0),1.0,3000.0,1,"");
      ui.label("H");
      changed|=widgets::num_f32(ui,"fence_h",&mut project.fence.h,self.grid_spacing.max(1.0),1.0,3000.0,1,"mm");
    });
    changed
  }

  // ---------------------------------------------------------------- canvas
  /// Draws the canvas and handles interaction. Returns true when geometry changed.
  pub fn show(&mut self,ui:&mut egui::Ui,project:&mut Project,selected:&mut Vec<usize>,plan:&Plan,sim:&SimOverlay<'_>)->bool{
    let mut changed=false;
    selected.retain(|&i|i<project.shapes.len());
    let status_h=24.0;
    let avail=ui.available_size();
    let (resp,painter)=ui.allocate_painter(Vec2::new(avail.x.max(100.0),(avail.y-status_h).max(100.0)),Sense::click_and_drag());
    let rect=resp.rect;
    // Fit once the canvas size is stable (first frames can report a provisional window size).
    if self.fit_pending{self.fit(rect,project);if (rect.size()-self.last_rect.size()).length()<0.5{self.fit_pending=false;}}
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
        Tool::Origin=>{}
        Tool::Select|Tool::Move=>{
          let handle=if self.tool==Tool::Select&&selected.len()==1{
            project.shapes.get(selected[0]).and_then(|s|self.handle_at(rect,s,origin)).map(|h|(selected[0],h))}else{None};
          if let Some((index,handle))=handle{
            self.drag=Drag::Handle{index,handle,original:project.shapes[index].clone()};
          }else if let Some(i)=self.hit(project,w){
            if !selected.contains(&i){if !mods.shift{selected.clear();}selected.push(i);}
            self.move_base=None;
            self.drag=Drag::Move{start:self.snap_pt(w),originals:selected.iter().map(|&i|(i,project.shapes[i].clone())).collect()};
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
          for (i,o) in originals{if let Some(s)=project.shapes.get_mut(*i){s.x=o.x+dx;s.y=o.y+dy;}}
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
            let (x0,y0,x1,y1)=s.extent();
            let pick=if window{x0>=bx0&&x1<=bx1&&y0>=by0&&y1<=by1}else{x0<=bx1&&x1>=bx0&&y0<=by1&&y1>=by0};
            if pick&&!selected.contains(&i){selected.push(i);}
          }
        }}
        Drag::Move{..}|Drag::Handle{..}|Drag::Origin{..}|Drag::FenceCorner{..}=>changed=true,
        _=>{}
      }
      if resp.drag_stopped_by(PointerButton::Primary)&&matches!(self.tool,Tool::Rectangle|Tool::Line|Tool::Circle){
        if let Some(e)=snapped{changed|=self.finish_draw(project,selected,e);}
      }
      if resp.drag_stopped_by(PointerButton::Primary)&&self.tool==Tool::Origin{
        if let Some(e)=snapped{project.origin=e;changed=true;}
      }
    }

    // ------------------------------------------------ clicks
    if resp.clicked(){if let (Some(w),Some(s))=(world,snapped){
      match self.tool{
        Tool::Select=>{
          match self.hit(project,w){
            Some(i)=>{if mods.shift{if let Some(p)=selected.iter().position(|&j|j==i){selected.remove(p);}else{selected.push(i);}}else{*selected=vec![i];}}
            None=>{if !mods.shift{selected.clear();}}
          }
        }
        Tool::Pan=>{}
        Tool::Origin=>{project.origin=s;changed=true;}
        Tool::Move=>{
          if selected.is_empty(){if let Some(i)=self.hit(project,w){*selected=vec![i];}}
          else if let Some(base)=self.move_base.take(){
            let (dx,dy)=(s.0-base.0,s.1-base.1);
            for &i in selected.iter(){if let Some(sh)=project.shapes.get_mut(i){sh.x+=dx;sh.y+=dy;}}
            changed=true;
          }else{self.move_base=Some(s);}
        }
        Tool::Rectangle|Tool::Line|Tool::Circle=>{
          if self.draw_start.is_none(){self.draw_start=Some(s);}else{changed|=self.finish_draw(project,selected,s);}
        }
      }
    }}
    if resp.secondary_clicked(){self.cancel(project);}

    // ------------------------------------------------ keyboard (only when no text field has focus)
    if ui.ctx().memory(|m|m.focused().is_none()){
      if ui.input(|i|i.key_pressed(Key::Escape)){
        if self.busy(){self.cancel(project);}else if !selected.is_empty(){selected.clear();}else{self.tool=Tool::Select;}
      }
      if ui.input(|i|i.key_pressed(Key::Delete))&&!selected.is_empty(){
        self.cancel(project);
        let mut idx=selected.clone();idx.sort_unstable();idx.dedup();
        for i in idx.into_iter().rev(){if i<project.shapes.len(){project.shapes.remove(i);}}
        selected.clear();changed=true;
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
        (_,Tool::Rectangle|Tool::Line|Tool::Circle|Tool::Origin)=>CursorIcon::Crosshair,
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
      let base=if m.laser{Color32::from_rgb(105,105,105)}else{Color32::from_rgb(60,66,78)};
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
    for (i,s) in project.shapes.iter().enumerate(){
      let color=if selected.contains(&i){C_SELECT}else if hovered==Some(i){C_HOVER}else{C_SHAPE};
      self.paint_shape(&painter,rect,s,0.0,0.0,Stroke::new(if selected.contains(&i){2.0_f32}else{1.5},color));
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
      let mut ghost=Shape::rectangle(a.0,a.1,dx,dy);
      let label=match self.tool{
        Tool::Line=>{ghost.kind=ShapeKind::Line;format!("L {:.2} mm   A {:.1}°",(dx*dx+dy*dy).sqrt(),dy.atan2(dx).to_degrees())}
        Tool::Circle=>{let r=(dx*dx+dy*dy).sqrt();ghost=Shape::rectangle(a.0-r,a.1-r,2.0*r,2.0*r);ghost.kind=ShapeKind::Circle;
          painter.line_segment([self.to_screen(rect,a.0,a.1),self.to_screen(rect,b.0,b.1)],Stroke::new(1.0_f32,C_PREVIEW.gamma_multiply(0.6)));
          format!("Ø {:.2} mm",2.0*r)}
        _=>format!("{:.2} × {:.2} mm",dx.abs(),dy.abs()),
      };
      self.paint_shape(&painter,rect,&ghost,0.0,0.0,Stroke::new(1.4_f32,C_PREVIEW));
      let q=self.to_screen(rect,b.0,b.1);
      painter.text(q+Vec2::new(12.0,-12.0),egui::Align2::LEFT_BOTTOM,label,egui::FontId::proportional(12.0),C_PREVIEW);
    }
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
    if matches!(self.tool,Tool::Rectangle|Tool::Line|Tool::Circle|Tool::Move)&&resp.hovered(){if let Some(c)=snapped{
      let q=self.to_screen(rect,c.0,c.1);
      painter.line_segment([q-Vec2::new(7.0,0.0),q+Vec2::new(7.0,0.0)],Stroke::new(1.0_f32,C_PREVIEW));
      painter.line_segment([q-Vec2::new(0.0,7.0),q+Vec2::new(0.0,7.0)],Stroke::new(1.0_f32,C_PREVIEW));
    }}

    // ================================================ status bar
    let hint=match self.tool{
      Tool::Select=>"Click: select · Shift+click: add/remove · Drag empty: box select (rightward = window, leftward = crossing) · Drag shape: move · Drag handle: resize",
      Tool::Pan=>"Drag to pan",
      Tool::Move=>if selected.is_empty(){"Click a shape to select it"}else if self.move_base.is_none(){"Click base point"}else{"Click destination"},
      Tool::Origin=>"Click to place the job origin (job 0,0). The program runs relative to it.",
      Tool::Rectangle=>if self.draw_start.is_none(){"Click first corner"}else{"Click opposite corner"},
      Tool::Line=>if self.draw_start.is_none(){"Click start point"}else{"Click end point"},
      Tool::Circle=>if self.draw_start.is_none(){"Click centre"}else{"Click radius point"},
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
      ui.separator();
      ui.colored_label(C_ORIGIN,format!("Origin X {:.3} Y {:.3}",project.origin.0,project.origin.1));
      ui.weak(if project.fence.on{format!("Fence ON {} × {}",project.fence.w,project.fence.h)}else{"Fence OFF".to_string()});
      ui.separator();
      ui.strong(format!("{:?}",self.tool));
      ui.add(egui::Label::new(egui::RichText::new(hint).weak()).truncate());
    });
    changed
  }
}
// =============================================================================
// End of file: canvas.rs
// Location:    laser-cnc-controller/src/canvas.rs
// Created:     2026-10-13 18:25:33 UTC
// =============================================================================
