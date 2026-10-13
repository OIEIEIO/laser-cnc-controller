// =============================================================================
// File:     app.rs
// Location: laser-cnc-controller/src/app.rs
// Version:  v0.1.9
// Comments: Application shell: theme, panels, machine-control row, project
//           management, run recording, simulation timing, G-code monitor.
//           - Fixed machine-control row under the File/Help line, in job order:
//             Spot ON/OFF [power], Trace [Part|Job|Fence] [feed] | Start,
//             Pause/Resume, Reset | Laser OFF   (v0.1.6 layout).
//           - Reset: feed hold (!) then soft reset (Ctrl-X).
//           - Pause/Resume is one button driven by the GRBL state word:
//             Run -> "Pause" sends '@' (safety door: hold position, laser off);
//             Door/Hold -> "Resume" sends '~' (GRBL restores laser, then moves).
//             A DOOR flag shows while the controller reports Door.
//           - Canvas, tools, grid/snap/zoom live in src/canvas.rs.
//           - Shape Properties: right column, top (v0.1.8).
//           - G-code / Stream: compact monospace, per-line state, follows the
//             active line, controller state, filtered RX/TX log.
//           - No sliders or drag-to-change fields (src/widgets.rs spin fields).
//           - Panels collapsible/floatable/resizable; layout persisted.
//           v0.1.4:
//           - Projects panel: saved-project dropdown with Open / Delete;
//             Active Project with New / Save / Save As / Close / Export G-code;
//             Job Details; Run History for the open project.
//           - File menu: New, Open, Save, Save As, Close, Delete, Export G-code.
//           - Save on a never-saved project asks for a name; Save As warns
//             before overwriting; Delete confirms. Save As copies get a new
//             project id (empty run history).
//           - Unsaved-changes marker (*) in the panel and window title, and a
//             Save / Discard / Cancel prompt on New, Open and Close.
//           - Every Job and Frame run is recorded (src/runlog.rs) and indexed in
//             redb; every command sent goes to the daily log.
//           - Runs panel (left column): all runs, details, editable notes,
//             open run folder, open exports/logs folders.
//           - Generate G-code button and stream legend removed.
//           v0.1.5:
//           - Serial transport moved to a worker thread (src/machine.rs);
//             the app drains its events once per frame (machine.poll()).
//           - Serial Communications: Connect / Cancel (while connecting) /
//             Disconnect; connection block with status, firmware banner,
//             $I build, port, state, WPos/MPos, laser word from $G, modes,
//             Unlock ($X, enabled in Alarm), $# offsets and $$ settings.
//           - G-code / Stream: manual command line under the RX/TX log
//             (Enter or Send; disabled while a job streams).
//           - Line errors no longer end a run: streaming continues (LaserGRBL
//             Buffered behaviour); the run ends as "Error" with the first
//             error and the count. ALARM ends the run as "Error".
//           - Fixed 1.15 pixels-per-point removed (OS scaling; egui
//             Ctrl+Plus / Ctrl+Minus zoom).
//           v0.1.6:
//           - Zero X / Zero Y send G92 X0 / G92 Y0 (GRBL here rejects G10 L20).
//             Go to zero sits on the same row.
//           - Jog step: 0.1 / 1 / 10 mm buttons plus a typed step field.
//           - Job origin marker and work fence (src/canvas.rs, src/gcode.rs):
//             the program runs relative to the marker.
//           - Jogs outside the work fence are refused (pop-up) while the fence
//             is ON and the axis has been zeroed this session. Base position:
//             reported WPos, or the last commanded jog target while moving.
//           - Spot ON / Spot OFF: one latching button (M3 S<spot> / M5).
//             Laser ON and its power field removed; Laser OFF stays (M5, also
//             clears Spot). Spot clears on Laser OFF, job/trace start, Reset,
//             disconnect, alarm or any controller reset (banner).
//           - Frame replaced by Trace with target Part (selection box) / Job
//             (all shapes box) / Fence, relative to the origin marker, at spot
//             power and trace feed. Run kind "Trace".
//           - Live run view: during a Job or Trace the canvas shows the run's
//             path with done / current segment and the head at origin + WPos
//             (smoothed). Progress follows the reported position along the
//             path (GRBL 0.9 does not report the executing line).
//           - Floating G-code / Stream window: background 80 % transparent.
//           - Work zero flags clear on any controller reset or disconnect.
//           v0.1.7:
//           - Simulation panel and offline simulation removed.
//           - Simulate toggle + speed (1/2/5/10/50x) in the machine-control row
//             after Laser OFF. Switchable only while disconnected. Connect then
//             attaches the virtual GRBL (src/vgrbl.rs): every button, the
//             stream view and the live run view work as on the machine.
//             SIMULATION badge in the control row; Serial panel shows
//             SIMULATOR. Speed changes apply during a run.
//           - Run records tagged SIM (RunRecord::simulated) in Runs, Run
//             History and the daily log.
//           - Head marker red while the laser is on, yellow while off: from
//             the virtual controller in Simulate, otherwise from the path
//             segment the head is on (burn or travel).
//           - Panels: 7 (layout key v4; floating flags reset once).
//           v0.1.8:
//           - Shape Properties docks at the top of the right column (above Jog
//             Controls) as a normal collapsible / floatable panel; it expands
//             when a new shape is selected. Tool-strip slot removed.
//           - Jog step buttons: 0.1 mm and 10 mm, plus the typed step field.
//           - Stream header counts clamp at zero (after Reset / Disconnect the
//             sent counter returned to 0 while OK did not, and Sent wrapped).
//           v0.1.9:
//           - Machine Profile (Work X / Y, Travel F, Max PWM S) is a user
//             setting saved with the app settings (eframe storage), restored
//             at startup and applied to every project (new, opened, closed).
//             Defaults 380 x 280 mm, 2500 mm/min, S1000. Profile values are
//             left out of the unsaved-changes comparison.
// =============================================================================
use eframe::egui::{self,Color32,CornerRadius,Stroke,Vec2};
use eframe::egui::collapsing_header::CollapsingState;
use serde::{Deserialize,Serialize};
use crate::geometry::{self,Project,ShapeKind,Fill,Operation};
use crate::gcode::{self,Move,Plan};
use crate::storage::Store;
use crate::machine::{Link,Machine};
use crate::canvas::{CanvasView,SimOverlay};
use crate::runlog::{self,ActiveRun,RunRecord};
use crate::widgets;
use std::time::Instant;

#[derive(Clone,Copy,Default,Serialize,Deserialize)]
struct Panel { floating:bool }

const TITLES:[&str;7]=["Serial Communications","Projects","Shape Properties","Machine Profile","Jog Controls","G-code / Stream","Runs"];
const LEFT_DOCK:[usize;4]=[0,1,3,6];
const RIGHT_DOCK:[usize;3]=[2,4,5];
const PROPS:usize=2;
const LAYOUT_KEY:&str="panel_layout_v4";
const PROFILE_KEY:&str="machine_profile_v1";

/// Machine Profile: a property of the machine, saved with the app settings.
#[derive(Clone,Copy,PartialEq,Serialize,Deserialize)]
#[serde(default)]
struct MachineProfile{work_x:f32,work_y:f32,travel_feed:f32,max_power:u32}
impl Default for MachineProfile{fn default()->Self{Self{work_x:380.0,work_y:280.0,travel_feed:2500.0,max_power:1000}}}
const SIM_SPEEDS:[f32;5]=[1.0,2.0,5.0,10.0,50.0];
const FENCE_EPS:f32=0.001;

/// Trace target in the machine-control row.
#[derive(Clone,Copy,PartialEq)]
enum TraceTarget{Part,Job,Fence}

/// Live view of a Job or Trace run: its path in canvas coordinates and progress along it.
struct LiveView{path:Vec<Move>,origin:(f32,f32),progress:usize,head:Option<(f32,f32)>}

/// Project action that may need an unsaved-changes prompt first.
#[derive(Clone,PartialEq)]
enum Pending{New,Open(String),Close}

/// Modal dialogs.
enum Dialog{
  None,
  SaveAs{name:String,copy:bool,then:Option<Pending>},
  ConfirmOverwrite{name:String,copy:bool,then:Option<Pending>},
  ConfirmDelete{name:String},
  Unsaved{then:Pending},
  Notice{title:String,msg:String},
}

pub struct LaserApp {
    project:Project, plan:Plan, store:Option<Store>, machine:Machine,
    view:CanvasView, selected:Vec<usize>, props_last_sel:Vec<usize>,
    panels:[Panel;7],
    last_frame:Instant,
    spot_power:u32,frame_feed:f32,spot_on:bool,trace_target:TraceTarget,monitor_follow:usize,
    max_power:u32, profile:MachineProfile, project_active:bool, saved_once:bool, saved_json:String, dirty:bool, last_title:String,
    dialog:Dialog, saved_names:Vec<String>, project_to_load:String, notice:String, job_lines:Vec<String>,
    runs:Vec<RunRecord>, selected_run:Option<String>, notes_edit:String, active_run:Option<ActiveRun>,
    manual_cmd:String,
    zero_x:bool, zero_y:bool, seen_resets:u32, live:Option<LiveView>, jog_pred:Option<(f32,f32)>, last_jog:Option<Instant>,
}

impl LaserApp {
 pub fn new(cc:&eframe::CreationContext<'_>)->Self {
    // ---- Theme: neutral dark panels, black canvas, single blue accent ----
    let mut visuals=egui::Visuals::dark();
    visuals.panel_fill=Color32::from_rgb(16,17,19);
    visuals.window_fill=Color32::from_rgb(22,23,26);
    visuals.extreme_bg_color=Color32::from_rgb(8,8,9);
    visuals.faint_bg_color=Color32::from_rgb(24,25,28);
    visuals.window_stroke=Stroke::new(1.0_f32,Color32::from_rgb(48,50,56));
    visuals.window_corner_radius=CornerRadius::same(6);
    visuals.menu_corner_radius=CornerRadius::same(6);
    visuals.window_shadow=egui::Shadow{offset:[0,4],blur:14,spread:0,color:Color32::from_black_alpha(110)};
    visuals.popup_shadow=egui::Shadow{offset:[0,3],blur:10,spread:0,color:Color32::from_black_alpha(100)};
    visuals.widgets.noninteractive.bg_fill=Color32::from_rgb(20,21,24);
    visuals.widgets.noninteractive.bg_stroke=Stroke::new(1.0_f32,Color32::from_rgb(40,42,47));
    visuals.widgets.inactive.bg_fill=Color32::from_rgb(38,40,45);
    visuals.widgets.inactive.weak_bg_fill=Color32::from_rgb(38,40,45);
    visuals.widgets.hovered.bg_fill=Color32::from_rgb(56,59,66);
    visuals.widgets.hovered.weak_bg_fill=Color32::from_rgb(56,59,66);
    visuals.widgets.active.bg_fill=Color32::from_rgb(70,74,82);
    visuals.widgets.active.weak_bg_fill=Color32::from_rgb(70,74,82);
    for w in [&mut visuals.widgets.noninteractive,&mut visuals.widgets.inactive,&mut visuals.widgets.hovered,&mut visuals.widgets.active,&mut visuals.widgets.open]{w.corner_radius=CornerRadius::same(4);}
    visuals.selection.bg_fill=Color32::from_rgb(38,92,150);
    visuals.selection.stroke=Stroke::new(1.0_f32,Color32::from_rgb(150,195,240));
    visuals.hyperlink_color=Color32::from_rgb(110,170,235);
    let mut style=(*cc.egui_ctx.style()).clone();
    style.visuals=visuals;
    style.spacing.item_spacing=Vec2::new(8.0,6.0);
    style.spacing.button_padding=Vec2::new(10.0,4.0);
    style.spacing.interact_size.y=24.0;
    style.spacing.window_margin=egui::Margin::same(10);
    cc.egui_ctx.set_style(style);

    // ---- Restore floating/docked layout from the last session ----
    let mut panels=[Panel::default();7];
    if let Some(storage)=cc.storage {if let Some(saved)=eframe::get_value::<[Panel;7]>(storage,LAYOUT_KEY){panels=saved;}}
    // ---- Machine Profile from the user settings (defaults 380 x 280) ----
    let profile=cc.storage.and_then(|s|eframe::get_value::<MachineProfile>(s,PROFILE_KEY)).unwrap_or_default();

    let (store,err)=match Store::open(){Ok(s)=>(Some(s),String::new()),Err(e)=>(None,format!("Database unavailable: {e}"))};
    let mut project=Project::blank();
    project.work_x=profile.work_x;project.work_y=profile.work_y;project.travel_feed=profile.travel_feed;
    let saved_json=Self::comparable_json(&project);
    let (mut names,mut runs)=(vec![],vec![]);
    if let Some(s)=&store {names=s.names().unwrap_or_default();runs=s.runs().unwrap_or_default();}
    let plan=gcode::generate(&project,profile.max_power);
    let mut machine=Machine::default();machine.refresh();
    Self {project,plan,store,machine,view:CanvasView::default(),selected:vec![],props_last_sel:vec![],
      panels,
      last_frame:Instant::now(),
      spot_power:2,frame_feed:1200.0,spot_on:false,trace_target:TraceTarget::Job,monitor_follow:usize::MAX,max_power:profile.max_power,profile,
      project_active:false,saved_once:false,saved_json,dirty:false,last_title:String::new(),
      dialog:Dialog::None,saved_names:names,project_to_load:String::new(),notice:err,job_lines:vec![],
      runs,selected_run:None,notes_edit:String::new(),active_run:None,manual_cmd:String::new(),
      zero_x:false,zero_y:false,seen_resets:0,live:None,jog_pred:None,last_jog:None}
 }
 /// Apply the Machine Profile to the open project, then regenerate the program.
 fn regenerate(&mut self){
    self.project.work_x=self.profile.work_x;self.project.work_y=self.profile.work_y;self.project.travel_feed=self.profile.travel_feed;
    self.max_power=self.profile.max_power;
    self.plan=gcode::generate(&self.project,self.max_power);
 }
 /// Project JSON without the Machine Profile fields (those belong to the machine, not the project).
 fn comparable_json(p:&Project)->String{
    let mut c=p.clone();c.work_x=0.0;c.work_y=0.0;c.travel_feed=0.0;
    serde_json::to_string(&c).unwrap_or_default()
 }
 fn duration_hms(seconds:f32)->String {let t=seconds.max(0.0).ceil() as u64; format!("{:02}:{:02}:{:02}",t/3600,(t/60)%60,t%60)}
 fn action(ui:&mut egui::Ui,label:&str,color:Color32)->egui::Response {
    ui.add(egui::Button::new(egui::RichText::new(label).color(Color32::WHITE).strong()).fill(color).min_size(Vec2::new(88.0,30.0)))
 }

 // ======================================================================== projects
 fn mark_clean(&mut self){self.saved_json=Self::comparable_json(&self.project);}
 fn refresh_names(&mut self){if let Some(s)=&self.store{self.saved_names=s.names().unwrap_or_default();}}
 /// Write the open project under its current name.
 fn save(&mut self){
    let Some(s)=&self.store else {self.notice="Database unavailable".into();return;};
    match s.save(&self.project).and_then(|_|s.remember(&self.project.name)) {
      Ok(())=>{self.project_active=true;self.saved_once=true;self.notice=format!("Saved '{}'",self.project.name);self.refresh_names();self.mark_clean();},
      Err(e)=>self.notice=format!("Save failed: {e}"),
    }
 }
 /// Save button: first save of a project asks for a name.
 fn request_save(&mut self){
    if !self.project_active {return;}
    if self.saved_once {self.save();} else {self.dialog=Dialog::SaveAs{name:self.project.name.clone(),copy:false,then:None};}
 }
 fn request_save_as(&mut self){
    if !self.project_active {return;}
    let name=if self.saved_once{format!("{} - Copy",self.project.name)}else{self.project.name.clone()};
    self.dialog=Dialog::SaveAs{name,copy:self.saved_once,then:None};
 }
 /// Name confirmed in the Save As dialog (overwrite already confirmed if needed).
 fn finish_save_as(&mut self,name:String,copy:bool,then:Option<Pending>){
    if copy {self.project.id=geometry::new_id();}
    self.project.name=name;
    self.save();
    if let Some(p)=then {self.perform(p);}
 }
 /// Route an action through the unsaved-changes prompt when needed.
 fn request(&mut self,p:Pending){if self.dirty {self.dialog=Dialog::Unsaved{then:p};} else {self.perform(p);}}
 fn perform(&mut self,p:Pending){
    match p {
      Pending::New=>{
        self.project=Project::blank();self.project_active=true;self.saved_once=false;
        self.selected.clear();self.view.request_fit();self.regenerate();self.mark_clean();self.notice="New project".into();
      }
      Pending::Close=>{
        self.project=Project::blank();self.project_active=false;self.saved_once=false;
        self.selected.clear();self.regenerate();self.mark_clean();self.notice="Project closed".into();
      }
      Pending::Open(name)=>{
        let Some(s)=&self.store else {self.notice="Database unavailable".into();return;};
        match s.load(&name){
          Ok(Some(mut p))=>{
            // Projects saved before v0.1.4 have no id: assign one now and store it.
            if p.id.is_empty(){p.id=geometry::new_id();let _=s.save(&p);}
            self.project=p;self.project_active=true;self.saved_once=true;
            self.selected.clear();self.view.request_fit();self.regenerate();self.mark_clean();
            self.notice=format!("Opened '{name}'");
          }
          Ok(None)=>self.notice=format!("'{name}' not found"),
          Err(e)=>self.notice=format!("Open failed: {e}"),
        }
      }
    }
 }
 fn delete_project(&mut self,name:&str){
    let Some(s)=&self.store else {return;};
    match s.delete(name){
      Ok(())=>{
        self.notice=format!("Deleted '{name}' (run records kept)");
        if self.project_active&&self.saved_once&&self.project.name==name {self.perform(Pending::Close);}
        if self.project_to_load==name {self.project_to_load.clear();}
        self.refresh_names();
      }
      Err(e)=>self.notice=format!("Delete failed: {e}"),
    }
 }
 fn export_gcode(&mut self){
    match runlog::export_gcode(&self.project.name,&self.plan.lines){
      Ok(p)=>self.notice=format!("Exported {}",p.display()),
      Err(e)=>self.notice=format!("Export failed: {e}"),
    }
 }
 /// Bring the Projects panel into view (docked and expanded).
 fn show_panel(&mut self,ctx:&egui::Context,id:usize){
    let mut state=CollapsingState::load_with_default_open(ctx,egui::Id::new(("panel",TITLES[id])),false);
    state.set_open(true);state.store(ctx);
 }

 // ======================================================================== runs
 fn refresh_runs(&mut self){if let Some(s)=&self.store{self.runs=s.runs().unwrap_or_default();}}
 /// Stream `lines` and open a run record for it. `path` (canvas coordinates) drives the live view.
 fn begin_run(&mut self,kind:&str,lines:Vec<String>,path:Vec<Move>){
    if self.active_run.is_some()||self.machine.running {self.notice="A run is already in progress".into();return;}
    self.job_lines=lines;
    self.machine.start(&self.job_lines);
    if !self.machine.running {return;}
    // Every program starts with M5.
    self.spot_on=false;
    self.live=Some(LiveView{path,origin:self.project.origin,progress:0,head:None});
    let record=RunRecord{kind:kind.into(),project_id:self.project.id.clone(),project_name:self.project.name.clone(),
      estimate_s:if kind=="Job"{self.plan.estimated_seconds}else{0.0},
      port:self.machine.port_label(),baud:self.machine.baud,firmware:self.machine.firmware.clone(),simulated:self.machine.simulate,
      app_version:env!("CARGO_PKG_VERSION").into(),max_power:self.max_power,spot_power:self.spot_power,frame_feed:self.frame_feed,
      ..Default::default()};
    match ActiveRun::begin(record,&self.job_lines,&self.project){
      Ok(run)=>{
        let tag=if run.record.simulated{"SIM "}else{""};
        let _=runlog::append_daily(&[format!("{} RUN START {tag}{} {} ({} lines) -> {}",runlog::clock(),kind,run.record.project_name,self.job_lines.len(),run.record.folder)]);
        if let Some(s)=&self.store{let _=s.save_run(&run.record);}
        self.active_run=Some(run);
      }
      Err(e)=>self.notice=format!("Run record could not be created: {e}"),
    }
 }
 /// Move machine journal entries to the daily log and the active run's serial.log.
 fn drain_journal(&mut self){
    if self.machine.journal.is_empty(){return;}
    let t=runlog::clock();
    let lines:Vec<String>=self.machine.journal.drain(..).map(|e|format!("{t} {e}")).collect();
    if let Some(run)=&mut self.active_run {run.serial.extend(lines.iter().cloned());}
    if let Err(e)=runlog::append_daily(&lines){self.notice=format!("Daily log write failed: {e}");}
 }
 /// Track state changes and decide when the active run has ended.
 fn track_run(&mut self){
    let Some(run)=&mut self.active_run else {return;};
    let state=self.machine.state.clone();
    if !state.is_empty()&&state!=run.last_state {run.event(&format!("controller {state}"));run.last_state=state.clone();}
    let total=self.job_lines.len();
    let outcome=if !self.machine.connected(){Some("Disconnected")}
      else if run.reset_requested {Some("Reset")}
      else if self.machine.alarm.is_some(){Some("Error")}
      else if !self.machine.running {
        // All lines answered; wait for the controller to report Idle (motion finished).
        let since=*run.finishing_since.get_or_insert_with(Instant::now);
        if (state.starts_with("Idle")&&since.elapsed().as_secs_f32()>0.5)||since.elapsed().as_secs()>600 {
          Some(if self.machine.job_errors>0{"Error"}else{"Completed"})
        }else{None}
      } else {None};
    let Some(outcome)=outcome else {return;};
    let run=self.active_run.take().unwrap();
    self.live=None;
    // Alarm text, or the first line error with the total count (streaming continued past line errors).
    let error=if outcome=="Error"{
      self.machine.alarm.clone().or_else(||self.machine.job_error.as_ref().map(|e|format!("{e} ({} line error(s))",self.machine.job_errors)))
    }else{None};
    let record=run.finish(outcome,error,self.machine.error_line,self.machine.acknowledged.min(total));
    let tag=if record.simulated{"SIM "}else{""};
    let _=runlog::append_daily(&[format!("{} RUN END {tag}{} {} — {} ({})",runlog::clock(),record.kind,record.project_name,record.outcome,Self::duration_hms(record.duration_s))]);
    if let Some(s)=&self.store{let _=s.save_run(&record);}
    self.notice=format!("{}{} run {}: {}",if record.simulated{"SIM "}else{""},record.kind,record.outcome.to_lowercase(),record.id);
    self.refresh_runs();
 }

 /// Trace rectangle (canvas coordinates) for the selected target.
 fn trace_rect(&self)->Result<(f32,f32,f32,f32),String>{
    let bbox=|idx:&mut dyn Iterator<Item=usize>|{
      let mut r=(f32::MAX,f32::MAX,f32::MIN,f32::MIN);
      for i in idx{if let Some(s)=self.project.shapes.get(i){let (a,b,c,d)=s.extent();r=(r.0.min(a),r.1.min(b),r.2.max(c),r.3.max(d));}}
      if r.0==f32::MAX{None}else{Some(r)}
    };
    match self.trace_target{
      TraceTarget::Part=>bbox(&mut self.selected.iter().copied()).ok_or_else(||"Select a part on the canvas to trace it".to_string()),
      TraceTarget::Job=>bbox(&mut (0..self.project.shapes.len())).ok_or_else(||"Nothing to trace: the canvas has no shapes".to_string()),
      TraceTarget::Fence=>{
        if !self.project.fence.on{return Err("Turn the work fence on to trace it".into());}
        let (ox,oy)=self.project.origin;Ok((ox,oy,ox+self.project.fence.w,oy+self.project.fence.h))
      }
    }
 }
 /// Trace program (relative to the origin marker) and its path in canvas coordinates.
 fn trace_program(&self)->Result<(Vec<String>,Vec<Move>),String>{
    let (x0,y0,x1,y1)=self.trace_rect()?;
    let corners=[(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)];
    let warnings=gcode::limit_warnings(&self.project,gcode::relative_span(&corners,self.project.origin),"Trace");
    if let Some(w)=warnings.first(){return Err(w.clone());}
    let (ox,oy)=self.project.origin;
    let c=|v:f32|gcode::coord(v);
    let mut lines=vec!["M5".to_string(),"G21".into(),"G90".into(),format!("G0 X{:.3} Y{:.3}",c(x0-ox),c(y0-oy)),format!("M3 S{}",self.spot_power)];
    let mut path=vec![Move{x:x0,y:y0,feed:self.project.travel_feed,power:0,laser:false}];
    for (i,(x,y)) in corners.iter().skip(1).enumerate(){
      lines.push(if i==0{format!("G1 X{:.3} Y{:.3} F{:.1}",c(x-ox),c(y-oy),self.frame_feed)}else{format!("G1 X{:.3} Y{:.3}",c(x-ox),c(y-oy))});
      path.push(Move{x:*x,y:*y,feed:self.frame_feed,power:self.spot_power,laser:true});
    }
    lines.push("M5".into());
    Ok((lines,path))
 }
 fn notice_dialog(&mut self,title:&str,msg:String){self.dialog=Dialog::Notice{title:title.into(),msg};}
 fn jog(&mut self,dx:f32,dy:f32){
    if !self.machine.connected(){self.notice="No controller connected".into();return;}
    if self.machine.running{self.notice="Jog is unavailable while a program streams".into();return;}
    // Base position: reported WPos once the controller is idle after the last jog; otherwise the last commanded target.
    let settled=self.machine.state.starts_with("Idle")&&self.last_jog.map_or(true,|t|t.elapsed().as_millis()>400);
    let wpos=self.machine.info.wpos.map(|w|(w.0,w.1));
    let base=if settled{wpos}else{self.jog_pred.or(wpos)};
    let f=self.project.fence;
    if f.on&&(self.zero_x||self.zero_y){
      let Some(b)=base else {
        self.notice_dialog("Jog refused","The controller is not reporting a work position (WPos), so the move cannot be checked against the work fence.\n\nTurn the work fence off to jog.".into());
        return;
      };
      let t=(b.0+dx,b.1+dy);
      let out_x=self.zero_x&&(t.0< -FENCE_EPS||t.0>f.w+FENCE_EPS);
      let out_y=self.zero_y&&(t.1< -FENCE_EPS||t.1>f.h+FENCE_EPS);
      if out_x||out_y{
        self.notice_dialog("Jog refused",format!("This move would take the head to X {:.3}  Y {:.3}, outside the work fence (0 – {:.1} × 0 – {:.1} mm from the job zero).\n\nEnlarge the work fence, turn it off, or jog the other way.",t.0,t.1,f.w,f.h));
        return;
      }
    }
    self.jog_pred=base.map(|b|(b.0+dx,b.1+dy));
    self.last_jog=Some(Instant::now());
    // GRBL 0.9 has no $J command: incremental G91 move, then G90.
    self.machine.jog(dx,dy);
 }
 /// Spot ON / Spot OFF latching button.
 fn toggle_spot(&mut self){
    if !self.machine.connected(){self.notice="No controller connected".into();return;}
    if self.machine.running{self.notice="Spot is unavailable while a program streams".into();return;}
    if self.spot_on{self.machine.command("M5");self.spot_on=false;}
    else{self.machine.command(&format!("M3 S{}",self.spot_power));self.spot_on=true;}
 }
 fn laser_off(&mut self){
    if !self.machine.connected(){self.notice="No controller connected".into();return;}
    self.machine.command("M5");
    if !self.machine.running{self.spot_on=false;}
 }
 /// Session state a manual command changes (work zero, spot).
 fn note_manual(&mut self,cmd:&str){
    let c=cmd.to_ascii_uppercase().replace(' ',"");
    if c.contains("G92")&&!c.contains("G92.1"){if c.contains('X'){self.zero_x=true;}if c.contains('Y'){self.zero_y=true;}}
    if c.contains("M5"){self.spot_on=false;}
    else if c.contains("M3")||c.contains("M4"){self.spot_on=true;}
 }
 /// Head position and progress along the live run's path.
 fn update_live(&mut self,dt:f32){
    let Some(lv)=&mut self.live else {return;};
    let Some(w)=self.machine.info.wpos else {return;};
    let target=(lv.origin.0+w.0,lv.origin.1+w.1);
    let a=1.0-(-dt*12.0).exp();
    lv.head=Some(match lv.head{None=>target,Some(h)=>(h.0+(target.0-h.0)*a,h.1+(target.1-h.1)*a)});
    // Nearest segment within a forward window; earliest one within 0.02 mm of the best (adjacent raster lines).
    let end=(lv.progress+300).min(lv.path.len());
    let mut dists=Vec::with_capacity(end.saturating_sub(lv.progress));
    for i in lv.progress..end{
      let p0=if i==0{lv.origin}else{(lv.path[i-1].x,lv.path[i-1].y)};
      dists.push(seg_dist(target,p0,(lv.path[i].x,lv.path[i].y)));
    }
    let best=dists.iter().cloned().fold(f32::MAX,f32::min);
    if best<1.0{if let Some(k)=dists.iter().position(|&d|d<=best+0.02){lv.progress+=k;}}
 }

 // ======================================================================== panels
 fn shape_properties(&mut self, ui:&mut egui::Ui){
     if self.selected.len()>1 {ui.weak(format!("{} shapes selected. Select one shape to edit.",self.selected.len()));return;}
     let step=self.view.grid_spacing;
     let max_power=self.max_power;
     if let Some(&i)=self.selected.first() {if let Some(s)=self.project.shapes.get_mut(i){
       let mut changed=false;
       ui.strong(format!("{:?} #{}",s.kind,i+1));
       egui::Grid::new("geom-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
         match s.kind {
           ShapeKind::Rectangle=>{
             // Normalise so X/Y is the lower-left corner and W/H are positive.
             let (x0,y0,x1,y1)=s.extent();s.x=x0;s.y=y0;s.w=x1-x0;s.h=y1-y0;
             ui.label("X");changed|=widgets::num_f32(ui,"p_x",&mut s.x,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Y");changed|=widgets::num_f32(ui,"p_y",&mut s.y,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Width");changed|=widgets::num_f32(ui,"p_w",&mut s.w,step,0.001,3000.0,3,"mm");ui.end_row();
             ui.label("Height");changed|=widgets::num_f32(ui,"p_h",&mut s.h,step,0.001,3000.0,3,"mm");ui.end_row();
           }
           ShapeKind::Line=>{
             let (mut ex,mut ey)=(s.x+s.w,s.y+s.h);
             ui.label("Start X");changed|=widgets::num_f32(ui,"p_sx",&mut s.x,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Start Y");changed|=widgets::num_f32(ui,"p_sy",&mut s.y,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("End X");if widgets::num_f32(ui,"p_ex",&mut ex,step,-3000.0,3000.0,3,"mm"){changed=true;}ui.end_row();
             ui.label("End Y");if widgets::num_f32(ui,"p_ey",&mut ey,step,-3000.0,3000.0,3,"mm"){changed=true;}ui.end_row();
             s.w=ex-s.x;s.h=ey-s.y;
             ui.label("Length");ui.weak(format!("{:.3} mm",(s.w*s.w+s.h*s.h).sqrt()));ui.end_row();
           }
           ShapeKind::Circle=>{
             let (x0,y0,x1,y1)=s.extent();
             let (mut cx,mut cy,mut d)=((x0+x1)/2.0,(y0+y1)/2.0,x1-x0);
             ui.label("Centre X");changed|=widgets::num_f32(ui,"p_cx",&mut cx,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Centre Y");changed|=widgets::num_f32(ui,"p_cy",&mut cy,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Diameter");changed|=widgets::num_f32(ui,"p_d",&mut d,step,0.001,3000.0,3,"mm");ui.end_row();
             if changed {s.x=cx-d/2.0;s.y=cy-d/2.0;s.w=d;s.h=d;}
           }
         }
       });
       ui.separator();ui.strong("Operation");
       if s.kind==ShapeKind::Line {ui.weak("Line stroke");}else {
         ui.horizontal_wrapped(|ui|{changed|=ui.radio_value(&mut s.operation,Operation::Outline,"Outline").changed();changed|=ui.radio_value(&mut s.operation,Operation::Fill,"Fill").changed();changed|=ui.radio_value(&mut s.operation,Operation::Both,"Both").changed();});
         ui.label("Fill type");ui.horizontal_wrapped(|ui|{changed|=ui.radio_value(&mut s.fill,Fill::Horizontal,"Horizontal").changed();changed|=ui.radio_value(&mut s.fill,Fill::Vertical,"Vertical").changed();changed|=ui.radio_value(&mut s.fill,Fill::Crosshatch,"Crosshatch").changed();});
       }
       egui::Grid::new("process-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
         if s.kind!=ShapeKind::Line {
           ui.label("Lines/mm");changed|=widgets::num_f32(ui,"p_lpm",&mut s.lines_per_mm,1.0,0.1,100.0,1,"");ui.end_row();
           ui.label("Spacing");ui.weak(format!("{:.4} mm",1.0/s.lines_per_mm.max(0.1)));ui.end_row();
         }
         ui.label("Feed");changed|=widgets::num_f32(ui,"p_feed",&mut s.feed,50.0,1.0,30000.0,0,"mm/min");ui.end_row();
         ui.label("Power");changed|=widgets::num_u32(ui,"p_pow",&mut s.power,10,0,max_power,&format!("/ {max_power}"));ui.end_row();
         ui.label("Passes");changed|=widgets::num_u32(ui,"p_pass",&mut s.passes,1,1,100,"");ui.end_row();
       });
       if changed {self.regenerate();}
     }}else{ui.weak("Select a shape on the canvas.");}
 }
 fn panel_header(&mut self,ui:&mut egui::Ui,id:usize) {
   // Docked header: title on the left, Float on the right. Collapse arrow is drawn by CollapsingState.
   ui.strong(TITLES[id]);
   ui.with_layout(egui::Layout::right_to_left(egui::Align::Center),|ui|{
     if ui.small_button("Float").on_hover_text("Detach into a movable, resizable window").clicked(){self.panels[id].floating=true;}
   });
 }
 fn projects_panel(&mut self,ui:&mut egui::Ui){
   // ---- saved projects: Open / Delete act on the dropdown selection ----
   egui::ComboBox::from_id_salt("saved_project_choice")
     .selected_text(if self.project_to_load.is_empty(){"Saved projects…"}else{&self.project_to_load})
     .width(ui.available_width()).show_ui(ui,|ui|{
       for name in &self.saved_names {ui.selectable_value(&mut self.project_to_load,name.clone(),name);}
     });
   let has_pick=!self.project_to_load.is_empty();
   ui.horizontal(|ui|{
     if ui.add_enabled(has_pick,egui::Button::new("Open").min_size(Vec2::new(80.0,26.0))).on_hover_text("Open the selected project").clicked(){
       let name=self.project_to_load.clone();self.request(Pending::Open(name));
     }
     if ui.add_enabled(has_pick,egui::Button::new("Delete").min_size(Vec2::new(80.0,26.0))).on_hover_text("Delete the selected project (asks first)").clicked(){
       self.dialog=Dialog::ConfirmDelete{name:self.project_to_load.clone()};
     }
   });
   ui.separator();
   // ---- active project: New / Save / Save As / Close / Export act on the open project ----
   ui.strong("Active Project");
   if self.project_active {
     ui.horizontal(|ui|{
       ui.label(&self.project.name);
       if self.dirty {ui.colored_label(Color32::from_rgb(235,190,70),"*").on_hover_text("Unsaved changes");}
       if !self.saved_once {ui.weak("(not saved yet)");}
     });
   } else {ui.weak("No project open.");}
   let active=self.project_active;
   ui.horizontal_wrapped(|ui|{
     if ui.button("New").clicked(){self.request(Pending::New);}
     if ui.add_enabled(active,egui::Button::new("Save")).clicked(){self.request_save();}
     if ui.add_enabled(active,egui::Button::new("Save As")).clicked(){self.request_save_as();}
     if ui.add_enabled(active,egui::Button::new("Close")).clicked(){self.request(Pending::Close);}
     if ui.add_enabled(active&&!self.plan.moves.is_empty(),egui::Button::new("Export G-code")).on_hover_text("Write the program to ~/LaserCNC/exports").clicked(){self.export_gcode();}
   });
   if !self.project_active {return;}
   // ---- job details ----
   ui.separator();ui.strong("Job Details");
   ui.label(format!("Estimated: {}",Self::duration_hms(self.plan.estimated_seconds)));
   ui.small("Constant-speed lower bound");
   ui.label(format!("Shapes: {}",self.project.shapes.len()));
   if let Some(shape)=self.project.shapes.first(){
      let (x0,y0,x1,y1)=shape.extent();
      ui.label(format!("Part: {:.1} × {:.1} mm",x1-x0,y1-y0));
      ui.label(format!("Fill: {:?} | {:?}",shape.operation,shape.fill));
      ui.label(format!("Density: {:.1} lines/mm",shape.lines_per_mm));
      ui.label(format!("Pitch: {:.3} mm",1.0/shape.lines_per_mm.max(0.1)));
      ui.label(format!("Speed: {:.0} mm/min",shape.feed));
      ui.label(format!("Power: S{} | Passes: {}",shape.power,shape.passes));
   }
   ui.label(format!("Engraving: {:.1} mm",self.plan.engraving_mm));
   ui.label(format!("Travel: {:.1} mm",self.plan.travel_mm));
   ui.label(format!("G-code: {} lines",self.plan.lines.len()));
   // ---- run history for this project ----
   ui.separator();ui.strong("Run History");
   let mine:Vec<(String,String)>=self.runs.iter().filter(|r|r.project_id==self.project.id)
     .map(|r|(r.id.clone(),format!("{} {:<5} {:<3} {:<9} {}",r.started.get(5..16).unwrap_or(&r.started),r.kind,if r.simulated{"SIM"}else{""},r.outcome,Self::duration_hms(r.duration_s)))).collect();
   let jobs=self.runs.iter().filter(|r|r.project_id==self.project.id&&r.kind=="Job").count();
   if mine.is_empty(){ui.weak("No runs yet.");return;}
   ui.label(format!("Runs: {} (Jobs {} · Traces {})",mine.len(),jobs,mine.len()-jobs));
   ui.label(format!("Last: {}",self.runs.iter().find(|r|r.project_id==self.project.id).map(|r|r.started.clone()).unwrap_or_default()));
   let mut clicked=None;
   egui::ScrollArea::vertical().id_salt("run_history").max_height(160.0).show(ui,|ui|{
     for (id,text) in &mine {
       if ui.add(egui::Button::new(egui::RichText::new(text).monospace().size(11.0)).selected(self.selected_run.as_ref()==Some(id)).frame(self.selected_run.as_ref()==Some(id)).truncate()).on_hover_text("Show in the Runs panel").clicked(){clicked=Some(id.clone());}
     }
   });
   if let Some(id)=clicked {self.select_run(&id);self.show_panel(ui.ctx(),6);}
 }
 fn select_run(&mut self,id:&str){
   self.selected_run=Some(id.to_string());
   self.notes_edit=self.runs.iter().find(|r|r.id==id).map(|r|r.notes.clone()).unwrap_or_default();
 }
 fn runs_panel(&mut self,ui:&mut egui::Ui){
   ui.horizontal_wrapped(|ui|{
     if ui.small_button("Refresh").clicked(){self.refresh_runs();}
     if ui.small_button("Runs folder").clicked(){runlog::open_folder(&runlog::runs_dir());}
     if ui.small_button("Exports").clicked(){runlog::open_folder(&runlog::exports_dir());}
     if ui.small_button("Daily logs").clicked(){runlog::open_folder(&runlog::logs_dir());}
   });
   if let Some(run)=&self.active_run {
     ui.colored_label(Color32::from_rgb(110,210,130),format!("Recording: {} · {} · {}",run.record.kind,run.record.project_name,Self::duration_hms(run.started.elapsed().as_secs_f32())));
   }
   if self.runs.is_empty(){ui.weak("No runs recorded yet.");return;}
   let mut clicked=None;
   egui::ScrollArea::vertical().id_salt("runs_list").max_height(200.0).auto_shrink([false,true]).show(ui,|ui|{
     for r in &self.runs {
       let c=match r.outcome.as_str(){"Completed"=>Color32::from_rgb(110,210,130),"Running"=>Color32::from_rgb(235,190,70),_=>Color32::from_rgb(235,110,100)};
       let text=egui::RichText::new(format!("{} {:<5} {:<3} {:<9} {}",r.started.get(5..16).unwrap_or(&r.started),r.kind,if r.simulated{"SIM"}else{""},r.outcome,r.project_name)).monospace().size(11.0).color(c);
       if ui.add(egui::Button::new(text).selected(self.selected_run.as_ref()==Some(&r.id)).frame(self.selected_run.as_ref()==Some(&r.id)).truncate()).on_hover_text(&r.id).clicked(){clicked=Some(r.id.clone());}
     }
   });
   if let Some(id)=clicked {self.select_run(&id);}
   let Some(sel)=self.selected_run.clone() else {return;};
   let Some(r)=self.runs.iter().find(|r|r.id==sel).cloned() else {return;};
   ui.separator();
   egui::Grid::new("run_details").num_columns(2).spacing(Vec2::new(10.0,4.0)).show(ui,|ui|{
     ui.weak("Project");ui.label(&r.project_name);ui.end_row();
     ui.weak("Type");ui.label(if r.simulated{format!("{} (SIM, virtual controller)",r.kind)}else{r.kind.clone()});ui.end_row();
     ui.weak("Started");ui.label(&r.started);ui.end_row();
     ui.weak("Ended");ui.label(&r.ended);ui.end_row();
     ui.weak("Duration");ui.label(if r.estimate_s>0.0{format!("{} (est. {})",Self::duration_hms(r.duration_s),Self::duration_hms(r.estimate_s))}else{Self::duration_hms(r.duration_s)});ui.end_row();
     ui.weak("Outcome");ui.label(&r.outcome);ui.end_row();
     if let Some(e)=&r.error {ui.weak("Error");ui.colored_label(Color32::from_rgb(240,95,95),format!("{e}{}",r.error_line.map(|l|format!(" (line {l})")).unwrap_or_default()));ui.end_row();}
     ui.weak("Lines");ui.label(format!("{} / {} acknowledged",r.lines_acked,r.lines_total));ui.end_row();
     let pauses=r.events.iter().filter(|e|e.event.contains("Door")||e.event.contains("Hold")).count();
     ui.weak("Pauses");ui.label(pauses.to_string());ui.end_row();
     ui.weak("Port");ui.add(egui::Label::new(format!("{} @ {}",r.port,r.baud)).truncate());ui.end_row();
     ui.weak("Firmware");ui.add(egui::Label::new(r.firmware.clone().unwrap_or_else(||"—".into())).truncate());ui.end_row();
   });
   ui.label("Notes");
   ui.add(egui::TextEdit::multiline(&mut self.notes_edit).desired_rows(3).desired_width(f32::INFINITY).hint_text("Material, focus, result…"));
   ui.horizontal(|ui|{
     if ui.add_enabled(self.notes_edit!=r.notes,egui::Button::new("Save notes")).clicked(){
       let mut updated=r.clone();updated.notes=self.notes_edit.clone();
       let mut ok=runlog::write_run_json(&updated).map_err(|e|e.to_string());
       if let Some(s)=&self.store{ok=ok.and(s.save_run(&updated));}
       self.notice=match ok{Ok(())=>"Run notes saved".into(),Err(e)=>format!("Notes save failed: {e}")};
       self.refresh_runs();
     }
     if ui.button("Open run folder").clicked(){runlog::open_folder(std::path::Path::new(&r.folder));}
   });
 }
 /// Serial panel: connection acknowledgement and read-only controller information.
 fn connection_info(&mut self,ui:&mut egui::Ui){
   let green=Color32::from_rgb(110,210,130);let amber=Color32::from_rgb(235,190,70);let red=Color32::from_rgb(235,90,90);
   let m=&self.machine;
   match m.link{
     Link::Disconnected=>{ui.colored_label(if m.status.starts_with("Connect failed")||m.status.starts_with("Port error"){red}else{Color32::from_gray(150)},&m.status);return;}
     Link::Connecting=>{ui.colored_label(amber,&m.status);return;}
     Link::Connected=>{ui.colored_label(green,"● Connected");}
   }
   let dash=|o:&Option<String>|o.clone().unwrap_or_else(||"—".into());
   let alarm=m.state.starts_with("Alarm");
   egui::Grid::new("controller_info").num_columns(2).spacing(Vec2::new(10.0,4.0)).show(ui,|ui|{
     ui.weak("Firmware");ui.add(egui::Label::new(dash(&m.info.banner)).truncate());ui.end_row();
     ui.weak("Build");ui.add(egui::Label::new(dash(&m.info.build)).truncate());ui.end_row();
     ui.weak("Port");ui.label(if m.simulate{"SIMULATOR (virtual GRBL 0.9i)".to_string()}else{format!("{} @ {}",m.selected_port,m.baud)});ui.end_row();
     let st=if m.state.is_empty(){"—".to_string()}else{m.state.clone()};
     let sc=if alarm{red}else if st.starts_with("Door")||st.starts_with("Hold"){amber}else if st.starts_with("Run"){green}else{Color32::from_gray(215)};
     ui.weak("State");ui.colored_label(sc,st);ui.end_row();
     if let Some(p)=m.info.wpos{ui.weak("WPos");ui.monospace(format!("X{:.3} Y{:.3} Z{:.3}",p.0,p.1,p.2));ui.end_row();}
     if let Some(p)=m.info.mpos{ui.weak("MPos");ui.monospace(format!("X{:.3} Y{:.3} Z{:.3}",p.0,p.1,p.2));ui.end_row();}
     ui.weak("Laser");
     match m.info.spindle(){
       Some((w,s)) if w=="M5"=>{ui.colored_label(green,format!("M5 off ({s})"));}
       Some((w,s))=>{ui.colored_label(amber,format!("{w} ON {s}"));}
       None=>{ui.label("—");}
     }
     ui.end_row();
     ui.weak("Modes");
     match &m.info.modes{Some(md)=>{ui.add(egui::Label::new(egui::RichText::new(md).monospace().size(11.0)).truncate()).on_hover_text(md);}None=>{ui.label("—");}}
     ui.end_row();
   });
   let offsets=m.info.offsets.clone();let settings=m.info.settings.clone();
   if ui.add_enabled(alarm,egui::Button::new("Unlock ($X)")).on_hover_text("Clear the GRBL alarm lock. Check the head position before running.").clicked(){self.machine.unlock();}
   egui::CollapsingHeader::new(format!("Work offsets ($#) · {}",offsets.len())).id_salt("info_offsets").show(ui,|ui|{
     for o in &offsets{ui.label(egui::RichText::new(o).monospace().size(11.0));}
   });
   egui::CollapsingHeader::new(format!("Settings ($$) · {}",settings.len())).id_salt("info_settings").show(ui,|ui|{
     egui::ScrollArea::vertical().id_salt("info_settings_scroll").max_height(220.0).show(ui,|ui|{
       for s in &settings{ui.add(egui::Label::new(egui::RichText::new(s).monospace().size(11.0)).truncate());}
     });
   });
 }
 fn panel_content(&mut self,ui:&mut egui::Ui,id:usize) {
   match id {
     0=>{
       if self.machine.simulate{
         ui.colored_label(Color32::from_rgb(190,150,240),"SIMULATOR — virtual GRBL 0.9i");
         ui.weak("Simulate is on: Connect attaches the virtual controller, not the Nano.");
       }else{
         ui.label("Serial port");
         ui.add(egui::TextEdit::singleline(&mut self.machine.selected_port).desired_width(f32::INFINITY));
         ui.horizontal(|ui|{
           egui::ComboBox::from_id_salt("ports").selected_text("Ports").show_ui(ui,|ui|{for port in self.machine.ports.clone(){if ui.selectable_label(self.machine.selected_port==port,&port).clicked(){self.machine.selected_port=port;}}});
           if ui.button("Refresh").clicked(){self.machine.refresh();}
         });
         ui.horizontal(|ui|{
           ui.label("Baud");
           egui::ComboBox::from_id_salt("baud").selected_text(self.machine.baud.to_string()).show_ui(ui,|ui|{for baud in [9600,57600,115200,230400]{ui.selectable_value(&mut self.machine.baud,baud,baud.to_string());}});
         });
       }
       match self.machine.link{
         Link::Disconnected=>{if Self::action(ui,"Connect",Color32::from_rgb(30,121,71)).clicked(){self.machine.connect();}}
         Link::Connecting=>{if Self::action(ui,"Cancel",Color32::from_rgb(91,99,112)).on_hover_text("Stop waiting for the GRBL banner and close the port").clicked(){self.machine.disconnect();}}
         Link::Connected=>{if Self::action(ui,"Disconnect",Color32::from_rgb(91,99,112)).clicked(){self.machine.disconnect();}}
       }
       self.connection_info(ui);
     },
     1=>{self.projects_panel(ui);},
     2=>{self.shape_properties(ui);},
     3=>{
       let mut changed=false;
       egui::Grid::new("machine-profile-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
         ui.label("Work X");changed|=widgets::num_f32(ui,"mp_wx",&mut self.profile.work_x,10.0,1.0,3000.0,1,"mm");ui.end_row();
         ui.label("Work Y");changed|=widgets::num_f32(ui,"mp_wy",&mut self.profile.work_y,10.0,1.0,3000.0,1,"mm");ui.end_row();
         ui.label("Travel F");changed|=widgets::num_f32(ui,"mp_tf",&mut self.profile.travel_feed,100.0,1.0,30000.0,0,"mm/min");ui.end_row();
         ui.label("Max PWM S");changed|=widgets::num_u32(ui,"mp_pwm",&mut self.profile.max_power,100,1,100000,"");ui.end_row();
       });
       ui.weak("Saved with the app settings; applies to every project.");
       if changed {self.regenerate();}
     },
     4=>{
       ui.weak("Step jogging (GRBL 0.9i)");
       ui.label("Step (mm)");
       let step=ui.data_mut(|d|d.get_persisted::<f32>(egui::Id::new("jog_step")).unwrap_or(1.0));
       let mut new_step=step;
       ui.horizontal_wrapped(|ui|{
         for v in [0.1,10.0] {if ui.selectable_label((step-v).abs()<0.001,format!("{v} mm")).clicked(){new_step=v;}}
         // Typed step distance with ▲▼ (any value 0.01–500 mm).
         widgets::num_f32(ui,"jog_step_field",&mut new_step,1.0,0.01,500.0,2,"mm");
       });
       ui.data_mut(|d|d.insert_persisted(egui::Id::new("jog_step"),new_step));
       ui.add_space(4.0);
       ui.vertical_centered(|ui|{
         let sz=Vec2::new(74.0,62.0);
         let gap=5.0;
         let pad=Color32::from_rgb(44,47,53);
         egui::Grid::new("jog_cross").spacing(Vec2::new(gap,gap)).show(ui,|ui|{
           ui.label("");
           if ui.add_sized(sz,egui::Button::new(egui::RichText::new("⏶ Y+").size(17.0)).fill(pad)).clicked(){self.jog(0.0,new_step);}
           ui.label("");ui.end_row();
           if ui.add_sized(sz,egui::Button::new(egui::RichText::new("⏴ X−").size(17.0)).fill(pad)).clicked(){self.jog(-new_step,0.0);}
           ui.add_enabled(false,egui::Button::new(egui::RichText::new("HOME").size(15.0)).min_size(sz));
           if ui.add_sized(sz,egui::Button::new(egui::RichText::new("X+ ⏵").size(17.0)).fill(pad)).clicked(){self.jog(new_step,0.0);}
           ui.end_row();
           ui.label("");
           if ui.add_sized(sz,egui::Button::new(egui::RichText::new("⏷ Y−").size(17.0)).fill(pad)).clicked(){self.jog(0.0,-new_step);}
           ui.label("");ui.end_row();
         });
       });
       ui.add_space(4.0);
       ui.horizontal_wrapped(|ui|{
         let ok=self.machine.connected()&&!self.machine.running;
         if ui.add_enabled(ok,egui::Button::new("Zero X")).on_hover_text("G92 X0: the spot under the head is job X 0").clicked(){self.machine.command("G92 X0");self.zero_x=true;}
         if ui.add_enabled(ok,egui::Button::new("Zero Y")).on_hover_text("G92 Y0: the spot under the head is job Y 0").clicked(){self.machine.command("G92 Y0");self.zero_y=true;}
         if ui.add_enabled(ok,egui::Button::new("Go to zero")).on_hover_text("G90 G0 X0 Y0").clicked(){self.machine.command("G90 G0 X0 Y0");self.jog_pred=Some((0.0,0.0));self.last_jog=Some(Instant::now());}
       });
       let zx=if self.zero_x{"set"}else{"not set"};let zy=if self.zero_y{"set"}else{"not set"};
       ui.weak(format!("Job zero: X {zx} · Y {zy} (this session)"));
     },
     5=>{self.monitor_contents(ui,false);},
     6=>{self.runs_panel(ui);},
     _=>{}
   }
 }
 fn panel_ui(&mut self,ui:&mut egui::Ui,id:usize){
   // Collapse state is stored in egui memory and persisted between launches.
   // First launch default: only Serial Communications (id 0) is expanded.
   let state=CollapsingState::load_with_default_open(ui.ctx(),egui::Id::new(("panel",TITLES[id])),id==0);
   let _=state.show_header(ui,|ui|self.panel_header(ui,id)).body_unindented(|ui|{ui.add_space(2.0);self.panel_content(ui,id);ui.add_space(4.0);});
 }
 fn panels_layout(&mut self,ctx:&egui::Context){
   // Left dock: Serial, Projects, Machine Profile, Runs.
   egui::SidePanel::left("left_dock").resizable(true).default_width(280.0).min_width(220.0).show(ctx,|ui|{
     egui::ScrollArea::vertical().show(ui,|ui|{
       for id in LEFT_DOCK {if !self.panels[id].floating {self.panel_ui(ui,id);ui.separator();}}
     });
   });
   // Right dock: Shape Properties, Jog Controls, G-code / Stream stacked in one column.
   if RIGHT_DOCK.iter().any(|&id|!self.panels[id].floating) {
     egui::SidePanel::right("right_dock").resizable(true).default_width(320.0).min_width(240.0).show(ctx,|ui|{
       egui::ScrollArea::vertical().show(ui,|ui|{
         for id in RIGHT_DOCK {if !self.panels[id].floating {self.panel_ui(ui,id);ui.separator();}}
       });
     });
   }
   // Floating windows: native title bar (drag to move, arrow to collapse), resizable edges.
   for id in 0..TITLES.len() {
     if self.panels[id].floating {
       let mut win=egui::Window::new(TITLES[id]).id(egui::Id::new(("floating_panel",TITLES[id])));
       // G-code / Stream floats over the canvas: background 80 % transparent, no shadow.
       if id==5{win=win.frame(egui::Frame::window(&ctx.style()).fill(Color32::from_rgba_unmultiplied(22,23,26,51)).shadow(egui::Shadow::NONE));}
       win.resizable(true).collapsible(true).min_width(240.0).min_height(160.0)
         .default_size(match id{5=>Vec2::new(380.0,620.0),PROPS=>Vec2::new(300.0,520.0),_=>Vec2::new(320.0,360.0)})
         .default_pos(if id==PROPS{egui::Pos2::new(330.0,190.0)}else{egui::Pos2::new(340.0+id as f32*24.0,150.0+id as f32*24.0)})
         .show(ctx,|ui|{
           ui.horizontal(|ui|{
             ui.with_layout(egui::Layout::right_to_left(egui::Align::Center),|ui|{
               let tip="Return to the default docked position";
               if ui.small_button("Dock").on_hover_text(tip).clicked(){self.panels[id].floating=false;}
             });
           });
           ui.separator();
           // Content fills the window so a corner drag sets the size directly (no snap-back).
           if id==5 {self.monitor_contents(ui,true);}
           else {egui::ScrollArea::vertical().id_salt(("floating_scroll",id)).auto_shrink([false,false]).show(ui,|ui|self.panel_content(ui,id));}
         });
     }
   }
 }
 fn toolbar(&mut self,ui:&mut egui::Ui){
   // ---- Row 1: application name, File, Help ----
   egui::menu::bar(ui,|ui|{
     ui.strong(format!("Laser CNC v{}",env!("CARGO_PKG_VERSION")));
     ui.separator();
     ui.menu_button("File",|ui|{
       let active=self.project_active;
       if ui.button("New").clicked(){self.request(Pending::New);ui.close_menu();}
       if ui.button("Open…").on_hover_text("Choose a project in the Projects panel").clicked(){self.panels[1].floating=false;self.show_panel(ui.ctx(),1);ui.close_menu();}
       ui.separator();
       if ui.add_enabled(active,egui::Button::new("Save")).clicked(){self.request_save();ui.close_menu();}
       if ui.add_enabled(active,egui::Button::new("Save As…")).clicked(){self.request_save_as();ui.close_menu();}
       if ui.add_enabled(active,egui::Button::new("Close")).clicked(){self.request(Pending::Close);ui.close_menu();}
       ui.separator();
       if ui.button("Delete…").on_hover_text("Choose a project in the Projects panel").clicked(){self.panels[1].floating=false;self.show_panel(ui.ctx(),1);ui.close_menu();}
       if ui.add_enabled(active&&!self.plan.moves.is_empty(),egui::Button::new("Export G-code")).clicked(){self.export_gcode();ui.close_menu();}
     });
     ui.menu_button("Help",|ui|{
       ui.label("Laser CNC Controller");ui.label(format!("Version {}",env!("CARGO_PKG_VERSION")));
       ui.label(format!("Data folder: {}",runlog::data_root().display()));
     });
   });
   ui.separator();
   // ---- Row 2: fixed machine controls (wraps on narrow windows) ----
   ui.horizontal_wrapped(|ui|{
     ui.spacing_mut().item_spacing.x=6.0;
     let max=self.max_power;
     let (spot_label,spot_fill,spot_tip)=if self.spot_on{("Spot OFF",Color32::from_rgb(196,132,28),"Spot is ON: click to send M5")}
       else{("Spot ON",Color32::from_rgb(38,84,148),"Low-power spot for positioning: M3 S<power>. Click again to switch off.")};
     if Self::action(ui,spot_label,spot_fill).on_hover_text(spot_tip).clicked(){self.toggle_spot();}
     widgets::num_u32(ui,"spot_power",&mut self.spot_power,1,0,max,&format!("/ {max}"));
     ui.separator();
     if Self::action(ui,"Trace",Color32::from_rgb(85,92,110)).on_hover_text("Trace the selected outline at spot power and trace feed, relative to the job origin").clicked(){
       match self.trace_program(){Ok((lines,path))=>self.begin_run("Trace",lines,path),Err(e)=>self.notice=e}
     }
     let fence_on=self.project.fence.on;
     if !fence_on&&self.trace_target==TraceTarget::Fence{self.trace_target=TraceTarget::Job;}
     egui::ComboBox::from_id_salt("trace_target").width(70.0)
       .selected_text(match self.trace_target{TraceTarget::Part=>"Part",TraceTarget::Job=>"Job",TraceTarget::Fence=>"Fence"})
       .show_ui(ui,|ui|{
         ui.selectable_value(&mut self.trace_target,TraceTarget::Part,"Part").on_hover_text("Box around the selected shape(s)");
         ui.selectable_value(&mut self.trace_target,TraceTarget::Job,"Job").on_hover_text("Box around all shapes");
         ui.add_enabled_ui(fence_on,|ui|{ui.selectable_value(&mut self.trace_target,TraceTarget::Fence,"Fence").on_hover_text("The work fence rectangle").on_disabled_hover_text("Turn the work fence on in the canvas tool strip");});
       });
     widgets::num_f32(ui,"frame_feed",&mut self.frame_feed,100.0,1.0,10000.0,0,"mm/min");
     ui.separator();
     if Self::action(ui,"⏵ Start",Color32::from_rgb(27,125,68)).clicked(){
       if !self.plan.moves.is_empty()&&self.plan.warnings.is_empty(){let lines=self.plan.lines.clone();let path=self.plan.moves.clone();self.begin_run("Job",lines,path);}
       else{self.notice="Start requires a valid toolpath and no warnings".into();}
     }
     // Pause/Resume driven by the GRBL state word from status reports.
     let state=self.machine.state.clone();
     let door=state.starts_with("Door");
     let held=door||state.starts_with("Hold");
     if held {
       if Self::action(ui,"⏵ Resume",Color32::from_rgb(37,83,133)).on_hover_text("Cycle start (~): GRBL restores the laser, then continues").clicked(){
         self.machine.resume();if let Some(r)=&mut self.active_run{r.event("resume requested (~)");}
       }
     } else {
       let can_pause=state=="Run"||self.machine.running;
       if ui.add_enabled(can_pause,egui::Button::new(egui::RichText::new("⏸ Pause").color(Color32::WHITE).strong()).fill(Color32::from_rgb(143,103,34)).min_size(Vec2::new(88.0,30.0)))
         .on_hover_text("Safety-door pause (@): hold position, laser off").clicked(){
         self.machine.pause();if let Some(r)=&mut self.active_run{r.event("pause requested (@)");}
       }
     }
     // DOOR flag: fixed-width slot so the row does not shift when it appears.
     let (slot,_)=ui.allocate_exact_size(Vec2::new(54.0,22.0),egui::Sense::hover());
     if door {
       ui.painter().rect_filled(slot,CornerRadius::same(4),Color32::from_rgb(214,160,40));
       ui.painter().text(slot.center(),egui::Align2::CENTER_CENTER,"DOOR",egui::FontId::proportional(12.0),Color32::BLACK);
       ui.interact(slot,egui::Id::new("door_flag"),egui::Sense::hover()).on_hover_text("Controller reports Door: motion held, laser off. Resume restores the laser, then continues.");
     }
     if Self::action(ui,"⏹ Reset",Color32::from_rgb(170,38,44)).on_hover_text("Feed hold + soft reset (Ctrl-X). Not a physical emergency stop.").clicked(){
       if let Some(r)=&mut self.active_run{r.reset_requested=true;r.event("reset requested");}
       self.machine.abort();
     }
     ui.separator();
     if Self::action(ui,"Laser OFF",Color32::from_rgb(144,53,55)).on_hover_text("M5 (also switches Spot off)").clicked(){self.laser_off();}
     ui.separator();
     // Simulate: virtual controller instead of the Nano. Switchable only while disconnected.
     let sim=self.machine.simulate;let can_switch=self.machine.link==Link::Disconnected;
     let fill=if sim{Color32::from_rgb(112,72,170)}else{Color32::from_rgb(70,74,82)};
     let tip=if can_switch{if sim{"Simulate is ON: Connect uses the virtual controller. Click to switch off."}else{"Switch to the virtual controller (no machine)"}}else{"Disconnect first to switch Simulate"};
     if ui.add_enabled(can_switch,egui::Button::new(egui::RichText::new("Simulate").color(Color32::WHITE).strong()).fill(fill).min_size(Vec2::new(88.0,30.0)))
       .on_hover_text(tip).on_disabled_hover_text(tip).clicked(){self.machine.simulate=!sim;}
     let mut speed=self.machine.sim_speed;
     egui::ComboBox::from_id_salt("sim_speed").width(56.0).selected_text(format!("{}×",speed as u32)).show_ui(ui,|ui|{
       for sp in SIM_SPEEDS{ui.selectable_value(&mut speed,sp,format!("{}×",sp as u32));}
     }).response.on_hover_text("Simulate speed (applies during a run)");
     if speed!=self.machine.sim_speed{self.machine.set_sim_speed(speed);}
     if sim{
       let (slot,_)=ui.allocate_exact_size(Vec2::new(96.0,22.0),egui::Sense::hover());
       ui.painter().rect_filled(slot,CornerRadius::same(4),Color32::from_rgb(112,72,170));
       ui.painter().text(slot.center(),egui::Align2::CENTER_CENTER,"SIMULATION",egui::FontId::proportional(12.0),Color32::WHITE);
     }
   });
   ui.add_space(2.0);
 }
 fn canvas(&mut self,ui:&mut egui::Ui){
   if self.view.toolbar(ui,&mut self.project,&mut self.selected){self.regenerate();}
   ui.add_space(2.0);
   let sim=match &self.live{
     // Live run: own path, progress from the reported position.
     Some(lv)=>{
       // Laser colour: the virtual controller's state in Simulate, else the current path segment (burn / travel).
       let laser=self.machine.sim_laser().unwrap_or_else(||lv.path.get(lv.progress).is_some_and(|m|m.laser));
       SimOverlay{active:lv.head.is_some(),index:lv.progress,head:lv.head.unwrap_or(lv.origin),start:lv.origin,path:Some(lv.path.as_slice()),laser}
     }
     None=>SimOverlay{active:false,index:0,head:self.plan.origin,start:self.plan.origin,path:None,laser:false},
   };
   if self.view.show(ui,&mut self.project,&mut self.selected,&self.plan,&sim){self.regenerate();}
   // A newly selected shape expands the docked Shape Properties panel.
   if !self.selected.is_empty()&&self.selected!=self.props_last_sel&&!self.panels[PROPS].floating {self.show_panel(ui.ctx(),PROPS);}
   self.props_last_sel=self.selected.clone();
 }
 fn monitor_contents(&mut self,ui:&mut egui::Ui,floating:bool){
   // Source: what is/was streamed (program or trace); otherwise the generated program.
   let live_job=!self.job_lines.is_empty();
   let total=if live_job{self.job_lines.len()}else{self.plan.lines.len()};
   // Counts never go negative: after Reset / Disconnect `next_line` returns to 0 while `acknowledged` keeps the run's count.
   let sent=self.machine.next_line.min(total);let acked=self.machine.acknowledged.min(sent);
   let current=sent.saturating_sub(1);
   let mono=egui::FontId::monospace(11.0);
   let small=egui::FontId::proportional(11.5);
   // ---- header: counts + copy ----
   ui.horizontal(|ui|{
     ui.label(egui::RichText::new(format!("Queued {}",total-sent)).font(small.clone()).color(Color32::from_gray(160)));
     ui.label(egui::RichText::new(format!("Sent {}",sent-acked)).font(small.clone()).color(Color32::from_rgb(235,200,80)));
     ui.label(egui::RichText::new(format!("OK {}",acked)).font(small.clone()).color(Color32::from_rgb(110,210,130)));
     ui.with_layout(egui::Layout::right_to_left(egui::Align::Center),|ui|{
       if ui.small_button("Copy G-code").on_hover_text("Copy the generated program").clicked(){ui.ctx().copy_text(self.plan.lines.join("\n"));}
     });
   });
   // ---- controller state ----
   ui.horizontal(|ui|{
     let st=if self.machine.connected(){if self.machine.state.is_empty(){"—".to_string()}else{self.machine.state.clone()}}else{"Disconnected".to_string()};
     let c=match st.as_str(){s if s.starts_with("Run")=>Color32::from_rgb(110,210,130),s if s.starts_with("Door")||s.starts_with("Hold")=>Color32::from_rgb(235,190,70),
       s if s.starts_with("Alarm")=>Color32::from_rgb(235,90,90),_=>Color32::from_gray(170)};
     ui.label(egui::RichText::new("Controller").font(small.clone()).color(Color32::from_gray(140)));
     ui.label(egui::RichText::new(st).font(small.clone()).color(c).strong());
     let src=if live_job{if self.machine.simulate{"· streamed (SIM)"}else{"· streamed"}}else{"· program"};
     ui.label(egui::RichText::new(src).font(small.clone()).color(Color32::from_gray(120)));
   });
   ui.separator();
   // ---- sizes: floating fills its window; docked uses a share of the screen ----
   // A new window's sizing pass reports unbounded space, so use a fixed starting height there.
   let avail=if ui.is_sizing_pass(){480.0}else{ui.available_height()};
   let (stream_h,log_h)=if floating{let l=(avail*0.28).clamp(60.0,260.0);((avail-l-64.0).max(60.0),l)}
     else{let h=ui.ctx().screen_rect().height();((h*0.36).clamp(200.0,700.0),(h*0.12).clamp(80.0,200.0))};
   // ---- program lines (virtualised rows, follows the active line) ----
   let row_h=ui.fonts(|f|f.row_height(&mono));
   let spacing=ui.spacing().item_spacing.y;
   let mut area=egui::ScrollArea::both().id_salt("gcode_lines").max_height(stream_h).auto_shrink([false,false]);
   if !floating {area=area.min_scrolled_height(stream_h);}
   let active=self.machine.running;
   if active&&current!=self.monitor_follow {
     area=area.vertical_scroll_offset(((current as f32)*(row_h+spacing)-stream_h*0.4).max(0.0));
     self.monitor_follow=current;
   }
   let err_line=self.machine.error_line;
   let lines=if live_job{&self.job_lines}else{&self.plan.lines};
   area.show_rows(ui,row_h,total,|ui,range|{
     for i in range {
       let (mark,color)=if err_line==Some(i){("✖",Color32::from_rgb(240,95,95))}
       else if live_job&&i<acked{("✔",Color32::from_rgb(110,210,130))}
       else if live_job&&i<sent{("▶",Color32::from_rgb(250,220,90))}
       else{("·",Color32::from_gray(150))};
       ui.add(egui::Label::new(egui::RichText::new(format!("{mark} {:05} {}",i+1,lines[i])).font(mono.clone()).color(color)).wrap_mode(egui::TextWrapMode::Extend));
     }
   });
   ui.separator();
   // ---- serial log: manual TX, controller messages, errors ----
   ui.label(egui::RichText::new("Serial RX/TX").font(small).strong());
   let mut log_area=egui::ScrollArea::both().id_salt("serial_log").max_height(log_h).auto_shrink([false,false]).stick_to_bottom(true);
   if !floating {log_area=log_area.min_scrolled_height(log_h);}
   log_area.show(ui,|ui|{
     for msg in &self.machine.log {
       let c=if msg.starts_with("TX"){Color32::from_rgb(120,175,235)}
         else if msg.contains("error")||msg.contains("ALARM"){Color32::from_rgb(240,95,95)}
         else if msg.starts_with("RX"){Color32::from_gray(215)}else{Color32::from_gray(150)};
       ui.add(egui::Label::new(egui::RichText::new(msg).font(mono.clone()).color(c)).wrap_mode(egui::TextWrapMode::Extend));
     }
   });
   // ---- manual command line ----
   ui.horizontal(|ui|{
     let can=self.machine.connected()&&!self.machine.running;
     let width=(ui.available_width()-64.0).max(80.0);
     let edit=ui.add_enabled(can,egui::TextEdit::singleline(&mut self.manual_cmd).font(mono.clone()).desired_width(width).hint_text("G-code or $ command"));
     let enter=edit.lost_focus()&&ui.input(|i|i.key_pressed(egui::Key::Enter));
     let has=!self.manual_cmd.trim().is_empty();
     if ui.add_enabled(can&&has,egui::Button::new("Send")).clicked()||(enter&&can&&has){
       let c=self.manual_cmd.trim().to_string();self.machine.command(&c);self.note_manual(&c);self.manual_cmd.clear();edit.request_focus();
     }
   });
 }
 // ======================================================================== dialogs
 fn dialogs(&mut self,ctx:&egui::Context){
   let dialog=std::mem::replace(&mut self.dialog,Dialog::None);
   let modal=|title:&str|egui::Window::new(title).collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER,Vec2::ZERO);
   self.dialog=match dialog {
     Dialog::None=>Dialog::None,
     Dialog::SaveAs{mut name,copy,then}=>{
       let mut next=None;
       modal(if copy{"Save Project As"}else{"Save Project"}).show(ctx,|ui|{
         ui.label("Project name");
         let r=ui.add(egui::TextEdit::singleline(&mut name).desired_width(260.0));
         if !r.has_focus()&&ui.memory(|m|m.focused().is_none()){r.request_focus();}
         let enter=ui.input(|i|i.key_pressed(egui::Key::Enter));
         if ui.input(|i|i.key_pressed(egui::Key::Escape)){next=Some(Dialog::None);}
         ui.horizontal(|ui|{
           let ok=!name.trim().is_empty();
           if ui.add_enabled(ok,egui::Button::new("Save")).clicked()||(enter&&ok){
             let n=name.trim().to_string();
             // Re-saving this project's own entry is not an overwrite of another project.
             let replaces_other=self.saved_names.contains(&n)&&!(self.saved_once&&!copy&&n==self.project.name);
             next=Some(if replaces_other{Dialog::ConfirmOverwrite{name:n,copy,then:then.clone()}}else{self.finish_save_as(n,copy,then.clone());Dialog::None});
           }
           if ui.button("Cancel").clicked(){next=Some(Dialog::None);}
         });
       });
       next.unwrap_or(Dialog::SaveAs{name,copy,then})
     }
     Dialog::ConfirmOverwrite{name,copy,then}=>{
       let mut next=None;
       modal("Replace Project?").show(ctx,|ui|{
         ui.label(format!("A project named '{name}' already exists. Replace it?"));
         ui.horizontal(|ui|{
           if ui.add(egui::Button::new(egui::RichText::new("Replace").color(Color32::WHITE)).fill(Color32::from_rgb(158,43,49))).clicked(){self.finish_save_as(name.clone(),copy,then.clone());next=Some(Dialog::None);}
           if ui.button("Cancel").clicked(){next=Some(Dialog::None);}
         });
       });
       next.unwrap_or(Dialog::ConfirmOverwrite{name,copy,then})
     }
     Dialog::ConfirmDelete{name}=>{
       let mut next=None;
       modal("Delete Project?").show(ctx,|ui|{
         ui.label(format!("Delete '{name}' permanently?"));
         ui.weak("Its run records and run folders are kept.");
         ui.horizontal(|ui|{
           if ui.add(egui::Button::new(egui::RichText::new("Delete").color(Color32::WHITE)).fill(Color32::from_rgb(158,43,49))).clicked(){self.delete_project(&name);next=Some(Dialog::None);}
           if ui.button("Cancel").clicked(){next=Some(Dialog::None);}
         });
       });
       next.unwrap_or(Dialog::ConfirmDelete{name})
     }
     Dialog::Unsaved{then}=>{
       let mut next=None;
       modal("Unsaved Changes").show(ctx,|ui|{
         ui.label(format!("'{}' has unsaved changes.",self.project.name));
         ui.horizontal(|ui|{
           if ui.button("Save").clicked(){
             if self.saved_once {self.save();self.perform(then.clone());next=Some(Dialog::None);}
             else {next=Some(Dialog::SaveAs{name:self.project.name.clone(),copy:false,then:Some(then.clone())});}
           }
           if ui.button("Discard").clicked(){self.perform(then.clone());next=Some(Dialog::None);}
           if ui.button("Cancel").clicked(){next=Some(Dialog::None);}
         });
       });
       next.unwrap_or(Dialog::Unsaved{then})
     }
     Dialog::Notice{title,msg}=>{
       let mut next=None;
       modal(&title).show(ctx,|ui|{
         ui.set_max_width(380.0);
         ui.label(&msg);
         ui.add_space(6.0);
         if ui.button("OK").clicked()||ui.input(|i|i.key_pressed(egui::Key::Enter)||i.key_pressed(egui::Key::Escape)){next=Some(Dialog::None);}
       });
       next.unwrap_or(Dialog::Notice{title,msg})
     }
   };
 }
}

/// Distance from point p to segment a-b.
fn seg_dist(p:(f32,f32),a:(f32,f32),b:(f32,f32))->f32{
  let (dx,dy)=(b.0-a.0,b.1-a.1);let l2=dx*dx+dy*dy;
  let t=if l2>0.0{(((p.0-a.0)*dx+(p.1-a.1)*dy)/l2).clamp(0.0,1.0)}else{0.0};
  ((p.0-a.0-t*dx).powi(2)+(p.1-a.1-t*dy).powi(2)).sqrt()
}

impl eframe::App for LaserApp {
 fn update(&mut self,ctx:&egui::Context,_frame:&mut eframe::Frame){
   let dt=self.last_frame.elapsed().as_secs_f32().min(0.2);self.last_frame=Instant::now();
   self.machine.poll();
   // A controller reset (banner) or disconnect invalidates the work zero and the spot state.
   if self.machine.reset_count!=self.seen_resets||!self.machine.connected(){
     self.seen_resets=self.machine.reset_count;self.zero_x=false;self.zero_y=false;self.spot_on=false;self.jog_pred=None;
   }
   if self.machine.state.starts_with("Alarm"){self.spot_on=false;}
   self.drain_journal();
   self.track_run();
   self.update_live(dt);
   // Drawing on an empty canvas starts an (unsaved) project.
   if !self.project_active&&!self.project.shapes.is_empty(){self.project_active=true;self.saved_once=false;}
   self.dirty=self.project_active&&Self::comparable_json(&self.project)!=self.saved_json;
   let title=format!("Laser CNC Controller v{}{}",env!("CARGO_PKG_VERSION"),
     if self.project_active{format!(" — {}{}",self.project.name,if self.dirty{" *"}else{""})}else{String::new()});
   if title!=self.last_title {ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));self.last_title=title;}
   egui::TopBottomPanel::top("top_bar").show(ctx,|ui|self.toolbar(ui));
   self.panels_layout(ctx);
   egui::CentralPanel::default().show(ctx,|ui|{
     for w in &self.plan.warnings {ui.colored_label(Color32::LIGHT_RED,w);}
     if let Some(err)=&self.machine.error{ui.colored_label(Color32::LIGHT_RED,err);}
     if !self.notice.is_empty(){ui.label(&self.notice);}
     self.canvas(ui);
   });
   self.dialogs(ctx);
   ctx.request_repaint_after(std::time::Duration::from_millis(16));
 }
 fn save(&mut self,storage:&mut dyn eframe::Storage){
   // Floating/docked flags. Collapse state, window positions/sizes and dock widths
   // are persisted automatically through egui memory.
   eframe::set_value(storage,LAYOUT_KEY,&self.panels);
   eframe::set_value(storage,PROFILE_KEY,&self.profile);
 }
}
// =============================================================================
// End of file: app.rs
// Location:    laser-cnc-controller/src/app.rs
// Created:     2026-10-13 18:25:33 UTC
// =============================================================================
