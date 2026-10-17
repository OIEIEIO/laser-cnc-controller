// =============================================================================
// File:     app.rs
// Location: laser-cnc-controller/src/app.rs
// Version:  v0.2.2
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
//           v0.2.0 (motion-planner crate):
//           - Motion settings: $110/$111 max rate, $120/$121 acceleration,
//             $11 junction deviation from the controller's $$ while connected;
//             otherwise from the Machine Profile (new fields, defaults 3000 /
//             3000 mm/min, 10 / 10 mm/s², 0.010 mm). Look-ahead 18 blocks.
//             The program is re-planned when the source or values change.
//           - Live run view (Job and Trace, real and Simulate): the streamed
//             lines are planned into a trajectory at Start, from the head's
//             WPos at that moment. The head marker, the done path and the
//             laser colour follow a run clock along that trajectory, so the
//             marker moves smoothly at the planned speeds and follows every
//             corner. The clock runs at the Simulate speed in Simulate, and
//             stops while the controller reports Hold, Door or Alarm.
//             Replaces following the smoothed 250 ms WPos reports.
//           - Job Details estimate: planned with acceleration, corners and the
//             stops at M3/M5 (src/gcode.rs).
//           v0.2.1 (canvas CAD update):
//           - Canvas Tools is a panel (id 7, "Canvas Tools"): docked above the
//             canvas in two rows, collapsible to its title bar, or floating
//             in a resizable window where the tool groups wrap to the window
//             width. Layout key v5 (floating flags reset once).
//           - Undo / redo: every committed project edit (canvas, Shape
//             Properties, origin, fence) is one step; drags commit when they
//             end. Tool strip buttons, Ctrl+Z, Ctrl+Y / Ctrl+Shift+Z.
//             History is cleared on New / Open / Close / Duplicate. Project
//             name, id and read-only flag are not part of the history.
//           - Shape Properties rewritten: layer; geometry per kind (rectangle,
//             line, circle, polyline, merged region, text); Trace (width,
//             along / across, density, line count, end cap, corners); Border
//             (width, alignment, direction, density, line count) and
//             Interior (outline pass, fill pattern or none, density,
//             relation abut / overlap / gap); Raster (bi- / unidirectional,
//             overscan); Process; Burn order (first / earlier / later /
//             last). Text: string, height, font, leader, detach, convert to
//             geometry. Notes attached to a shape move with it when its
//             position fields change.
//           - Built-in read-only examples (src/examples.rs) written to redb at
//             every start. Opening one is read-only: any edit is undone and a
//             prompt offers Duplicate (Save As copy, editable). Save / Save As
//             on an example duplicate it. Examples cannot be deleted and their
//             names cannot be used by Save As.
//           - File > New calibration pattern…: power × feed matrix, line
//             density, fill direction, overscan; on the 50 × 100 mm tape or
//             free; creates an unsaved project.
//           - Opening a project fits its shapes in view. Trace Job uses Burn
//             layer shapes only. Job Details counts shapes per layer.
//           - Comparable project JSON (unsaved marker, history) leaves out
//             name, id and read-only as well as the Machine Profile fields.
//           v0.2.2:
//           - Projects split in two left-column panels: Projects (saved
//             projects dropdown, Open, Delete) and Active Project (open
//             project name and state, New / Save / Save As / Duplicate /
//             Close / Export G-code, Job Details, Run History). Each
//             collapses and floats on its own. Active Project is expanded on
//             first launch. Layout key v6 (floating flags reset once).
//           - The Saved projects dropdown returns to "Saved projects…" after
//             Open, so it never shows a name that is not the open project.
//           - Job Details: Engraving and Travel show distance and planned
//             time; Estimated is their sum (src/gcode.rs).
//           - Live runs (Job and Trace, machine and Simulate) pass the run
//             clock and the trajectory duration to the canvas countdown.
// =============================================================================
use eframe::egui::{self,Color32,CornerRadius,Stroke,Vec2};
use eframe::egui::collapsing_header::CollapsingState;
use serde::{Deserialize,Serialize};
use crate::geometry::{self,Align,Cap,Fill,Join,Layer,Operation,Project,Relation,Shape,ShapeKind,StrokeDir,TextFont};
use crate::gcode::{self,Move,Plan};
use crate::storage::Store;
use crate::machine::{Link,Machine};
use crate::canvas::{self,CanvasView,Events,SimOverlay};
use crate::examples::{self,CalKind,CalParams};
use crate::paths;
use crate::runlog::{self,ActiveRun,RunRecord};
use crate::widgets;
use motion_planner::{MotionConfig,Trajectory};
use std::time::Instant;

#[derive(Clone,Copy,Default,Serialize,Deserialize)]
struct Panel { floating:bool }

const TITLES:[&str;9]=["Serial Communications","Projects","Shape Properties","Machine Profile","Jog Controls","G-code / Stream","Runs","Canvas Tools","Active Project"];
const LEFT_DOCK:[usize;5]=[0,1,8,3,6];
const RIGHT_DOCK:[usize;3]=[2,4,5];
const PROPS:usize=2;
const TOOLS:usize=7;
const ACTIVE:usize=8;
const LAYOUT_KEY:&str="panel_layout_v6";
/// Undo steps kept.
const HISTORY:usize=200;
const PROFILE_KEY:&str="machine_profile_v1";

/// Machine Profile: a property of the machine, saved with the app settings.
/// Motion fields ($110/$111, $120/$121, $11) are used when the controller's $$ is not available.
#[derive(Clone,Copy,PartialEq,Serialize,Deserialize)]
#[serde(default)]
struct MachineProfile{work_x:f32,work_y:f32,travel_feed:f32,max_power:u32,
  max_rate_x:f32,max_rate_y:f32,accel_x:f32,accel_y:f32,junction_dev:f32}
impl Default for MachineProfile{fn default()->Self{Self{work_x:380.0,work_y:280.0,travel_feed:2500.0,max_power:1000,
  max_rate_x:3000.0,max_rate_y:3000.0,accel_x:10.0,accel_y:10.0,junction_dev:0.010}}}
/// Planner look-ahead in blocks (GRBL 0.9 block buffer; hypothesis until checked against planner.h).
const LOOKAHEAD:usize=18;
const SIM_SPEEDS:[f32;5]=[1.0,2.0,5.0,10.0,50.0];
const FENCE_EPS:f32=0.001;

/// Trace target in the machine-control row.
#[derive(Clone,Copy,PartialEq)]
enum TraceTarget{Part,Job,Fence}

/// Live view of a Job or Trace run: the planned trajectory of the streamed lines (program
/// coordinates), its moves in canvas coordinates for drawing, and the run clock.
struct LiveView{traj:Trajectory,path:Vec<Move>,origin:(f32,f32),start:(f32,f32),clock:f32,last:Instant,
  progress:usize,head:(f32,f32),laser:bool}

/// Project action that may need an unsaved-changes prompt first.
#[derive(Clone)]
enum Pending{New,Open(String),Close,Load(Box<Project>)}
/// Burn order change from Shape Properties.
#[derive(Clone,Copy)]
enum OrderMove{First,Earlier,Later,Last}

/// Modal dialogs.
enum Dialog{
  None,
  SaveAs{name:String,copy:bool,then:Option<Pending>},
  ConfirmOverwrite{name:String,copy:bool,then:Option<Pending>},
  ConfirmDelete{name:String},
  Unsaved{then:Pending},
  Notice{title:String,msg:String},
  ReadOnly,
  Calibration{params:CalParams,error:String},
}

pub struct LaserApp {
    project:Project, plan:Plan, store:Option<Store>, machine:Machine,
    view:CanvasView, selected:Vec<usize>, props_last_sel:Vec<usize>,
    panels:[Panel;9],
    spot_power:u32,frame_feed:f32,spot_on:bool,trace_target:TraceTarget,monitor_follow:usize,
    max_power:u32, profile:MachineProfile, project_active:bool, saved_once:bool, saved_json:String, dirty:bool, last_title:String,
    dialog:Dialog, saved_names:Vec<String>, project_to_load:String, notice:String, job_lines:Vec<String>,
    runs:Vec<RunRecord>, selected_run:Option<String>, notes_edit:String, active_run:Option<ActiveRun>,
    manual_cmd:String,
    zero_x:bool, zero_y:bool, seen_resets:u32, live:Option<LiveView>, jog_pred:Option<(f32,f32)>, last_jog:Option<Instant>,
    motion:MotionConfig,
    // Undo history: `hist` is the last committed project; `undo` / `redo` hold earlier / undone states.
    undo:Vec<Project>, redo:Vec<Project>, hist:Project, hist_json:String,
    builtin_names:Vec<String>, cal_params:CalParams,
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
    let mut panels=[Panel::default();9];
    if let Some(storage)=cc.storage {if let Some(saved)=eframe::get_value::<[Panel;9]>(storage,LAYOUT_KEY){panels=saved;}}
    // ---- Machine Profile from the user settings (defaults 380 x 280) ----
    let profile=cc.storage.and_then(|s|eframe::get_value::<MachineProfile>(s,PROFILE_KEY)).unwrap_or_default();

    let (store,mut err)=match Store::open(){Ok(s)=>(Some(s),String::new()),Err(e)=>(None,format!("Database unavailable: {e}"))};
    // Built-in examples: written at every start (read-only, fixed ids).
    let builtins=examples::builtin();
    let builtin_names:Vec<String>=builtins.iter().map(|p|p.name.clone()).collect();
    if let Some(s)=&store{for p in &builtins{if let Err(e)=s.save(p){err=format!("Examples not written: {e}");}}}
    let mut project=Project::blank();
    project.work_x=profile.work_x;project.work_y=profile.work_y;project.travel_feed=profile.travel_feed;
    let saved_json=Self::comparable_json(&project);
    let (mut names,mut runs)=(vec![],vec![]);
    if let Some(s)=&store {names=s.names().unwrap_or_default();runs=s.runs().unwrap_or_default();}
    let mut machine=Machine::default();machine.refresh();
    let motion=Self::motion_config(&profile,&machine);
    let plan=gcode::generate(&project,profile.max_power,&motion);
    let hist=project.clone();let hist_json=Self::comparable_json(&project);
    Self {project,plan,store,machine,view:CanvasView::default(),selected:vec![],props_last_sel:vec![],
      panels,
      spot_power:2,frame_feed:1200.0,spot_on:false,trace_target:TraceTarget::Job,monitor_follow:usize::MAX,max_power:profile.max_power,profile,
      project_active:false,saved_once:false,saved_json,dirty:false,last_title:String::new(),
      dialog:Dialog::None,saved_names:names,project_to_load:String::new(),notice:err,job_lines:vec![],
      runs,selected_run:None,notes_edit:String::new(),active_run:None,manual_cmd:String::new(),
      zero_x:false,zero_y:false,seen_resets:0,live:None,jog_pred:None,last_jog:None,motion,
      undo:vec![],redo:vec![],hist,hist_json,builtin_names,cal_params:CalParams::default()}
 }
 /// Planner limits: the controller's $$ while connected, otherwise the Machine Profile.
 fn motion_config(profile:&MachineProfile,machine:&Machine)->MotionConfig{
    let base=MotionConfig{max_rate:[profile.max_rate_x,profile.max_rate_y],accel:[profile.accel_x,profile.accel_y],
      junction_deviation:profile.junction_dev,lookahead:LOOKAHEAD};
    if machine.connected(){base.with_grbl_settings(machine.info.settings.iter().map(|s|s.as_str()))}else{base}
 }
 /// True when the controller's $$ supplies the planner limits.
 fn motion_from_controller(&self)->bool{self.machine.connected()&&!self.machine.info.settings.is_empty()}
 /// Apply the Machine Profile to the open project, then regenerate the program.
 fn regenerate(&mut self){
    self.project.work_x=self.profile.work_x;self.project.work_y=self.profile.work_y;self.project.travel_feed=self.profile.travel_feed;
    self.max_power=self.profile.max_power;
    self.motion=Self::motion_config(&self.profile,&self.machine);
    self.plan=gcode::generate(&self.project,self.max_power,&self.motion);
 }
 /// Project JSON without the Machine Profile fields (those belong to the machine, not the project)
 /// and without name, id and read-only flag (changed only by saving / duplicating).
 fn comparable_json(p:&Project)->String{
    let mut c=p.clone();c.work_x=0.0;c.work_y=0.0;c.travel_feed=0.0;c.name.clear();c.id.clear();c.read_only=false;
    serde_json::to_string(&c).unwrap_or_default()
 }
 // ======================================================================== undo / redo
 fn reset_history(&mut self){self.undo.clear();self.redo.clear();self.hist=self.project.clone();self.hist_json=Self::comparable_json(&self.project);}
 /// Called once per frame after the UI: commit an edit to the history, or undo it on a read-only example.
 fn track_history(&mut self,json:&str){
    if json==self.hist_json||self.view.busy(){return;}
    if self.project.read_only{
      self.project=self.hist.clone();self.selected.retain(|&i|i<self.project.shapes.len());self.regenerate();
      if matches!(self.dialog,Dialog::None){self.dialog=Dialog::ReadOnly;}
      return;
    }
    let prev=std::mem::replace(&mut self.hist,self.project.clone());
    self.undo.push(prev);if self.undo.len()>HISTORY{self.undo.remove(0);}
    self.redo.clear();self.hist_json=json.to_string();
 }
 /// Restore a history state, keeping the open project's name, id and read-only flag.
 fn restore(&mut self,mut p:Project){
    p.name=self.project.name.clone();p.id=self.project.id.clone();p.read_only=self.project.read_only;
    self.project=p;self.hist=self.project.clone();self.hist_json=Self::comparable_json(&self.project);
    self.selected.retain(|&i|i<self.project.shapes.len());self.regenerate();
 }
 fn undo(&mut self){
    if self.view.busy(){return;}
    let Some(p)=self.undo.pop() else {self.notice="Nothing to undo".into();return;};
    self.redo.push(self.hist.clone());self.restore(p);self.notice=format!("Undo ({} more)",self.undo.len());
 }
 fn redo(&mut self){
    if self.view.busy(){return;}
    let Some(p)=self.redo.pop() else {self.notice="Nothing to redo".into();return;};
    self.undo.push(self.hist.clone());self.restore(p);self.notice=format!("Redo ({} more)",self.redo.len());
 }
 /// Requests from the canvas and its tool strip.
 fn handle(&mut self,ev:Events){
    if ev.undo{self.undo();}
    if ev.redo{self.redo();}
    if ev.changed{self.regenerate();}
    if let Some(n)=ev.notice{self.notice=n;}
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
 /// Save button: first save of a project asks for a name. A read-only example is duplicated instead.
 fn request_save(&mut self){
    if !self.project_active {return;}
    if self.project.read_only {self.request_duplicate();return;}
    if self.saved_once {self.save();} else {self.dialog=Dialog::SaveAs{name:self.project.name.clone(),copy:false,then:None};}
 }
 fn request_save_as(&mut self){
    if !self.project_active {return;}
    if self.project.read_only {self.request_duplicate();return;}
    let name=if self.saved_once{format!("{} - Copy",self.project.name)}else{self.project.name.clone()};
    self.dialog=Dialog::SaveAs{name,copy:self.saved_once,then:None};
 }
 /// Duplicate a read-only example into an editable project (asks for the name).
 fn request_duplicate(&mut self){
    let base=self.project.name.trim_start_matches("Example - ").trim_start_matches("Calibration - ").to_string();
    self.dialog=Dialog::SaveAs{name:format!("{base} - Copy"),copy:true,then:None};
 }
 /// Name confirmed in the Save As dialog (overwrite already confirmed if needed).
 fn finish_save_as(&mut self,name:String,copy:bool,then:Option<Pending>){
    if copy {self.project.id=geometry::new_id();self.project.read_only=false;self.reset_history();}
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
        self.selected.clear();self.view.request_fit();self.regenerate();self.mark_clean();self.reset_history();self.notice="New project".into();
      }
      Pending::Close=>{
        self.project=Project::blank();self.project_active=false;self.saved_once=false;
        self.selected.clear();self.regenerate();self.mark_clean();self.reset_history();self.notice="Project closed".into();
      }
      Pending::Load(p)=>{
        // Generated project (calibration pattern): open, not saved yet, marked unsaved.
        self.project=*p;self.project.ensure_uids();self.project_active=true;self.saved_once=false;
        self.selected.clear();self.view.request_fit_shapes();self.regenerate();self.reset_history();
        self.saved_json.clear();self.notice=format!("Generated '{}' (not saved yet)",self.project.name);
      }
      Pending::Open(name)=>{
        let Some(s)=&self.store else {self.notice="Database unavailable".into();return;};
        match s.load(&name){
          Ok(Some(mut p))=>{
            // Projects saved before v0.1.4 have no id: assign one now and store it.
            if p.id.is_empty(){p.id=geometry::new_id();let _=s.save(&p);}
            // Projects saved before v0.2.1: shapes get uids (attachment targets).
            p.ensure_uids();
            self.project=p;self.project_active=true;self.saved_once=true;
            self.selected.clear();self.view.request_fit_shapes();self.regenerate();self.mark_clean();self.reset_history();
            self.notice=if self.project.read_only{format!("Opened '{name}' (read-only example: Duplicate to edit)")}else{format!("Opened '{name}'")};
          }
          Ok(None)=>self.notice=format!("'{name}' not found"),
          Err(e)=>self.notice=format!("Open failed: {e}"),
        }
      }
    }
 }
 fn delete_project(&mut self,name:&str){
    if self.builtin_names.iter().any(|n|n==name){self.notice="Built-in examples cannot be deleted".into();return;}
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
 /// Stream `lines` and open a run record for it. The lines are planned into the trajectory that drives the live view.
 fn begin_run(&mut self,kind:&str,lines:Vec<String>){
    if self.active_run.is_some()||self.machine.running {self.notice="A run is already in progress".into();return;}
    self.job_lines=lines;
    self.machine.start(&self.job_lines);
    if !self.machine.running {return;}
    // Every program starts with M5.
    self.spot_on=false;
    // Plan from where the head is now (work coordinates; job 0,0 when not reported).
    let from=self.machine.info.wpos.map_or([0.0,0.0],|w|[w.0,w.1]);
    let traj=motion_planner::plan(&self.motion,&motion_planner::gcode::blocks_from_lines(&self.job_lines,from));
    let origin=self.project.origin;
    let path:Vec<Move>=traj.segments.iter().map(|s|Move{x:origin.0+s.block.end[0],y:origin.1+s.block.end[1],
      feed:s.block.feed,power:s.block.power,laser:s.block.laser,rapid:s.block.rapid}).collect();
    let start=(origin.0+traj.start[0],origin.1+traj.start[1]);
    self.live=Some(LiveView{traj,path,origin,start,clock:0.0,last:Instant::now(),progress:0,head:start,laser:false});
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
      TraceTarget::Job=>bbox(&mut (0..self.project.shapes.len()).filter(|&i|self.project.shapes[i].layer==Layer::Burn)).ok_or_else(||"Nothing to trace: no shapes on the Burn layer".to_string()),
      TraceTarget::Fence=>{
        if !self.project.fence.on{return Err("Turn the work fence on to trace it".into());}
        let (ox,oy)=self.project.origin;Ok((ox,oy,ox+self.project.fence.w,oy+self.project.fence.h))
      }
    }
 }
 /// Trace program, relative to the origin marker.
 fn trace_program(&self)->Result<Vec<String>,String>{
    let (x0,y0,x1,y1)=self.trace_rect()?;
    let corners=[(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)];
    let warnings=gcode::limit_warnings(&self.project,gcode::relative_span(&corners,self.project.origin),"Trace");
    if let Some(w)=warnings.first(){return Err(w.clone());}
    let (ox,oy)=self.project.origin;
    let c=|v:f32|gcode::coord(v);
    let mut lines=vec!["M5".to_string(),"G21".into(),"G90".into(),format!("G0 X{:.3} Y{:.3}",c(x0-ox),c(y0-oy)),format!("M3 S{}",self.spot_power)];
    for (i,(x,y)) in corners.iter().skip(1).enumerate(){
      lines.push(if i==0{format!("G1 X{:.3} Y{:.3} F{:.1}",c(x-ox),c(y-oy),self.frame_feed)}else{format!("G1 X{:.3} Y{:.3}",c(x-ox),c(y-oy))});
    }
    lines.push("M5".into());
    Ok(lines)
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
 /// Advance the run clock and place the head on the planned trajectory.
 /// Real elapsed time (not the capped frame time), scaled by the Simulate speed; held while the
 /// controller reports Hold, Door or Alarm.
 fn update_live(&mut self){
    let Some(lv)=&mut self.live else {return;};
    let now=Instant::now();
    let elapsed=(now-lv.last).as_secs_f32();lv.last=now;
    let state=&self.machine.state;
    let held=state.starts_with("Hold")||state.starts_with("Door")||state.starts_with("Alarm");
    if !held{lv.clock+=elapsed*if self.machine.simulate{self.machine.sim_speed}else{1.0};}
    let s=lv.traj.sample(lv.clock);
    lv.head=(lv.origin.0+s.pos[0],lv.origin.1+s.pos[1]);
    lv.progress=s.segment;
    lv.laser=s.laser;
 }

 // ======================================================================== panels
 /// Burn-order position (1-based) of shape `i` among the Burn layer shapes, and their count.
 fn burn_position(&self,i:usize)->(usize,usize){
    let burn:Vec<usize>=(0..self.project.shapes.len()).filter(|&k|self.project.shapes[k].layer==Layer::Burn).collect();
    (burn.iter().position(|&k|k==i).map_or(0,|p|p+1),burn.len())
 }
 /// Move shape `i` in the burn order (its place among the Burn layer shapes).
 fn move_order(&mut self,i:usize,to:OrderMove){
    let burn:Vec<usize>=(0..self.project.shapes.len()).filter(|&k|self.project.shapes[k].layer==Layer::Burn).collect();
    let Some(pos)=burn.iter().position(|&k|k==i) else {return;};
    let target=match to{OrderMove::First=>0,OrderMove::Earlier=>pos.saturating_sub(1),OrderMove::Later=>(pos+1).min(burn.len()-1),OrderMove::Last=>burn.len()-1};
    if target==pos{return;}
    let t=burn[target];
    let s=self.project.shapes.remove(i);
    let t_adj=if t>i{t-1}else{t};
    let at=if target>pos{t_adj+1}else{t_adj};
    self.project.shapes.insert(at,s);
    self.selected=vec![at];
    self.regenerate();
 }
 /// Replace burn text by plain geometry: polylines (single-stroke) or a merged-outline region (outline font).
 fn convert_text(&mut self,i:usize){
    let Some(t)=self.project.shapes.get(i).cloned() else {return;};
    let mut parts:Vec<Shape>=vec![];
    let copy=|mut g:Shape|{g.layer=t.layer;g.feed=t.feed;g.power=t.power;g.passes=t.passes;g.lines_per_mm=t.lines_per_mm;g.fill=t.fill;
      g.operation=t.operation;g.stroke_lpm=t.stroke_lpm;g.overscan=t.overscan;g.unidirectional=t.unidirectional;g.border_w=t.border_w;g};
    match t.font{
      TextFont::Stroke=>{for pts in crate::font::stroke_text(&t.text,t.x,t.y,t.h){let mut g=copy(Shape::polyline(pts,false));g.operation=Operation::Outline;parts.push(g);}}
      TextFont::Outline=>{if let Some(r)=paths::region(&t){parts.push(copy(Shape::region(paths::to_polys(&r))));}}
    }
    if parts.is_empty(){self.notice="Nothing to convert".into();return;}
    let n=parts.len();
    self.project.shapes.remove(i);
    for (k,g) in parts.into_iter().enumerate(){self.project.shapes.insert(i+k,g);}
    self.selected=(i..i+n).collect();
    self.notice=format!("Text converted to {n} shape(s)");
    self.regenerate();
 }
 fn shape_properties(&mut self, ui:&mut egui::Ui){
     let amber=Color32::from_rgb(235,190,70);
     if self.project.read_only&&self.project_active{
       ui.horizontal_wrapped(|ui|{
         ui.colored_label(amber,"Read-only example");
         if ui.button("Duplicate to edit").on_hover_text("Save an editable copy under a new name").clicked(){self.request_duplicate();}
       });
     }
     if self.selected.len()>1 {
       ui.weak(format!("{} shapes selected.",self.selected.len()));
       ui.weak("Merge (tool strip) joins closed shapes that share an edge or overlap into one outline. Select one shape to edit it.");
       return;
     }
     let Some(&i)=self.selected.first() else {ui.weak("Select a shape on the canvas.");return;};
     if i>=self.project.shapes.len(){return;}
     let step=self.view.grid_spacing;
     let max_power=self.max_power;
     let (order_pos,order_n)=self.burn_position(i);
     let before=self.project.shapes[i].extent();
     let mut changed=false;
     let mut leader_req=false;
     let mut convert=false;
     let mut order_move:Option<OrderMove>=None;
     let text_id=egui::Id::new(("text_edit",i));
     {
       let s=&mut self.project.shapes[i];
       let kind=match s.kind{
         ShapeKind::Rectangle=>"Rectangle".to_string(),ShapeKind::Circle=>"Circle".to_string(),
         ShapeKind::Line=>if s.width>0.0{"Line trace".into()}else{"Line".into()},
         ShapeKind::Polyline=>if s.closed{"Closed polyline".into()}else if s.width>0.0{"Polyline trace".into()}else{"Polyline".into()},
         ShapeKind::Region=>"Merged outline".to_string(),ShapeKind::Text=>"Text".to_string(),
       };
       ui.horizontal(|ui|{
         ui.strong(format!("{kind} #{}",i+1));
         ui.with_layout(egui::Layout::right_to_left(egui::Align::Center),|ui|{
           egui::ComboBox::from_id_salt("p_layer").width(76.0).selected_text(s.layer.name()).show_ui(ui,|ui|{
             for l in Layer::ALL{changed|=ui.selectable_value(&mut s.layer,l,l.name()).changed();}
           }).response.on_hover_text("Layer: Burn generates G-code; Notes and Guides are shown only");
         });
       });
       // ---------------- geometry
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
             let mut c=false;
             ui.label("Centre X");c|=widgets::num_f32(ui,"p_cx",&mut cx,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Centre Y");c|=widgets::num_f32(ui,"p_cy",&mut cy,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Diameter");c|=widgets::num_f32(ui,"p_d",&mut d,step,0.001,3000.0,3,"mm");ui.end_row();
             if c {s.x=cx-d/2.0;s.y=cy-d/2.0;s.w=d;s.h=d;changed=true;}
           }
           ShapeKind::Polyline|ShapeKind::Region=>{
             let (x0,y0,x1,y1)=s.extent();
             let (mut nx,mut ny)=(x0,y0);
             ui.label("X");let mx=widgets::num_f32(ui,"p_px",&mut nx,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Y");let my=widgets::num_f32(ui,"p_py",&mut ny,step,-3000.0,3000.0,3,"mm");ui.end_row();
             if mx||my{s.translate(nx-x0,ny-y0);changed=true;}
             ui.label("Size");ui.weak(format!("{:.3} × {:.3} mm",x1-x0,y1-y0));ui.end_row();
             if s.kind==ShapeKind::Polyline{
               ui.label("Vertices");ui.weak(format!("{} (drag the handles to move them)",s.points.len()));ui.end_row();
               ui.label("Closed");
               let mut cl=s.closed;
               if ui.add_enabled(s.points.len()>=3,egui::Checkbox::new(&mut cl,"closed loop")).on_hover_text("Closed: an outline with border and interior. Open: a trace.").changed(){
                 s.closed=cl;s.operation=if cl{Operation::Fill}else{Operation::Outline};changed=true;
               }
               ui.end_row();
               let n=s.points.len();
               let l:f32=(0..if s.closed{n}else{n.saturating_sub(1)}).map(|k|{let (a,b)=(s.points[k],s.points[(k+1)%n]);((b.0-a.0).powi(2)+(b.1-a.1).powi(2)).sqrt()}).sum();
               ui.label("Length");ui.weak(format!("{l:.3} mm"));ui.end_row();
             }else{
               let holes:usize=s.polys.iter().map(|p|p.holes.len()).sum();
               ui.label("Outline");ui.weak(format!("{} polygon(s), {} hole(s)",s.polys.len(),holes));ui.end_row();
             }
           }
           ShapeKind::Text=>{
             ui.label("Text");
             let mut t=ui.data_mut(|d|d.get_temp::<String>(text_id)).unwrap_or_else(||s.text.clone());
             let r=ui.add(egui::TextEdit::singleline(&mut t).desired_width(170.0));
             if r.has_focus(){ui.data_mut(|d|d.insert_temp(text_id,t.clone()));}
             if r.lost_focus(){if t!=s.text{s.text=t;changed=true;}ui.data_mut(|d|d.remove::<String>(text_id));}
             ui.end_row();
             ui.label("Height");changed|=widgets::num_f32(ui,"p_th",&mut s.h,0.5,0.5,200.0,2,"mm");ui.end_row();
             ui.label("X");changed|=widgets::num_f32(ui,"p_tx",&mut s.x,step,-3000.0,3000.0,3,"mm");ui.end_row();
             ui.label("Y");changed|=widgets::num_f32(ui,"p_ty",&mut s.y,step,-3000.0,3000.0,3,"mm");ui.end_row();
             if s.layer==Layer::Burn{
               ui.label("Font");
               ui.horizontal(|ui|{
                 changed|=ui.radio_value(&mut s.font,TextFont::Stroke,"Single-stroke").on_hover_text("Centerlines, one pass: small IDs").changed();
                 changed|=ui.radio_value(&mut s.font,TextFont::Outline,"Outline").on_hover_text("Closed glyphs with fill and border: larger text").changed();
               });
               ui.end_row();
             }
             ui.label("Size");ui.weak(format!("{:.2} × {:.2} mm",s.text_width(),s.h));ui.end_row();
           }
         }
       });
       // ---------------- text: leader, attachment, conversion
       if s.kind==ShapeKind::Text{
         ui.horizontal_wrapped(|ui|{
           if ui.button(if s.leader.is_some(){"Move leader"}else{"Add leader"}).on_hover_text("Then click the point it points to; a shape under the click attaches this note to it").clicked(){leader_req=true;}
           if s.leader.is_some()&&ui.button("Remove leader").clicked(){s.leader=None;changed=true;}
           if s.attach.is_some()&&ui.button("Detach").on_hover_text("Stop moving with the attached shape").clicked(){s.attach=None;changed=true;}
           if s.layer==Layer::Burn&&ui.button("Convert to geometry").on_hover_text("Replace the text by polylines (single-stroke) or a merged outline (outline font)").clicked(){convert=true;}
         });
         if s.attach.is_some(){ui.weak("Attached: moves with its shape.");}
       }
       if s.layer!=Layer::Burn{
         ui.weak(format!("{} layer: shown on the canvas, never burned.",s.layer.name()));
       }else{
         let stroke_text=s.kind==ShapeKind::Text&&s.font==TextFont::Stroke;
         // ---------------- trace
         if s.is_trace(){
           ui.separator();ui.strong("Trace");
           egui::Grid::new("trace-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
             ui.label("Width");changed|=widgets::num_f32(ui,"p_tw",&mut s.width,0.1,0.0,100.0,2,"mm");ui.end_row();
             if s.width>0.0{
               ui.label("Direction");ui.horizontal(|ui|{
                 changed|=ui.radio_value(&mut s.stroke_dir,StrokeDir::Along,"Along").on_hover_text("Parallel passes along the path, open ends").changed();
                 changed|=ui.radio_value(&mut s.stroke_dir,StrokeDir::Across,"Across").on_hover_text("Rails across the path").changed();});ui.end_row();
               ui.label("Density");changed|=widgets::num_f32(ui,"p_tlpm",&mut s.stroke_lpm,1.0,0.1,100.0,1,"lines/mm");ui.end_row();
               ui.label(if s.stroke_dir==StrokeDir::Along{"Lines"}else{"Rails"});
               if s.stroke_dir==StrokeDir::Along{ui.weak(format!("{} ({:.2} mm × {:.1} lines/mm)",paths::line_count(s.width,s.stroke_lpm),s.width,s.stroke_lpm));}
               else{ui.weak(format!("every {:.3} mm along the path",1.0/s.stroke_lpm.max(0.1)));}
               ui.end_row();
               ui.label("End cap");ui.horizontal(|ui|{
                 changed|=ui.radio_value(&mut s.cap,Cap::Square,"Square").on_hover_text("Flush with the end points").changed();
                 changed|=ui.radio_value(&mut s.cap,Cap::Round,"Round").on_hover_text("Half circle centred on each end point (adds width/2)").changed();});ui.end_row();
               if s.kind==ShapeKind::Polyline{
                 ui.label("Corners");ui.horizontal(|ui|{
                   changed|=ui.radio_value(&mut s.join,Join::Miter,"Miter").changed();
                   changed|=ui.radio_value(&mut s.join,Join::Round,"Round").changed();});ui.end_row();
               }
             }else{ui.label("");ui.weak("Width 0: one pass on the centerline");ui.end_row();}
           });
         }
         // ---------------- border + interior
         if s.is_closed(){
           ui.separator();ui.strong("Border");
           egui::Grid::new("border-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
             ui.label("Width");changed|=widgets::num_f32(ui,"p_bw",&mut s.border_w,0.1,0.0,100.0,2,"mm");ui.end_row();
             if s.border_w>0.0{
               ui.label("Alignment");ui.horizontal(|ui|{
                 changed|=ui.radio_value(&mut s.border_align,Align::Inside,"Inside").changed();
                 changed|=ui.radio_value(&mut s.border_align,Align::Centered,"Centred").changed();
                 changed|=ui.radio_value(&mut s.border_align,Align::Outside,"Outside").changed();});ui.end_row();
               ui.label("Direction");ui.horizontal(|ui|{
                 changed|=ui.radio_value(&mut s.stroke_dir,StrokeDir::Along,"Along").on_hover_text("Closed contour passes").changed();
                 changed|=ui.radio_value(&mut s.stroke_dir,StrokeDir::Across,"Across").on_hover_text("Rails across the band").changed();});ui.end_row();
               ui.label("Density");changed|=widgets::num_f32(ui,"p_blpm",&mut s.stroke_lpm,1.0,0.1,100.0,1,"lines/mm");ui.end_row();
               if s.stroke_dir==StrokeDir::Along{ui.label("Lines");ui.weak(format!("{} ({:.2} mm × {:.1} lines/mm)",paths::line_count(s.border_w,s.stroke_lpm),s.border_w,s.stroke_lpm));ui.end_row();}
               ui.label("Corners");ui.horizontal(|ui|{
                 changed|=ui.radio_value(&mut s.join,Join::Miter,"Miter").changed();
                 changed|=ui.radio_value(&mut s.join,Join::Round,"Round").changed();});ui.end_row();
             }else{ui.label("");ui.weak("Width 0: no border");ui.end_row();}
           });
           ui.separator();ui.strong("Interior");
           egui::Grid::new("interior-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
             let mut outline=matches!(s.operation,Operation::Outline|Operation::Both);
             let mut filled=matches!(s.operation,Operation::Fill|Operation::Both);
             ui.label("Outline pass");
             if ui.checkbox(&mut outline,"one pass on the drawn edge").changed(){changed=true;}
             ui.end_row();
             ui.label("Fill");
             let name=|f:Fill|match f{Fill::Horizontal=>"Horizontal",Fill::Vertical=>"Vertical",Fill::Crosshatch=>"Crosshatch",Fill::Concentric=>"Concentric"};
             egui::ComboBox::from_id_salt("p_fill").width(110.0).selected_text(if filled{name(s.fill)}else{"None"}).show_ui(ui,|ui|{
               if ui.selectable_label(!filled,"None").clicked(){filled=false;changed=true;}
               for f in [Fill::Horizontal,Fill::Vertical,Fill::Crosshatch,Fill::Concentric]{
                 if ui.selectable_label(filled&&s.fill==f,name(f)).clicked(){filled=true;s.fill=f;changed=true;}
               }
             });
             ui.end_row();
             s.operation=match (outline,filled){(true,true)=>Operation::Both,(true,false)=>Operation::Outline,(false,true)=>Operation::Fill,(false,false)=>Operation::None};
             if filled{
               ui.label("Density");changed|=widgets::num_f32(ui,"p_lpm",&mut s.lines_per_mm,1.0,0.1,100.0,1,"lines/mm");ui.end_row();
               ui.label("Spacing");ui.weak(format!("{:.4} mm",1.0/s.lines_per_mm.max(0.1)));ui.end_row();
               if s.border_w>0.0{
                 ui.label("Relation");ui.horizontal(|ui|{
                   changed|=ui.radio_value(&mut s.relation,Relation::Abut,"Abut").on_hover_text("Interior starts at the border's inner edge").changed();
                   changed|=ui.radio_value(&mut s.relation,Relation::Overlap,"Overlap").on_hover_text("Interior extends into the border").changed();
                   changed|=ui.radio_value(&mut s.relation,Relation::Gap,"Gap").on_hover_text("Unburned ring between border and interior").changed();});ui.end_row();
                 if s.relation!=Relation::Abut{ui.label(if s.relation==Relation::Overlap{"Overlap"}else{"Gap"});changed|=widgets::num_f32(ui,"p_rel",&mut s.relation_mm,0.1,0.0,100.0,2,"mm");ui.end_row();}
               }
             }
           });
         }
         // ---------------- raster
         if !stroke_text{
           ui.separator();ui.strong("Raster");
           egui::Grid::new("raster-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
             ui.label("Scan");ui.horizontal(|ui|{
               let mut uni=s.unidirectional;
               changed|=ui.radio_value(&mut uni,false,"Bidirectional").changed();
               changed|=ui.radio_value(&mut uni,true,"Unidirectional").on_hover_text("Every pass in the same direction (travel back between passes)").changed();
               s.unidirectional=uni;});ui.end_row();
             ui.label("Overscan");changed|=widgets::num_f32(ui,"p_ovs",&mut s.overscan,0.5,0.0,50.0,2,"mm");ui.end_row();
           });
           if s.overscan>0.0{ui.small("Laser-off lead-in / lead-out at the burn feed on every open pass. GRBL 0.9 stops at each M3/M5 (planner model), so the pass ends may still burn heavier; see README.");}
         }
       }
       // ---------------- process
       if s.layer==Layer::Burn{
         ui.separator();ui.strong("Process");
         egui::Grid::new("process-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
           ui.label("Feed");changed|=widgets::num_f32(ui,"p_feed",&mut s.feed,50.0,1.0,30000.0,0,"mm/min");ui.end_row();
           ui.label("Power");changed|=widgets::num_u32(ui,"p_pow",&mut s.power,10,0,max_power,&format!("/ {max_power}"));ui.end_row();
           ui.label("Passes");changed|=widgets::num_u32(ui,"p_pass",&mut s.passes,1,1,100,"");ui.end_row();
         });
         ui.separator();
         ui.horizontal_wrapped(|ui|{
           ui.strong("Burn order");ui.label(format!("{order_pos} of {order_n}"));
           if ui.small_button("First").clicked(){order_move=Some(OrderMove::First);}
           if ui.small_button("Earlier").clicked(){order_move=Some(OrderMove::Earlier);}
           if ui.small_button("Later").clicked(){order_move=Some(OrderMove::Later);}
           if ui.small_button("Last").clicked(){order_move=Some(OrderMove::Last);}
         });
       }
       if s.kind==ShapeKind::Polyline{s.sync_box();}
     }
     if changed {
       // Notes attached to this shape follow a pure move (same size, new position).
       let after=self.project.shapes[i].extent();
       let (dx,dy)=(after.0-before.0,after.1-before.1);
       let same=((after.2-after.0)-(before.2-before.0)).abs()<1e-4&&((after.3-after.1)-(before.3-before.1)).abs()<1e-4;
       if same&&(dx.abs()>1e-6||dy.abs()>1e-6)&&self.project.shapes[i].kind!=ShapeKind::Text{
         for f in canvas::followers(&self.project,&[i]){self.project.shapes[f].translate(dx,dy);}
       }
       self.regenerate();
     }
     if leader_req{self.view.begin_leader(i);self.notice="Click the point the leader points to".into();}
     if convert{self.convert_text(i);}
     if let Some(m)=order_move{self.move_order(i,m);}
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
       for name in &self.saved_names {
         let label=if self.builtin_names.contains(name){format!("{name}  · read-only")}else{name.clone()};
         ui.selectable_value(&mut self.project_to_load,name.clone(),label);
       }
     });
   let has_pick=!self.project_to_load.is_empty();
   let pick_builtin=self.builtin_names.contains(&self.project_to_load);
   ui.horizontal(|ui|{
     if ui.add_enabled(has_pick,egui::Button::new("Open").min_size(Vec2::new(80.0,26.0))).on_hover_text("Open the selected project").clicked(){
       // The dropdown is a browser only: cleared once the project is opened.
       let name=std::mem::take(&mut self.project_to_load);self.request(Pending::Open(name));
     }
     if ui.add_enabled(has_pick&&!pick_builtin,egui::Button::new("Delete").min_size(Vec2::new(80.0,26.0))).on_hover_text("Delete the selected project (asks first)").on_disabled_hover_text("Built-in examples cannot be deleted").clicked(){
       self.dialog=Dialog::ConfirmDelete{name:self.project_to_load.clone()};
     }
   });
 }
 /// Active Project panel: the open project, its actions, Job Details and Run History.
 fn active_panel(&mut self,ui:&mut egui::Ui){
   if self.project_active {
     ui.horizontal(|ui|{
       ui.label(&self.project.name);
       if self.dirty {ui.colored_label(Color32::from_rgb(235,190,70),"*").on_hover_text("Unsaved changes");}
       if !self.saved_once {ui.weak("(not saved yet)");}
       if self.project.read_only {ui.colored_label(Color32::from_rgb(235,190,70),"read-only");}
     });
   } else {ui.weak("No project open.");}
   let active=self.project_active;
   let ro=self.project.read_only;
   ui.horizontal_wrapped(|ui|{
     if ui.button("New").clicked(){self.request(Pending::New);}
     if ro {
       if ui.add_enabled(active,egui::Button::new("Duplicate")).on_hover_text("Save an editable copy of this example").clicked(){self.request_duplicate();}
     } else {
       if ui.add_enabled(active,egui::Button::new("Save")).clicked(){self.request_save();}
       if ui.add_enabled(active,egui::Button::new("Save As")).clicked(){self.request_save_as();}
     }
     if ui.add_enabled(active,egui::Button::new("Close")).clicked(){self.request(Pending::Close);}
     if ui.add_enabled(active&&!self.plan.moves.is_empty(),egui::Button::new("Export G-code")).on_hover_text("Write the program to ~/LaserCNC/exports").clicked(){self.export_gcode();}
   });
   if !self.project_active {return;}
   // ---- job details ----
   ui.separator();ui.strong("Job Details");
   // Whole seconds per part, so the shown parts add up exactly to the shown total.
   let (eng_s,trav_s)=(self.plan.engrave_seconds.round(),self.plan.travel_seconds.round());
   ui.label(format!("Estimated: {}",Self::duration_hms(eng_s+trav_s))).on_hover_text("Engraving time + Travel time");
   ui.small(if self.motion_from_controller(){"Planned: acceleration, corners, M3/M5 stops (controller $$)"}else{"Planned: acceleration, corners, M3/M5 stops (Machine Profile)"});
   let count=|l:Layer|self.project.shapes.iter().filter(|s|s.layer==l).count();
   ui.label(format!("Shapes: burn {} · notes {} · guides {}",count(Layer::Burn),count(Layer::Notes),count(Layer::Guides)));
   let burn:Vec<&Shape>=self.project.shapes.iter().filter(|s|s.layer==Layer::Burn).collect();
   if !burn.is_empty(){
      let (mut x0,mut y0,mut x1,mut y1)=(f32::MAX,f32::MAX,f32::MIN,f32::MIN);
      for s in &burn{let e=s.extent();x0=x0.min(e.0);y0=y0.min(e.1);x1=x1.max(e.2);y1=y1.max(e.3);}
      ui.label(format!("Burn extent: {:.1} × {:.1} mm",x1-x0,y1-y0));
      let shape=burn[0];
      ui.label(format!("First shape: S{} · F{:.0} · {} pass(es)",shape.power,shape.feed,shape.passes));
      if shape.is_closed(){ui.label(format!("Fill: {:?} | {:?} · {:.1} lines/mm",shape.operation,shape.fill,shape.lines_per_mm));}
      if shape.is_trace(){ui.label(format!("Trace: {:.2} mm {:?} · {:.1} lines/mm",shape.width,shape.stroke_dir,shape.stroke_lpm));}
   }
   ui.label(format!("Engraving: {:.1} mm · {}",self.plan.engraving_mm,Self::duration_hms(eng_s))).on_hover_text("Laser on: distance and planned time");
   ui.label(format!("Travel: {:.1} mm · {}",self.plan.travel_mm,Self::duration_hms(trav_s))).on_hover_text("Laser off (G0 travel and overscan leads): distance and planned time, including the stops at M3/M5");
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
     ACTIVE=>{self.active_panel(ui);},
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
       ui.separator();
       ui.strong("Motion");
       if self.motion_from_controller(){ui.colored_label(Color32::from_rgb(110,210,130),"In use: controller $$ (values below apply when disconnected)");}
       else{ui.weak("In use: these values (controller $$ is used while connected)");}
       egui::Grid::new("machine-motion-grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
         ui.label("Max rate X ($110)");changed|=widgets::num_f32(ui,"mp_rx",&mut self.profile.max_rate_x,100.0,1.0,100000.0,0,"mm/min");ui.end_row();
         ui.label("Max rate Y ($111)");changed|=widgets::num_f32(ui,"mp_ry",&mut self.profile.max_rate_y,100.0,1.0,100000.0,0,"mm/min");ui.end_row();
         ui.label("Accel X ($120)");changed|=widgets::num_f32(ui,"mp_ax",&mut self.profile.accel_x,10.0,0.1,100000.0,1,"mm/s²");ui.end_row();
         ui.label("Accel Y ($121)");changed|=widgets::num_f32(ui,"mp_ay",&mut self.profile.accel_y,10.0,0.1,100000.0,1,"mm/s²");ui.end_row();
         ui.label("Junction dev ($11)");changed|=widgets::num_f32(ui,"mp_jd",&mut self.profile.junction_dev,0.001,0.001,10.0,3,"mm");ui.end_row();
       });
       let m=self.motion;
       ui.weak(format!("Planner: {:.0} / {:.0} mm/min · {:.1} / {:.1} mm/s² · {:.3} mm · {} blocks",m.max_rate[0],m.max_rate[1],m.accel[0],m.accel[1],m.junction_deviation,m.lookahead));
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
   let state=CollapsingState::load_with_default_open(ui.ctx(),egui::Id::new(("panel",TITLES[id])),id==0||id==ACTIVE);
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
       let (min_w,min_h)=if id==TOOLS{(60.0,60.0)}else{(240.0,160.0)};
       win.resizable(true).collapsible(true).min_width(min_w).min_height(min_h)
         .default_size(match id{5=>Vec2::new(380.0,620.0),PROPS=>Vec2::new(300.0,520.0),TOOLS=>Vec2::new(560.0,130.0),_=>Vec2::new(320.0,360.0)})
         .default_pos(if id==PROPS{egui::Pos2::new(330.0,190.0)}else if id==TOOLS{egui::Pos2::new(330.0,140.0)}else{egui::Pos2::new(340.0+id as f32*24.0,150.0+id as f32*24.0)})
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
           else if id==TOOLS {self.canvas_tools(ui,false);}
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
       if ui.button("New calibration pattern…").on_hover_text("Power × feed, line density, fill direction or overscan test pattern").clicked(){
         self.dialog=Dialog::Calibration{params:self.cal_params.clone(),error:String::new()};ui.close_menu();
       }
       if ui.button("Open…").on_hover_text("Choose a project in the Projects panel").clicked(){self.panels[1].floating=false;self.show_panel(ui.ctx(),1);ui.close_menu();}
       ui.separator();
       if self.project.read_only{
         if ui.add_enabled(active,egui::Button::new("Duplicate…")).on_hover_text("Save an editable copy of this example").clicked(){self.request_duplicate();ui.close_menu();}
       }else{
         if ui.add_enabled(active,egui::Button::new("Save")).clicked(){self.request_save();ui.close_menu();}
         if ui.add_enabled(active,egui::Button::new("Save As…")).clicked(){self.request_save_as();ui.close_menu();}
       }
       if ui.add_enabled(active,egui::Button::new("Close")).clicked(){self.request(Pending::Close);ui.close_menu();}
       ui.separator();
       if ui.button("Delete…").on_hover_text("Choose a project in the Projects panel").clicked(){self.panels[1].floating=false;self.show_panel(ui.ctx(),1);ui.close_menu();}
       if ui.add_enabled(active&&!self.plan.moves.is_empty(),egui::Button::new("Export G-code")).clicked(){self.export_gcode();ui.close_menu();}
     });
     ui.menu_button("Edit",|ui|{
       if ui.add_enabled(!self.undo.is_empty(),egui::Button::new(format!("Undo  ({})",self.undo.len()))).on_hover_text("Ctrl+Z").clicked(){self.undo();ui.close_menu();}
       if ui.add_enabled(!self.redo.is_empty(),egui::Button::new(format!("Redo  ({})",self.redo.len()))).on_hover_text("Ctrl+Y / Ctrl+Shift+Z").clicked(){self.redo();ui.close_menu();}
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
       match self.trace_program(){Ok(lines)=>self.begin_run("Trace",lines),Err(e)=>self.notice=e}
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
       if !self.plan.moves.is_empty()&&self.plan.warnings.is_empty(){let lines=self.plan.lines.clone();self.begin_run("Job",lines);}
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
 /// Canvas tool strip (docked: two rows; floating: wraps to the window width).
 fn canvas_tools(&mut self,ui:&mut egui::Ui,docked:bool){
   let (cu,cr)=(!self.undo.is_empty()&&!self.project.read_only,!self.redo.is_empty()&&!self.project.read_only);
   let ev=self.view.toolbar(ui,&mut self.project,&mut self.selected,docked,cu,cr);
   self.handle(ev);
 }
 fn canvas(&mut self,ui:&mut egui::Ui){
   if !self.panels[TOOLS].floating{
     // Docked above the canvas; the arrow collapses it to its title bar. Open by default.
     let state=CollapsingState::load_with_default_open(ui.ctx(),egui::Id::new(("panel",TITLES[TOOLS])),true);
     let _=state.show_header(ui,|ui|self.panel_header(ui,TOOLS)).body_unindented(|ui|{self.canvas_tools(ui,true);});
   }
   ui.add_space(2.0);
   let sim=match &self.live{
     // Live run: its planned path; head, progress and laser colour from the run clock.
     Some(lv)=>{let total=lv.traj.duration();
       SimOverlay{active:true,index:lv.progress,head:lv.head,start:lv.start,path:Some(lv.path.as_slice()),laser:lv.laser,
         timer:Some((lv.clock.min(total),total,self.machine.state.starts_with("Hold")||self.machine.state.starts_with("Door")))}}
     None=>SimOverlay{active:false,index:0,head:self.plan.origin,start:self.plan.origin,path:None,laser:false,timer:None},
   };
   let ev=self.view.show(ui,&mut self.project,&mut self.selected,&self.plan,&sim);
   self.handle(ev);
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
         let reserved=self.builtin_names.iter().any(|n|n==name.trim());
         if reserved{ui.colored_label(Color32::from_rgb(240,95,95),"That name belongs to a built-in example. Choose another.");}
         ui.horizontal(|ui|{
           let ok=!name.trim().is_empty()&&!reserved;
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
     Dialog::ReadOnly=>{
       let mut next=None;
       modal("Read-only Example").show(ctx,|ui|{
         ui.set_max_width(380.0);
         ui.label(format!("'{}' is a built-in example and cannot be changed. The edit was undone.",self.project.name));
         ui.label("Duplicate it to get an editable copy?");
         ui.horizontal(|ui|{
           if ui.button("Duplicate").clicked(){
             let base=self.project.name.trim_start_matches("Example - ").trim_start_matches("Calibration - ").to_string();
             next=Some(Dialog::SaveAs{name:format!("{base} - Copy"),copy:true,then:None});
           }
           if ui.button("Cancel").clicked()||ui.input(|i|i.key_pressed(egui::Key::Escape)){next=Some(Dialog::None);}
         });
       });
       next.unwrap_or(Dialog::ReadOnly)
     }
     Dialog::Calibration{mut params,mut error}=>{
       let mut next=None;
       modal("New Calibration Pattern").show(ctx,|ui|{
         ui.set_max_width(420.0);
         egui::ComboBox::from_id_salt("cal_kind").width(200.0).selected_text(params.kind.name()).show_ui(ui,|ui|{
           for k in [CalKind::PowerFeed,CalKind::Density,CalKind::Direction,CalKind::Overscan]{ui.selectable_value(&mut params.kind,k,k.name());}
         });
         let max=self.max_power;
         egui::Grid::new("cal_grid").num_columns(2).spacing(Vec2::new(10.0,6.0)).show(ui,|ui|{
           let p=&mut params;
           match p.kind{
             CalKind::PowerFeed=>{
               ui.label("Power from");widgets::num_u32(ui,"cal_p0",&mut p.power_from,10,0,max,&format!("/ {max}"));ui.end_row();
               ui.label("Power to");widgets::num_u32(ui,"cal_p1",&mut p.power_to,10,0,max,&format!("/ {max}"));ui.end_row();
               ui.label("Power steps (rows)");widgets::num_u32(ui,"cal_pn",&mut p.power_steps,1,1,30,"");ui.end_row();
               ui.label("Feed from");widgets::num_f32(ui,"cal_f0",&mut p.feed_from,100.0,1.0,30000.0,0,"mm/min");ui.end_row();
               ui.label("Feed to");widgets::num_f32(ui,"cal_f1",&mut p.feed_to,100.0,1.0,30000.0,0,"mm/min");ui.end_row();
               ui.label("Feed steps (columns)");widgets::num_u32(ui,"cal_fn",&mut p.feed_steps,1,1,30,"");ui.end_row();
               ui.label("Density");widgets::num_f32(ui,"cal_lpm",&mut p.lpm,1.0,0.1,100.0,1,"lines/mm");ui.end_row();
             }
             CalKind::Density=>{
               ui.label("Density from");widgets::num_f32(ui,"cal_l0",&mut p.lpm_from,1.0,0.1,100.0,1,"lines/mm");ui.end_row();
               ui.label("Density to");widgets::num_f32(ui,"cal_l1",&mut p.lpm_to,1.0,0.1,100.0,1,"lines/mm");ui.end_row();
               ui.label("Steps (rows)");widgets::num_u32(ui,"cal_ln",&mut p.lpm_steps,1,1,30,"");ui.end_row();
             }
             CalKind::Overscan=>{
               ui.label("Overscan up to");widgets::num_f32(ui,"cal_o1",&mut p.overscan_max,0.5,0.0,50.0,2,"mm");ui.end_row();
               ui.label("Steps (rows, from 0)");widgets::num_u32(ui,"cal_on",&mut p.overscan_steps,1,1,30,"");ui.end_row();
             }
             CalKind::Direction=>{ui.label("Rows");ui.weak("horizontal, vertical, crosshatch, concentric, trace along, trace across");ui.end_row();}
           }
           if p.kind!=CalKind::PowerFeed{
             ui.label("Power");widgets::num_u32(ui,"cal_pw",&mut p.power,10,0,max,&format!("/ {max}"));ui.end_row();
             ui.label("Feed");widgets::num_f32(ui,"cal_fd",&mut p.feed,100.0,1.0,30000.0,0,"mm/min");ui.end_row();
             if p.kind!=CalKind::Density{ui.label("Density");widgets::num_f32(ui,"cal_lp",&mut p.lpm,1.0,0.1,100.0,1,"lines/mm");ui.end_row();}
           }
           ui.label("Patch width");widgets::num_f32(ui,"cal_w",&mut p.patch_w,1.0,0.5,200.0,1,"mm");ui.end_row();
           ui.label("Patch height");widgets::num_f32(ui,"cal_h",&mut p.patch_h,1.0,0.5,200.0,1,"mm");ui.end_row();
           ui.label("Gap");widgets::num_f32(ui,"cal_g",&mut p.gap,0.5,0.0,100.0,1,"mm");ui.end_row();
           ui.label("Placement");ui.checkbox(&mut p.on_tape,"on the 50 × 100 mm tape (5 mm clearance)");ui.end_row();
         });
         ui.weak("Labels go on the Notes layer. The project opens unsaved and editable.");
         if !error.is_empty(){ui.colored_label(Color32::from_rgb(240,95,95),&error);}
         ui.horizontal(|ui|{
           if ui.button("Create").clicked(){
             match examples::calibration(&params){
               Ok(p)=>{self.cal_params=params.clone();self.request(Pending::Load(Box::new(p)));next=Some(Dialog::None);}
               Err(e)=>error=e,
             }
           }
           if ui.button("Cancel").clicked(){self.cal_params=params.clone();next=Some(Dialog::None);}
         });
       });
       // A Create that went through the unsaved-changes prompt leaves that prompt in self.dialog.
       match next{Some(Dialog::None) if !matches!(self.dialog,Dialog::None)=>std::mem::replace(&mut self.dialog,Dialog::None),Some(d)=>d,None=>Dialog::Calibration{params,error}}
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

impl eframe::App for LaserApp {
 fn update(&mut self,ctx:&egui::Context,_frame:&mut eframe::Frame){
   self.machine.poll();
   // Re-plan when the motion limits change (connect / $$ received / disconnect / Machine Profile).
   if Self::motion_config(&self.profile,&self.machine)!=self.motion{self.regenerate();}
   // A controller reset (banner) or disconnect invalidates the work zero and the spot state.
   if self.machine.reset_count!=self.seen_resets||!self.machine.connected(){
     self.seen_resets=self.machine.reset_count;self.zero_x=false;self.zero_y=false;self.spot_on=false;self.jog_pred=None;
   }
   if self.machine.state.starts_with("Alarm"){self.spot_on=false;}
   self.drain_journal();
   self.track_run();
   self.update_live();
   // Undo / redo keys (not while a text field has focus).
   if matches!(self.dialog,Dialog::None)&&ctx.memory(|m|m.focused().is_none()){
     let (z,y,shift,cmd)=ctx.input(|i|(i.key_pressed(egui::Key::Z),i.key_pressed(egui::Key::Y),i.modifiers.shift,i.modifiers.command));
     if cmd&&z&&!shift{self.undo();}else if cmd&&(y||(z&&shift)){self.redo();}
   }
   egui::TopBottomPanel::top("top_bar").show(ctx,|ui|self.toolbar(ui));
   self.panels_layout(ctx);
   egui::CentralPanel::default().show(ctx,|ui|{
     for w in &self.plan.warnings {ui.colored_label(Color32::LIGHT_RED,w);}
     if let Some(err)=&self.machine.error{ui.colored_label(Color32::LIGHT_RED,err);}
     if !self.notice.is_empty(){ui.label(&self.notice);}
     self.canvas(ui);
   });
   self.dialogs(ctx);
   // After the UI: commit this frame's edit to the history (or undo it on a read-only example),
   // then the unsaved marker and window title.
   let json=Self::comparable_json(&self.project);
   self.track_history(&json);
   let json=if json==self.hist_json{json}else{Self::comparable_json(&self.project)};
   // Drawing on an empty canvas starts an (unsaved) project.
   if !self.project_active&&!self.project.shapes.is_empty(){self.project_active=true;self.saved_once=false;}
   self.dirty=self.project_active&&json!=self.saved_json;
   let title=format!("Laser CNC Controller v{}{}",env!("CARGO_PKG_VERSION"),
     if self.project_active{format!(" — {}{}{}",self.project.name,if self.project.read_only{" (read-only)"}else{""},if self.dirty{" *"}else{""})}else{String::new()});
   if title!=self.last_title {ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));self.last_title=title;}
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
// Created:     2026-10-17 18:43:10 UTC
// =============================================================================
