// =============================================================================
// File:     runlog.rs
// Location: laser-cnc-controller/src/runlog.rs
// Version:  v0.1.3
// Comments: Burn records, daily machine log and G-code export.
//           Data folder (visible): ~/LaserCNC  (Windows: %USERPROFILE%\LaserCNC)
//             override with LASER_CNC_HOME=/path
//             exports/  <project>_<YYYY-MM-DD_HHMM>.gcode   (Export G-code)
//             runs/     <project>_<YYYY-MM-DD_HHMMSS>/       (one per Job/Trace run)
//                         program.gcode  exact lines streamed
//                         run.json       run record (outcome, timing, events, settings, notes)
//                         project.json   project snapshot at run start
//                         serial.log     TX/RX for this run
//                         images/        reserved for inspection captures
//             logs/     YYYY-MM-DD.log  every command sent + controller messages
//           v0.1.1: run kind "Trace" replaces "Frame" (folder suffix _trace);
//           older "Frame" records keep their names.
//           v0.1.2: `simulated` flag (Simulate mode runs, tagged SIM; folder
//           suffix _sim).
//           v0.1.3: estimate_s is the planned time (acceleration, corners,
//           M3/M5 stops) from app v0.2.0; comment only, no format change.
// =============================================================================
use chrono::Local;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;
use crate::geometry::Project;

pub fn data_root() -> PathBuf {
    if let Some(p) = std::env::var_os("LASER_CNC_HOME") { return PathBuf::from(p); }
    let home = std::env::var_os(if cfg!(windows) { "USERPROFILE" } else { "HOME" }).map(PathBuf::from).unwrap_or_else(|| PathBuf::from("."));
    home.join("LaserCNC")
}
pub fn exports_dir() -> PathBuf { data_root().join("exports") }
pub fn runs_dir() -> PathBuf { data_root().join("runs") }
pub fn logs_dir() -> PathBuf { data_root().join("logs") }

pub fn now_text() -> String { Local::now().format("%Y-%m-%d %H:%M:%S").to_string() }
pub fn clock() -> String { Local::now().format("%H:%M:%S%.3f").to_string() }

/// File-system-safe version of a project name.
pub fn safe_name(s: &str) -> String {
    let t: String = s.trim().chars().map(|c| if c.is_alphanumeric() || c == ' ' || c == '-' || c == '_' { c } else { '_' }).collect();
    if t.is_empty() { "Untitled".into() } else { t }
}

/// Write the program to exports/<project>_<YYYY-MM-DD_HHMM>.gcode.
pub fn export_gcode(project_name: &str, lines: &[String]) -> Result<PathBuf, String> {
    let dir = exports_dir();
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let base = format!("{}_{}", safe_name(project_name), Local::now().format("%Y-%m-%d_%H%M"));
    let mut path = dir.join(format!("{base}.gcode"));
    let mut n = 2;
    while path.exists() { path = dir.join(format!("{base}_{n}.gcode")); n += 1; }
    let mut text = lines.join("\n");
    text.push('\n');
    std::fs::write(&path, text).map_err(|e| e.to_string())?;
    Ok(path)
}

/// Append already-timestamped lines to logs/<today>.log.
pub fn append_daily(lines: &[String]) -> Result<(), String> {
    if lines.is_empty() { return Ok(()); }
    let dir = logs_dir();
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let path = dir.join(format!("{}.log", Local::now().format("%Y-%m-%d")));
    let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).map_err(|e| e.to_string())?;
    for l in lines { writeln!(f, "{l}").map_err(|e| e.to_string())?; }
    Ok(())
}

/// Open a folder in the system file manager.
pub fn open_folder(path: &Path) {
    let _ = std::fs::create_dir_all(path);
    let program = if cfg!(windows) { "explorer" } else if cfg!(target_os = "macos") { "open" } else { "xdg-open" };
    let _ = std::process::Command::new(program).arg(path).spawn();
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RunEvent { pub time: String, pub event: String }

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct RunRecord {
    pub id: String,
    pub folder: String,
    pub kind: String,            // "Job" | "Trace" (older records: "Frame")
    pub project_id: String,
    pub project_name: String,
    pub started: String,         // local "YYYY-MM-DD HH:MM:SS"
    pub ended: String,
    pub duration_s: f32,
    pub estimate_s: f32,         // planned time (Job only); v0.1.x records: constant-speed lower bound
    pub outcome: String,         // Running | Completed | Reset | Error | Disconnected
    pub error: Option<String>,
    pub error_line: Option<usize>, // 1-based program line rejected by GRBL
    pub lines_total: usize,
    pub lines_acked: usize,
    pub events: Vec<RunEvent>,   // pauses, resumes, controller state changes
    pub port: String,
    pub baud: u32,
    pub firmware: Option<String>,
    pub app_version: String,
    pub max_power: u32,
    pub spot_power: u32,
    pub frame_feed: f32,         // trace feed (field name kept for older records)
    pub settings: Option<Project>, // geometry and per-shape process settings at run start
    pub notes: String,
    pub simulated: bool,         // Simulate mode run (virtual controller), shown as SIM
}

/// Rewrite run.json in the run folder (used at start, end and on notes edits).
pub fn write_run_json(r: &RunRecord) -> Result<(), String> {
    let path = PathBuf::from(&r.folder).join("run.json");
    std::fs::write(path, serde_json::to_vec_pretty(r).map_err(|e| e.to_string())?).map_err(|e| e.to_string())
}

/// A run in progress: folder already created, record finalised by finish().
pub struct ActiveRun {
    pub record: RunRecord,
    pub started: Instant,
    pub serial: Vec<String>,
    pub finishing_since: Option<Instant>,
    pub reset_requested: bool,
    pub last_state: String,
}
impl ActiveRun {
    /// Create runs/<project>_<stamp>/ with program.gcode, project.json, images/ and a provisional run.json.
    pub fn begin(mut record: RunRecord, program: &[String], project: &Project) -> Result<Self, String> {
        let stamp = Local::now().format("%Y-%m-%d_%H%M%S").to_string();
        let mut id = format!("{}_{}", safe_name(&record.project_name), stamp);
        if record.kind == "Trace" { id.push_str("_trace"); }
        if record.simulated { id.push_str("_sim"); }
        let folder = runs_dir().join(&id);
        std::fs::create_dir_all(folder.join("images")).map_err(|e| e.to_string())?;
        let mut prog = program.join("\n"); prog.push('\n');
        std::fs::write(folder.join("program.gcode"), prog).map_err(|e| e.to_string())?;
        std::fs::write(folder.join("project.json"), serde_json::to_vec_pretty(project).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
        record.id = id;
        record.folder = folder.to_string_lossy().to_string();
        record.started = now_text();
        record.outcome = "Running".into();
        record.lines_total = program.len();
        record.settings = Some(project.clone());
        write_run_json(&record)?;
        Ok(Self { record, started: Instant::now(), serial: vec![], finishing_since: None, reset_requested: false, last_state: String::new() })
    }
    pub fn event(&mut self, e: &str) { self.record.events.push(RunEvent { time: now_text(), event: e.to_string() }); }
    /// Finalise: outcome, timing, serial.log and run.json.
    pub fn finish(mut self, outcome: &str, error: Option<String>, error_line: Option<usize>, acked: usize) -> RunRecord {
        self.record.ended = now_text();
        self.record.duration_s = self.started.elapsed().as_secs_f32();
        self.record.outcome = outcome.to_string();
        self.record.error = error;
        self.record.error_line = error_line.map(|i| i + 1);
        self.record.lines_acked = acked;
        let mut log = self.serial.join("\n"); log.push('\n');
        let _ = std::fs::write(PathBuf::from(&self.record.folder).join("serial.log"), log);
        let _ = write_run_json(&self.record);
        self.record
    }
}
// =============================================================================
// End of file: runlog.rs
// Location:    laser-cnc-controller/src/runlog.rs
// Created:     2026-10-16 21:27:22 UTC
// =============================================================================
