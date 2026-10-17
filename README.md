<!--
=============================================================================
File:     README.md
Location: laser-cnc-controller/README.md
Version:  v0.2.2
Comments: Project overview, build, controls and limitations.
          v0.1.4: project management, run records, daily log, Runs panel,
          Shape Properties dock slot in the canvas tool strip.
          v0.1.5: threaded LaserGRBL-style buffered streaming, connection
          handshake and controller info, Unlock ($X), manual command line,
          no fixed UI scale.
          v0.1.6: job origin marker, work fence, Trace (Part / Job / Fence),
          live run view, G92 zeroing, jog step field, Spot ON/OFF toggle,
          transparent floating G-code / Stream window.
          v0.1.7: Simulate mode (virtual GRBL 0.9i) replaces the Simulation
          panel; SIM-tagged run records; head marker red (laser on) /
          yellow (off).
          v0.1.8: Shape Properties docks in the right column; jog steps
          0.1 / 10 mm + typed; stream count fix.
          v0.1.9: Machine Profile saved as a user setting (default
          380 x 280 mm); default grid 5 mm.
          v0.2.0: motion-planner crate (workspace member): planned head
          animation in live and Simulate runs, Simulate with acceleration,
          planned time estimate, motion fields in the Machine Profile.
          v0.2.1: canvas CAD update: polyline, text and leader tools,
          merge, traces (width, along / across), borders with interior
          relation, concentric fill, overscan, layers (Burn / Notes /
          Guides), burn order, undo / redo, Canvas Tools panel, read-only
          built-in examples, calibration pattern generator.
          v0.2.2: Projects and Active Project panels; burn / travel time in
          Job Details; run countdown on the canvas.
=============================================================================
-->
# Laser CNC Controller — v0.2.2

Local geometry editor, G-code generator, simulator and GRBL 0.9i serial controller for a 2-axis (XY) desktop diode-laser gantry. Laser power is controlled with GRBL's spindle commands: `M3 S<value>` on, `M5` off.

## Build (Ubuntu 24.04)

```bash
sudo apt update
sudo apt install build-essential pkg-config libx11-dev libxcursor-dev libxi-dev libxrandr-dev libgl1-mesa-dev libasound2-dev libwayland-dev libxkbcommon-dev
cargo run --release
```

## Build (Windows)

Rust via rustup (MSVC toolchain) and Visual Studio C++ Build Tools, then `cargo run --release`.

v0.2.1 needs Rust 1.88 or newer (geo crate). Check with `rustc --version`; update with `rustup update`.

## Data locations

- **Built-in examples:** written to the projects database at every start (read-only, see Examples below).

- **Projects and run index (redb):** `~/.local/share/laser-cnc-controller/laser-cnc.redb` (Linux), `%APPDATA%\LaserCNC` (Windows). Override with `LASER_CNC_DATA=/path`.
- **Records (visible):** `~/LaserCNC/` (Windows `%USERPROFILE%\LaserCNC`). Override with `LASER_CNC_HOME=/path`.
  - `exports/` — Export G-code: `<project>_<YYYY-MM-DD_HHMM>.gcode`
  - `runs/<project>_<YYYY-MM-DD_HHMMSS>[_frame]/` — one folder per Job or Frame run:
    - `program.gcode` — exact lines streamed
    - `run.json` — kind, project id/name, start/end, duration, estimate, outcome (Completed / Reset / Error / Disconnected), error and line, lines acknowledged, events (pause/resume requests, controller state changes), port, baud, firmware banner, power/feed settings, project snapshot, notes
    - `project.json` — project snapshot at run start
    - `serial.log` — timestamped TX/RX for the run
    - `images/` — reserved for inspection captures
  - `logs/YYYY-MM-DD.log` — every command sent (manual and streamed), controller messages, connect/disconnect, run start/end
- **Panel layout and Machine Profile:** `app.ron` in the eframe data directory for app id `laser-cnc-controller` (saved on exit and every 30 s).

## Machine Profile

Work X / Work Y (machine travel, default **380 × 280 mm**), Travel F (default 2500 mm/min) and Max PWM S (default 1000) describe the machine, not a project. They are saved with the app settings, restored at startup and applied to every project; changing them does not mark the project as unsaved.

**Motion** (planner limits): Max rate X / Y (`$110` / `$111`, default 3000 mm/min), Accel X / Y (`$120` / `$121`, default 10 mm/s²), Junction dev (`$11`, default 0.010 mm). While connected, the controller's own `$$` values are used instead (the panel shows which source is in use); the profile values apply when disconnected. A summary line shows the values the planner is using, plus its look-ahead (18 blocks).

## Connecting

Serial Communications panel: choose the port and baud (GRBL 0.9 default 115200), press **Connect**.

1. The port opens with DTR held low and stale input discarded, then a soft reset (Ctrl-X) is sent.
2. The status shows **Connecting… waiting for GRBL banner** (button: **Cancel**). On Linux, opening the port also hardware-resets the Nano; if no banner arrives within 2.5 s the soft reset is sent once more. No banner within 6 s → **Connect failed**.
3. On the `Grbl …` banner the panel shows **● Connected** and reads the controller with read-only queries — `$I`, `$G`, `$#`, `$$` — plus `?` status. Nothing is written to the controller's settings.

Connection block:

- **Firmware** — banner, e.g. `Grbl 0.9i ['$' for help]`
- **Build** — `$I`
- **Port** — port @ baud
- **State** — Idle / Run / Hold / Door / Alarm
- **WPos / MPos** — from the status report
- **Laser** — spindle word from `$G`: `M5 off` (green) or `M3/M4 ON S…` (amber)
- **Modes** — full `$G` parser state (hover for the whole line)
- **Unlock ($X)** — enabled only while the state is Alarm
- **Work offsets ($#)** and **Settings ($$)** — collapsible lists

## Job origin, zero and work fence

The machine has no limit switches or homing: GRBL counts steps from wherever the head was at power-on. The job's 0,0 is set by the operator.

- **Job origin marker** (cyan crosshair, "0,0 job") — placed on the canvas with the **Origin** tool (click), or dragged with **Select**. The program is generated relative to it: G-code X/Y = canvas X/Y − marker. Where the shapes sit on the canvas otherwise does not matter.
- **Zero X / Zero Y** (Jog Controls) — `G92 X0` / `G92 Y0`: the spot under the head becomes job 0,0, i.e. the marker. Cleared from the session on Reset, any controller reset, or disconnect; re-zero then.
- **Work fence** — rectangle from the marker, W × H mm (tool-strip fields, or drag its top-right handle with Select). Tool-strip toggle:
  - **ON** (solid orange): every program move — burns and laser-off travel — must stay inside 0…W, 0…H, otherwise a warning shows and Start / Trace are blocked. Once an axis is zeroed this session, jogs that would leave the fence are refused with a pop-up.
  - **OFF** (dashed grey): no fence checks.
- Always checked: the job's span from the marker must fit the machine travel (Machine Profile Work X × Work Y).

Every power-on:

1. Connect; wait for **● Connected**, State Idle, Laser `M5 off`.
2. Place the material. **Spot ON**, jog the spot to where the marker should be on the material (e.g. the part's lower-left corner, or 5 mm down-left of it if the marker is 5 mm down-left of the part on the canvas).
3. **Zero X**, **Zero Y**. WPos should read `X0.000 Y0.000`.
4. **Spot OFF**. **Trace** (Part / Job / Fence) to check the placement on the material, then **Start**.

## Machine control row (fixed, in job order)

- **Spot ON / Spot OFF** [power] — one latching button: `M3 S<power>` / `M5`. Lit while on. Switches off by itself on Laser OFF, the start of a job or trace (programs begin with `M5`), Reset, disconnect, alarm or any controller reset.
- **Trace** [Part | Job | Fence] [feed] — traces a rectangle at spot power and trace feed, relative to the job origin: **Part** = box around the selected shape(s), **Job** = box around all shapes, **Fence** = the work fence (only while the fence is ON). `M5`, `G0` to the corner, `M3 S<spot power>`, `G1` perimeter, `M5`. Recorded as a Trace run.
- **Start** — streams the job. Recorded as a Job run.
- **Pause / Resume** (one button, driven by the GRBL status state word):
  - Controller `Run` → **Pause** sends `@` (GRBL 0.9 safety door): decelerate, hold position, laser (spindle) off.
  - Controller `Door` or `Hold` → **Resume** sends `~`: GRBL restores the laser at its previous `M3 S` value, then continues motion.
  - A **DOOR** flag shows beside the button while the controller reports `Door`.
  - Hypothesis until verified on the machine: the controller firmware accepts `@`. Verify once at spot power (status shows `Door`, beam off; Resume restores beam, motion continues).
- **Reset** — `!` then Ctrl-X. Software stop only; not a physical emergency stop.
- **Laser OFF** — `M5` (also switches Spot off). For more power than a positioning spot, raise the Spot power field.
- **Simulate** [1× / 2× / 5× / 10× / 50×] — see Simulate below. A **SIMULATION** badge shows while it is on.

The row wraps onto a second line on narrow windows.

Manual commands, jogs and the control-row buttons that send G-code are refused while a job streams.

## Simulate

**Simulate** (machine-control row, after Laser OFF) swaps the Nano for a built-in virtual GRBL 0.9i (`src/vgrbl.rs`). Switch it only while disconnected; then **Connect** attaches the virtual controller (Serial panel shows **SIMULATOR**). Everything else is the real code path: Spot, jog, Zero X / Y, Go to zero, Trace, Start, Pause / Resume, Reset, Unlock, the work-fence checks, buffered streaming with sent / `ok` marks, and the live run view.

Virtual controller model:

- 128-byte serial RX buffer, 18-block planner; `M3`/`M4`/`M5`, `G92` and `$` commands wait for the planner to empty.
- Motion with acceleration from the motion planner (`crates/motion-planner`): per-axis max rate and acceleration, junction-deviation corner speeds, and a speed limit that lets the head stop by the end of the buffered blocks. `G1` at F (capped by `$110`/`$111`), `G0` at `$110`/`$111`. Limits from its `$$`: 3000 mm/min, 10 mm/s², 0.010 mm. Time scaled by the speed selector, changeable during a run.
- `!` hold and `@` door decelerate to a stop; Ctrl-X stops at once.
- `G90`/`G91`, `G92`, `G0`/`G1`, F, S, `M3`/`M4`/`M5`. `G10` → `error: Unsupported command` (as the real controller answers `G10 L20`).
- `$$` (settings with `$130`/`$131` = 380 / 280 mm), `$#`, `$G`, `$I`, `$N`, `$X`; `$H` → `error: Setting disabled`. Settings writes are refused.
- `?` status (MPos and WPos), `!` hold, `@` door (laser off), `~` resume (laser restored), Ctrl-X reset; reset during motion → `ALARM: Abort during cycle`, Alarm lock until **Unlock ($X)**.

Simulated runs are recorded like real ones and tagged **SIM** (Runs, Run History, daily log; folder suffix `_sim`, port `SIMULATOR`).

## Projects

Two panels in the left column, each collapsible and floatable on its own:

**Projects** — a browser for stored projects: **Saved projects** dropdown with **Open** and **Delete** (Delete asks first and keeps run records). After Open the dropdown returns to "Saved projects…", so it never shows a name other than the open project's. Keep it collapsed when not choosing a project.

**Active Project** (expanded on first launch) — the open project:

- Name, `*` when there are unsaved changes, `(not saved yet)` for a new project, `read-only` for an example; **New**, **Save**, **Save As** (or **Duplicate** for an example), **Close**, **Export G-code**.
- **Job Details** — Estimated (= Engraving time + Travel time), shapes per layer, burn extent, first shape's settings, **Engraving** distance and planned time (laser on), **Travel** distance and planned time (laser off: G0 travel, overscan leads, the stops around M3/M5), G-code lines. Each planned move's acceleration and deceleration count with that move.
- **Run History** — runs of the open project (Jobs/Traces, last run, list). Click a run to show it in the Runs panel.

Rules:

- Save on a project that has never been saved asks for a name. Save As warns before replacing an existing project.
- Save As creates a copy with a new project id and an empty run history.
- New, Open and Close prompt **Save / Discard / Cancel** when there are unsaved changes. The window title also shows `*`.
- Each project has a permanent id; run records link to it, so history survives renaming via Save As. Projects saved before v0.1.4 get an id the first time they are opened.
- File menu: New, New calibration pattern…, Open…, Save, Save As… (Duplicate… for an example), Close, Delete…, Export G-code. Edit menu: Undo, Redo.
- Opening a project fits its shapes in view.

## Runs panel

All recorded runs, newest first (colour by outcome). Selecting a run shows its details and an editable **Notes** field (**Save notes** writes `run.json` and the database). Buttons open the run folder, `runs/`, `exports/` and `logs/`. A "Recording" line shows while a run is in progress.

## Canvas (mini-CAD)

**Canvas Tools** is a panel like the others: docked above the canvas in two rows, the arrow collapses it to its title bar, **Float** puts it in a movable, resizable window where the tool groups wrap to the window width (groups never split), **Dock** returns it.

- Row 1: Select, Pan, Move, Origin | Rectangle, Line, Polyline, Circle, Text | Undo, Redo | Duplicate, Delete, Clear | Merge, burn order numbers
- Row 2: Grid on/off, grid spacing (0.1 / 0.5 / 1 / 5 / 10 mm, default 5 mm), Snap | Zoom (px/mm), Fit work area, Fit shapes | Layer for new shapes, Show (layer visibility) | Fence on/off, W, H (work fence size from the job origin)

G-code regenerates automatically on every edit.

| Input | Action |
|---|---|
| Left click | Select (Select tool); place point (drawing tools: click to start, click to finish) |
| Shift + left click | Add/remove from selection |
| Left drag on empty | Box select — rightward = window (fully inside), leftward = crossing |
| Left drag on shape | Move selection (attached notes move with it) |
| Left drag on handle | Resize (rectangle: corners/edges; circle: radius; line: endpoints; polyline: vertices) |
| Left drag on origin marker (Select) | Move the job origin |
| Left drag on fence corner (Select) | Resize the work fence |
| Left click (Origin tool) | Place the job origin |
| Polyline tool | Click vertices; click the first vertex to close; double-click or Enter to finish open; Backspace removes the last vertex |
| Text tool | Click to place (lower-left corner); edit in Shape Properties |
| Right drag | Pan (any tool) |
| Mouse wheel | Zoom at cursor |
| Middle drag, Shift + right drag | Zoom |
| Right click, Esc | Cancel current operation (Esc again clears selection, then returns to Select) |
| Delete | Delete selection (notes attached to deleted shapes are detached) |
| Ctrl+Z / Ctrl+Y, Ctrl+Shift+Z | Undo / redo (also tool strip and Edit menu) |

### Layers

- **Burn** — generates G-code.
- **Notes** (purple) — text and leaders shown on the canvas and in Simulate, never burned.
- **Guides** (dashed grey) — reference outlines (tape edge, clearance), never burned.

New shapes go on the layer chosen in the tool strip; Shape Properties moves a shape to another layer. **Show** hides layers from view and picking; a hidden Burn layer still burns.

### Burned geometry

- **Open line / polyline (trace).** Width 0: one pass on the centerline. Width > 0: N = round(width × density) lines, pitch width / N (2 mm at 10 lines/mm = 20 lines), open ends, no outline unless asked.
  - **Along:** N passes parallel to the path. End cap **Square** = flush with the end points; **Round** = half circle centred on each end point (adds width/2). Polyline corners **Miter** or **Round**.
  - **Across:** rails perpendicular to each segment every 1/density mm, clipped to the trace outline and to the corner bisectors, so corners neither overlap nor gap.
- **Closed shape** (rectangle, circle, closed polyline, merged outline, outline-font text):
  - **Outline pass** (off by default): one closed pass on the drawn edge.
  - **Border:** width (0 = none), alignment **Inside / Centred / Outside** the drawn edge, **Along** (N closed contour passes) or **Across** (rails across the band), density, corners.
  - **Interior:** fill **None / Horizontal / Vertical / Crosshatch / Concentric**, density. With a border: **Abut** (starts at the border's inner edge), **Overlap** (extends into the border by the given mm), **Gap** (unburned ring of the given mm).
  - Raster line positions are unchanged from v0.2.0: from one edge to the other, both edges included.
- **Raster:** **Bidirectional** (alternate passes reverse) or **Unidirectional** (travel back between passes). **Overscan** (mm): laser-off lead-in and lead-out at the burn feed along every open pass: `G0` to the lead-in, `G1` laser off to the pass start, `M3`, burn, `M5`, `G1` laser off to the lead-out. Drawn amber on the canvas.
- **Outline passes are one closed loop.** v0.2.0 burned each rectangle edge and each circle segment with its own `M5` / `G0` / `M3`; v0.2.1 burns the loop continuously.
- Selected traces and bordered shapes show their burned extent as a dashed outline.

### Merge

Select closed shapes on one layer and press **Merge**. Shapes qualify when they share an edge of non-zero length or overlap; snapped edges count as shared (coordinates rounded to 0.0001 mm). Corner-only contact or any gap does not merge: the merge is refused, the shapes that do not connect flash red for 3 s and the reason shows above the canvas. The merged outline takes the settings of the first selected shape and its place in the burn order; notes attached to any piece are re-attached to it. Merge is destructive; **Undo** restores the pieces.

### Text and leaders

- **Notes layer:** screen text (the egui proportional font), height in mm.
- **Burn layer:** **Single-stroke** font (this project's own centerline glyphs: A–Z, 0–9 and common symbols; lower case is drawn as upper case) for small IDs, or **Outline** font (Ubuntu-Light, glyphs as closed shapes with border / fill like any closed shape). **Convert to geometry** replaces the text by polylines or a merged outline.
- **Add leader**, then click the point it points to; a shape under the click attaches the note, which then moves with that shape. Moving the note itself keeps its leader point. **Detach** / **Remove leader** in Shape Properties.

### Burn order

Burn layer shapes burn in list order. The order button in the tool strip shows the numbers on the canvas; Shape Properties moves the selected shape **First / Earlier / Later / Last**.

### Undo / redo

Every committed edit (canvas, Shape Properties, origin, fence, merge, order) is one step; a drag commits when it ends. 200 steps. The history clears on New, Open, Close and Duplicate. Project name, id and read-only flag are not part of it.

**Shape Properties** is the top panel of the right column. Sections: layer; geometry (rectangle X/Y/W/H, line start/end, circle centre/diameter, polyline / merged outline position, size, closed flag; text string, height, position, font); Trace; Border and Interior; Raster; Process (feed, power, passes); Burn order. Line and border counts are shown read-only (e.g. "20 (2.00 mm × 10.0 lines/mm)").

During a Job or Trace run the canvas shows the run live: the run's path, green where done, a segment to the head, and the head marker. At Start the streamed lines are planned by the motion planner (acceleration, corners, a full stop at every `M3`/`M5`, look-ahead 18 blocks) from the head's WPos at that moment. A run clock then places the head on that trajectory every frame, so it moves at the planned speeds and follows every corner. The clock runs at the Simulate speed in Simulate and stops while the controller reports Hold, Door or Alarm. The head marker and its segment are **red while the laser is on** and **yellow while it is off**, from the trajectory. The marker is a model of the machine, not a measurement: GRBL 0.9 reports no executing line, and its position reports (every 250 ms) are not used for the marker.

**Run countdown.** During a Job or Trace run (machine or Simulate) a large timer sits in the canvas's top-right corner: time remaining, a progress bar, elapsed and total. It follows the same run clock as the head marker, so it is planned machine time: it runs at the Simulate speed in Simulate and stops while the controller reports Hold, Door (shown as PAUSED) or Alarm. On the machine it is a prediction from the planner; comparing it with the real end of a run checks the planner timing.

## Examples (built in, read-only)

Written to the projects database at every start with fixed ids (run history stays linked; a database reset gets them back). They show `· read-only` in the Saved projects list, cannot be deleted, and their names cannot be used by Save As.

- **Example - Feature demo** — every canvas feature at full size with numbered Notes-layer labels.
- **Example - LIG tape demo 50x100** — the same features inside a 50 × 100 mm tape with 5 mm clearance (40 × 90 mm usable): vertical traces 2 × 15 mm (w 0, along, across, round), 10 mm squares with 2 mm border (abut, overlap 0.5, gap 0.5), merged comb 30 × 14, circles ø10 (concentric, raster), overscan strips 30 × 3 (off, 2 mm), horizontal traces 30 mm (w 0, 2 mm along, 2 mm across). Guides: tape edge and clearance. Job origin at the tape's lower-left corner; work fence ON 50 × 100 mm (the tape).
- **Calibration - Power x feed / Line density / Fill direction / Overscan (tape)** — default calibration patterns on the tape.

Recipe in the examples: S150 (15 % of S1000), F1200, 10 lines/mm.

Opening an example is read-only: select items and inspect their properties, Simulate, Trace and Start all work. Any edit is undone and a prompt offers **Duplicate**, which asks for a name and saves an editable copy (new id). Save / Save As on an example also duplicate it.

**File > New calibration pattern…** generates an editable, unsaved project: **Power × feed matrix** (power range and steps as rows, feed range and steps as columns), **Line density** (density range and steps), **Fill direction** (horizontal, vertical, crosshatch, concentric, trace along, trace across), **Overscan** (0 up to a maximum, in steps). Patch size, gap, fixed power / feed / density, and placement on the 50 × 100 mm tape (refused with a message when it does not fit 40 × 90 mm) or free. Labels go on the Notes layer.

## Panels

- Left: Serial Communications, Projects, Active Project, Machine Profile, Runs. Right: Shape Properties, Jog Controls, G-code / Stream. Centre: Canvas Tools above the canvas. (v0.2.2 adds Active Project; floating/docked flags reset once.)
- Each panel collapses with its arrow and detaches with **Float** into a movable, resizable window; **Dock** returns it. Layout is saved on exit and restored at launch. First launch: Serial Communications and Active Project are expanded.
- Jog Controls: step 0.1 / 10 mm buttons plus a typed step field (0.01–500 mm, ▲▼); X/Y cross (HOME disabled: no homing switches); **Zero X**, **Zero Y**, **Go to zero** (`G90 G0 X0 Y0`) on one row, with the session zero state.
- G-code / Stream: compact monospace list with per-line state — `·` queued, `▶` sent awaiting `ok`, `✔` acknowledged, `✖` first line rejected by GRBL — following the active line; counts (Queued / Sent / OK); controller state word; Serial RX/TX log with manual commands, controller messages, errors and alarms; **manual command line** (G-code or `$` command, Enter or **Send**; disabled while a job streams). Floating window resizes from its corner; its background is 80 % transparent so it can sit over the canvas.
- All numeric inputs are typed entry (commit with Enter or by leaving the field) with ▲▼ step buttons. Geometry fields step by the grid spacing. No sliders.

## Streaming

Matches LaserGRBL's default **Buffered** streaming mode (character counting):

- Serial I/O runs on its own thread, independent of UI repaint (streaming continues while the window is minimised).
- GRBL's serial RX buffer is modelled as 127 bytes. A line is sent when the bytes already in flight plus the line fit; each `ok` / `error` releases the oldest in-flight line. Several lines are queued in GRBL at once, so the planner is not starved between short segments.
- Lines are trimmed and spaces removed (except `$` commands), terminated with `\n`.
- Real-time bytes (`?` `!` `~` `@` Ctrl-X) are written directly and not counted.
- Status (`?`) is queried every 250 ms.
- A line `error:` is counted and streaming continues (LaserGRBL behaviour). `ALARM`, a controller reset (banner) or **Reset** stops the stream.
- An `ok` means the line was accepted, not that motion has finished. A run ends once all lines are answered and the controller reports `Idle`: **Completed** with no errors, **Error** with the first error, its line and the error count. An `ALARM` during a run ends it as **Error**.

## Display scale

The UI follows the operating system's scaling. egui zoom: **Ctrl + Plus / Ctrl + Minus / Ctrl + 0**. Default window 1280 × 700, minimum 990 × 600.

## Known limitations

- Streaming, connection and jobs have been run on the machine (v0.1.5–v0.1.9). The motion planner's timing has not yet been compared against the machine.
- Planner hypotheses until checked: GRBL 0.9 look-ahead of 18 blocks; a full stop on every `M3`/`M4`/`M5`, `G92` and `$` line.
- The time estimate starts from job 0,0; the live view plans from the head's position at Start.
- HOME is not implemented.
- Overscan on GRBL 0.9 (hypothesis from the planner model, to be checked on the machine): the planner empties at every `M3` / `M5`, so the head comes to a full stop where each burn starts and ends, with or without a lead-in. The lead-in and lead-out add laser-off travel but are not expected to change the fused raster-end band. Removing the band needs laser output that switches without stopping (GRBL 1.1 laser mode, `$32=1`, `M4`).
- Very short polyline segments on a wide trace can make the inner parallel passes fold back at the corner.
- Text has no rotation.

## Files

- `src/main.rs` — entry point, window title, app id
- `src/app.rs` — application shell, panels, machine control row, project management, run tracking, Runs panel, dialogs, simulation timing, G-code monitor
- `src/canvas.rs` — CAD canvas engine: view, tools (incl. polyline, text, leader), mouse, selection, handles, merge, layers, burn order numbers, tool strip groups, status bar
- `src/paths.rs` — burn-path engine: traces, borders, fills, rails, contours, overscan, merge rules (geo crate)
- `src/font.rs` — single-stroke font, outline-font glyphs (Ubuntu-Light), text metrics
- `src/examples.rs` — built-in read-only examples, calibration pattern generator
- `src/widgets.rs` — numeric spin field, painted tool icons
- `src/runlog.rs` — `~/LaserCNC` folders, run records, daily log, G-code export
- `src/geometry.rs` — project and shape model (kinds, layers, trace / border / text fields), project id
- `src/gcode.rs` — toolpath from the burn strokes, G-code and time estimate
- `src/machine.rs` — GRBL serial worker thread, buffered (character-counting) streaming, connect handshake, controller info, command journal, Port (serial / virtual)
- `src/vgrbl.rs` — virtual GRBL 0.9i for Simulate
- `src/storage.rs` — redb persistence (projects, runs)
- `crates/motion-planner/` — `no_std` motion planner crate (workspace member): `src/lib.rs` kinematics, junction deviation, trapezoid profiles, look-ahead planner, trajectory sampling; `src/gcode.rs` G-code lines to planner blocks. `cargo test --workspace` runs its tests.
<!--
=============================================================================
End of file: README.md
Location:    laser-cnc-controller/README.md
Created:     2026-10-17 18:43:10 UTC
=============================================================================
-->
