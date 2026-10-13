<!--
=============================================================================
File:     README.md
Location: laser-cnc-controller/README.md
Version:  v0.1.5
Comments: Project overview, build, controls and limitations.
          v0.1.4: project management, run records, daily log, Runs panel,
          Shape Properties dock slot in the canvas tool strip.
          v0.1.5: threaded LaserGRBL-style buffered streaming, connection
          handshake and controller info, Unlock ($X), manual command line,
          no fixed UI scale.
=============================================================================
-->
# Laser CNC Controller — v0.1.5

Local geometry editor, G-code generator, simulator and GRBL 0.9i serial controller for a 2-axis (XY) desktop diode-laser gantry. Laser power is controlled with GRBL's spindle commands: `M3 S<value>` on, `M5` off.

## Build (Ubuntu 24.04)

```bash
sudo apt update
sudo apt install build-essential pkg-config libx11-dev libxcursor-dev libxi-dev libxrandr-dev libgl1-mesa-dev libasound2-dev libwayland-dev libxkbcommon-dev
cargo run --release
```

## Build (Windows)

Rust via rustup (MSVC toolchain) and Visual Studio C++ Build Tools, then `cargo run --release`.

## Data locations

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
- **Panel layout:** `app.ron` in the eframe data directory for app id `laser-cnc-controller`.

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

## Machine control row (fixed, in job order)

- **Spot ON** [power] — `M3 S<power>`. Low-power spot for positioning before zeroing X/Y.
- **Frame** [feed] — traces the geometry bounding box at spot power: `M5`, `G0` to the start corner, `M3 S<spot power>`, `G1` perimeter at the frame feed, `M5`. Recorded as a Frame run.
- **Start** — streams the job. Recorded as a Job run.
- **Pause / Resume** (one button, driven by the GRBL status state word):
  - Controller `Run` → **Pause** sends `@` (GRBL 0.9 safety door): decelerate, hold position, laser (spindle) off.
  - Controller `Door` or `Hold` → **Resume** sends `~`: GRBL restores the laser at its previous `M3 S` value, then continues motion.
  - A **DOOR** flag shows beside the button while the controller reports `Door`.
  - Hypothesis until verified on the machine: the controller firmware accepts `@`. Verify once at spot power (status shows `Door`, beam off; Resume restores beam, motion continues).
- **Reset** — `!` then Ctrl-X. Software stop only; not a physical emergency stop.
- **Laser ON** [power, default S150] — `M3 S<power>`. Beam stays on until Laser OFF, stationary or while jogging.
- **Laser OFF** — `M5`.

Manual commands, jogs and the control-row buttons that send G-code are refused while a job streams.

## Projects

Projects panel (left column), top to bottom:

- **Saved projects** dropdown with **Open** and **Delete** (acts on the dropdown selection; Delete asks first and keeps run records).
- **Active Project** — name, `*` when there are unsaved changes, `(not saved yet)` for a new project; **New**, **Save**, **Save As**, **Close**, **Export G-code** (act on the open project).
- **Job Details** — estimate, shapes, part size, process settings, engraving/travel length, G-code lines.
- **Run History** — runs of the open project (Jobs/Frames, last run, list). Click a run to show it in the Runs panel.

Rules:

- Save on a project that has never been saved asks for a name. Save As warns before replacing an existing project.
- Save As creates a copy with a new project id and an empty run history.
- New, Open and Close prompt **Save / Discard / Cancel** when there are unsaved changes. The window title also shows `*`.
- Each project has a permanent id; run records link to it, so history survives renaming via Save As. Projects saved before v0.1.4 get an id the first time they are opened.
- File menu: New, Open…, Save, Save As…, Close, Delete…, Export G-code.

## Runs panel

All recorded runs, newest first (colour by outcome). Selecting a run shows its details and an editable **Notes** field (**Save notes** writes `run.json` and the database). Buttons open the run folder, `runs/`, `exports/` and `logs/`. A "Recording" line shows while a run is in progress.

## Canvas (mini-CAD)

Tool strip: Select, Pan, Move | Rectangle, Line, Circle | Duplicate, Delete, Clear | Grid on/off, grid spacing (0.1 / 0.5 / 1 / 5 / 10 mm), Snap on/off | Zoom (px/mm), Fit | **Shape Properties [Float/Dock]**.

G-code regenerates automatically on every edit.

| Input | Action |
|---|---|
| Left click | Select (Select tool); place point (drawing tools: click to start, click to finish) |
| Shift + left click | Add/remove from selection |
| Left drag on empty | Box select — rightward = window (fully inside), leftward = crossing |
| Left drag on shape | Move selection |
| Left drag on handle | Resize (rectangle: corners/edges; circle: radius; line: endpoints) |
| Right drag | Pan (any tool) |
| Mouse wheel | Zoom at cursor |
| Middle drag, Shift + right drag | Zoom |
| Right click, Esc | Cancel current operation (Esc again clears selection, then returns to Select) |
| Delete | Delete selection |

Move tool: click base point, then click destination. Snap rounds points to the grid spacing. The status bar shows the cursor position in mm, selection count, zoom, grid and snap state, and the current tool and its instructions.

**Shape Properties** docks into the tool strip (hidden). Selecting a shape while it is docked floats it automatically at its last position and size; **Dock** hides it again. While floating it stays open and goes blank when nothing is selected. Parametric fields per type: rectangle X/Y/Width/Height, line start/end, circle centre/diameter, plus operation, fill, lines/mm, feed, power and passes.

## Panels

- Left: Serial Communications, Projects, Machine Profile, Runs. Right: Jog Controls, G-code / Stream, Simulation. Shape Properties: tool strip slot / floating.
- Each panel collapses with its arrow and detaches with **Float** into a movable, resizable window; **Dock** returns it. Layout is saved on exit and restored at launch. First launch: only Serial Communications is expanded.
- Simulation: Start (from the beginning), Pause/Resume toggle, Reset; speed 1× / 2× / 5× / 10× / 50×.
- G-code / Stream: compact monospace list with per-line state — `·` queued, `▶` sent awaiting `ok`, `✔` acknowledged, `✖` first line rejected by GRBL — following the active line; counts (Queued / Sent / OK); controller state word; Serial RX/TX log with manual commands, controller messages, errors and alarms; **manual command line** (G-code or `$` command, Enter or **Send**; disabled while a job streams). Floating window resizes from its corner.
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

- Hardware behaviour is not yet verified against the physical machine. The connect handshake, `$I`/`$G`/`$#`/`$$` reply formats and buffered streaming were tested only against a simulated GRBL 0.9 on a virtual serial port.
- Time estimate is a constant-speed lower bound (no acceleration).
- HOME is not implemented. No undo yet.

## Files

- `src/main.rs` — entry point, window title, app id
- `src/app.rs` — application shell, panels, machine control row, project management, run tracking, Runs panel, dialogs, simulation timing, G-code monitor
- `src/canvas.rs` — CAD canvas engine: view, tools, mouse, selection, handles, tool strip, status bar
- `src/widgets.rs` — numeric spin field, painted tool icons
- `src/runlog.rs` — `~/LaserCNC` folders, run records, daily log, G-code export
- `src/geometry.rs` — project and shape model, project id
- `src/gcode.rs` — toolpath, G-code and time estimate
- `src/machine.rs` — GRBL serial worker thread, buffered (character-counting) streaming, connect handshake, controller info, command journal
- `src/storage.rs` — redb persistence (projects, runs)
<!--
=============================================================================
End of file: README.md
Location:    laser-cnc-controller/README.md
Created:     2026-10-13 15:15:40 UTC
=============================================================================
-->
