// =============================================================================
// File:     main.rs
// Location: laser-cnc-controller/src/main.rs
// Version:  v0.1.5
// Comments: Native application entry point.
//           - v0.1.1: registers canvas and widgets modules.
//           - v0.1.2: registers runlog module.
//           - Stable app name/app id ("laser-cnc-controller") so the saved
//             panel layout survives version changes.
//           - Window title carries the package version from Cargo.toml.
//           - v0.1.3: default window 1280×700, minimum 990×600, so it fits a
//             laptop screen at 150% scaling.
//           - v0.1.4: registers vgrbl module (virtual GRBL for Simulate).
//           - v0.1.5: registers examples, font and paths modules (canvas CAD
//             update, app v0.2.1).
// =============================================================================
mod app;
mod canvas;
mod examples;
mod font;
mod geometry;
mod gcode;
mod machine;
mod paths;
mod runlog;
mod storage;
mod vgrbl;
mod widgets;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui_viewport(),
        ..Default::default()
    };
    eframe::run_native(
        "laser-cnc-controller",
        options,
        Box::new(|cc| Ok(Box::new(app::LaserApp::new(cc)))),
    )
}

fn egui_viewport() -> eframe::egui::ViewportBuilder {
    eframe::egui::ViewportBuilder::default()
        .with_title(format!("Laser CNC Controller v{}", env!("CARGO_PKG_VERSION")))
        .with_app_id("laser-cnc-controller")
        .with_inner_size([1280.0, 700.0])
        .with_min_inner_size([990.0, 600.0])
}
// =============================================================================
// End of file: main.rs
// Location:    laser-cnc-controller/src/main.rs
// Created:     2026-10-17 17:35:38 UTC
// =============================================================================
