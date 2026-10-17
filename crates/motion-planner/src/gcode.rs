// =============================================================================
// File:     gcode.rs
// Location: laser-cnc-controller/crates/motion-planner/src/gcode.rs
// Version:  v0.1.0
// Comments: G-code lines to planner Blocks (needs the "alloc" feature).
//           Reads the subset the laser controller streams to GRBL 0.9:
//           - G0 / G1 (modal), G90 / G91, G21 (mm assumed), G92 X Y (the
//             current point gets the given work coordinates), F, S,
//             M3 / M4 (laser on), M5 (laser off).
//           - Comments "( … )" and "; …" are removed; spaces and case ignored.
//           - '$' lines are skipped.
//           - G2 / G3 arcs are not interpolated: treated as a straight move to
//             the end point (the controller program contains no arcs).
//           Stops (Block::stop_before): GRBL 0.9 empties its planner before
//           executing an M3/M4/M5, G92 or '$' line, so the next move starts
//           from a full stop. Same rule as the virtual controller
//           (src/vgrbl.rs in the app).
//           Coordinates are program (work) coordinates.
// =============================================================================
use alloc::string::String;
use alloc::vec::Vec;
use crate::{Block, AXES};

/// Uppercase, spaces and comments removed.
fn clean(line: &str) -> String {
    let mut out = String::new();
    let mut paren = false;
    for c in line.chars() {
        match c {
            '(' => paren = true,
            ')' => paren = false,
            ';' => break,
            _ if paren => {}
            ' ' | '\t' | '\r' | '\n' => {}
            _ => out.push(c.to_ascii_uppercase()),
        }
    }
    out
}

/// Letter/number words of a cleaned line; None if a number cannot be read.
fn words(line: &str) -> Option<Vec<(char, f32)>> {
    let b = line.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let letter = b[i] as char;
        if !letter.is_ascii_alphabetic() {
            return None;
        }
        let mut j = i + 1;
        while j < b.len() && (b[j].is_ascii_digit() || matches!(b[j], b'.' | b'-' | b'+')) {
            j += 1;
        }
        let value = line.get(i + 1..j)?.parse::<f32>().ok()?;
        out.push((letter, value));
        i = j;
    }
    Some(out)
}

/// Modal state while reading a program.
#[derive(Clone, Copy, Debug)]
pub struct Reader {
    /// Current position, program coordinates.
    pub pos: [f32; AXES],
    pub absolute: bool,
    pub rapid: bool,
    /// Feed, mm/min.
    pub feed: f32,
    /// S value.
    pub power: u32,
    /// Spindle (laser) commanded on.
    pub spindle: bool,
    /// The next move starts from a full stop.
    pending_stop: bool,
}
impl Reader {
    /// Start at `pos` (program coordinates) in G0 G90, laser off, from a stop.
    pub fn new(pos: [f32; AXES]) -> Self {
        Self { pos, absolute: true, rapid: true, feed: 0.0, power: 0, spindle: false, pending_stop: true }
    }
    /// Read one line; returns the move it produces, if any.
    pub fn line(&mut self, raw: &str, source: usize) -> Option<Block> {
        let line = clean(raw);
        if line.is_empty() {
            return None;
        }
        if line.starts_with('$') {
            self.pending_stop = true;
            return None;
        }
        let words = words(&line)?;
        let mut target: [Option<f32>; AXES] = [None; AXES];
        let mut g92 = false;
        for &(letter, v) in &words {
            match letter {
                'G' => match libm::roundf(v * 10.0) as i32 {
                    0 => self.rapid = true,
                    10 | 20 | 30 => self.rapid = false,
                    900 => self.absolute = true,
                    910 => self.absolute = false,
                    920 => g92 = true,
                    _ => {}
                },
                'M' => match v as i32 {
                    3 | 4 => {
                        self.spindle = true;
                        self.pending_stop = true;
                    }
                    5 => {
                        self.spindle = false;
                        self.pending_stop = true;
                    }
                    _ => {}
                },
                'X' => target[0] = Some(v),
                'Y' => target[1] = Some(v),
                'F' => self.feed = v,
                'S' => self.power = v.max(0.0) as u32,
                _ => {}
            }
        }
        if g92 {
            // The current point gets the given work coordinates.
            for (i, t) in target.iter().enumerate() {
                if let Some(v) = t {
                    self.pos[i] = *v;
                }
            }
            self.pending_stop = true;
            return None;
        }
        if target.iter().all(|t| t.is_none()) {
            return None;
        }
        let start = self.pos;
        let mut end = start;
        for (i, t) in target.iter().enumerate() {
            if let Some(v) = t {
                end[i] = if self.absolute { *v } else { start[i] + *v };
            }
        }
        self.pos = end;
        let block = Block {
            start,
            end,
            feed: self.feed,
            rapid: self.rapid,
            laser: self.spindle && self.power > 0,
            power: self.power,
            stop_before: self.pending_stop,
            source,
        };
        self.pending_stop = false;
        Some(block)
    }
}

/// Moves of a whole program, starting at `start` (program coordinates).
pub fn blocks_from_lines<S: AsRef<str>>(lines: &[S], start: [f32; AXES]) -> Vec<Block> {
    let mut reader = Reader::new(start);
    lines.iter().enumerate().filter_map(|(i, l)| reader.line(l.as_ref(), i)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn laser_program_blocks() {
        let lines = ["G21", "G90", "M5", "G0 X1 Y1", "M3 S300", "G1 X5 Y1 F600", "G1 X5 Y2", "M5", "G0 X0 Y0"];
        let b = blocks_from_lines(&lines, [0.0, 0.0]);
        assert_eq!(b.len(), 4);
        assert!(b[0].rapid && !b[0].laser && b[0].stop_before);
        assert!(!b[1].rapid && b[1].laser && b[1].stop_before && b[1].feed == 600.0 && b[1].source == 5);
        assert!(b[2].laser && !b[2].stop_before && b[2].start == [5.0, 1.0]);
        assert!(b[3].rapid && !b[3].laser && b[3].stop_before);
    }

    #[test]
    fn relative_moves_g92_and_comments() {
        let lines = ["G92 X0 Y0", "G91 G0 X2 (jog) Y-1", "G90", "g1 x3 f100 ; slow"];
        let b = blocks_from_lines(&lines, [7.0, 7.0]);
        assert_eq!(b[0].start, [0.0, 0.0]);
        assert_eq!(b[0].end, [2.0, -1.0]);
        assert_eq!(b[1].end, [3.0, -1.0]);
        assert!(b[0].stop_before);
    }
}
// =============================================================================
// End of file: gcode.rs
// Location:    laser-cnc-controller/crates/motion-planner/src/gcode.rs
// Created:     2026-10-16 21:27:22 UTC
// =============================================================================
