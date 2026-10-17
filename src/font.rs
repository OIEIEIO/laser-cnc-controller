// =============================================================================
// File:     font.rs
// Location: laser-cnc-controller/src/font.rs
// Version:  v0.1.0
// Comments: Text geometry for the canvas Text tool (app v0.2.1).
//           - Single-stroke font: this project's own glyph set, drawn as
//             centerlines on a 4 x 6 grid (cap height 6 units). Burns one pass
//             per stroke; readable down to 2-3 mm. Characters: A-Z (lower
//             case is drawn as upper case), 0-9 (slashed zero), space and
//             . , - + / : = % # ( ) _ * ° ? ! ' " < > ×. Others draw as '?'.
//           - Outline font: Ubuntu-Light (the proportional font egui embeds,
//             crate epaint_default_fonts, Ubuntu Font Licence). Glyph outlines
//             are flattened to closed rings (curves split into straight
//             segments) for fill like any closed shape.
//           - Height h = capital-letter height in mm, for both fonts.
//           - Notes-layer text is drawn by egui with the same Ubuntu-Light
//             font; screen_font_size() converts the cap height to egui's font
//             size so the canvas text matches the measured width.
// =============================================================================
use ab_glyph::{Font, FontRef, OutlineCurve};
use std::sync::OnceLock;
use crate::geometry::{Pt, TextFont};

/// Single-stroke advance per character and per space, grid units (cap height 6).
const ADVANCE: f32 = 5.5;
const SPACE: f32 = 4.0;
const GRID_H: f32 = 6.0;

/// Glyph strokes: polylines separated by '|', points "x,y" separated by spaces.
fn glyph(c: char) -> &'static str {
    match c.to_ascii_uppercase() {
        'A' => "0,0 2,6 4,0|0.67,2 3.33,2",
        'B' => "0,0 0,6 3,6 4,5 4,4 3,3 0,3|3,3 4,2 4,1 3,0 0,0",
        'C' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1",
        'D' => "0,0 0,6 2,6 4,4 4,2 2,0 0,0",
        'E' => "4,6 0,6 0,0 4,0|0,3 3,3",
        'F' => "4,6 0,6 0,0|0,3 3,3",
        'G' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,3 2,3",
        'H' => "0,0 0,6|4,0 4,6|0,3 4,3",
        'I' => "1,6 3,6|2,6 2,0|1,0 3,0",
        'J' => "4,6 4,1 3,0 1,0 0,1",
        'K' => "0,0 0,6|4,6 0,2|1,3 4,0",
        'L' => "0,6 0,0 4,0",
        'M' => "0,0 0,6 2,3 4,6 4,0",
        'N' => "0,0 0,6 4,0 4,6",
        'O' => "1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,1 1,0",
        'P' => "0,0 0,6 3,6 4,5 4,4 3,3 0,3",
        'Q' => "1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,1 1,0|2.5,1.5 4,0",
        'R' => "0,0 0,6 3,6 4,5 4,4 3,3 0,3|2,3 4,0",
        'S' => "4,5 3,6 1,6 0,5 0,4 1,3 3,3 4,2 4,1 3,0 1,0 0,1",
        'T' => "0,6 4,6|2,6 2,0",
        'U' => "0,6 0,1 1,0 3,0 4,1 4,6",
        'V' => "0,6 2,0 4,6",
        'W' => "0,6 1,0 2,4 3,0 4,6",
        'X' => "0,0 4,6|0,6 4,0",
        'Y' => "0,6 2,3 4,6|2,3 2,0",
        'Z' => "0,6 4,6 0,0 4,0",
        '0' => "1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,1 1,0|0.6,1 3.4,5",
        '1' => "1,5 2,6 2,0|1,0 3,0",
        '2' => "0,5 1,6 3,6 4,5 4,4 0,0 4,0",
        '3' => "0,5 1,6 3,6 4,5 4,4 3,3 1,3|3,3 4,2 4,1 3,0 1,0 0,1",
        '4' => "3,0 3,6 0,2 4,2",
        '5' => "4,6 0,6 0,3 3,3 4,2 4,1 3,0 1,0 0,1",
        '6' => "3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,2 3,3 0,3",
        '7' => "0,6 4,6 1,0",
        '8' => "1,3 0,4 0,5 1,6 3,6 4,5 4,4 3,3 1,3 0,2 0,1 1,0 3,0 4,1 4,2 3,3",
        '9' => "4,3 1,3 0,4 0,5 1,6 3,6 4,5 4,1 3,0 1,0",
        '.' => "1.6,0 2.4,0 2.4,0.8 1.6,0.8 1.6,0",
        ',' => "2.2,0.6 2.2,0 1.6,-1",
        '-' => "1,3 3,3",
        '+' => "1,3 3,3|2,2 2,4",
        '/' => "0,0 4,6",
        ':' => "2,1 2,1.6|2,4 2,4.6",
        '=' => "1,2 3,2|1,4 3,4",
        '%' => "0,0 4,6|0,6 0,4.8 1.2,4.8 1.2,6 0,6|2.8,0 4,0 4,1.2 2.8,1.2 2.8,0",
        '#' => "1,0 1.5,6|2.5,0 3,6|0,2 4,2|0,4 4,4",
        '(' => "2.5,6 1.5,5 1.5,1 2.5,0",
        ')' => "1.5,6 2.5,5 2.5,1 1.5,0",
        '_' => "0,0 4,0",
        '*' => "2,1.5 2,4.5|0.7,2.25 3.3,3.75|0.7,3.75 3.3,2.25",
        '°' => "1.5,6 2.5,6 2.5,5 1.5,5 1.5,6",
        '!' => "2,6 2,2|2,0.4 2,0",
        '\'' => "2,6 2,4.5",
        '"' => "1.5,6 1.5,4.5|2.5,6 2.5,4.5",
        '<' => "4,5 0,3 4,1",
        '>' => "0,5 4,3 0,1",
        '×' => "1,2 3,4|1,4 3,2",
        _ => "0,5 1,6 3,6 4,5 4,4 2,3 2,2|2,0.4 2,0",
    }
}

fn parse(src: &str) -> Vec<Vec<Pt>> {
    src.split('|').map(|stroke| stroke.split_whitespace().filter_map(|p| {
        let (x, y) = p.split_once(',')?;
        Some((x.parse::<f32>().ok()?, y.parse::<f32>().ok()?))
    }).collect()).filter(|s: &Vec<Pt>| s.len() >= 2).collect()
}

/// Single-stroke text as polylines, anchor = lower-left of the first capital, height h (mm).
pub fn stroke_text(text: &str, x: f32, y: f32, h: f32) -> Vec<Vec<Pt>> {
    let k = h / GRID_H;
    let mut cx = 0.0f32;
    let mut out = vec![];
    for c in text.chars() {
        if c == ' ' { cx += SPACE; continue; }
        for s in parse(glyph(c)) { out.push(s.iter().map(|p| (x + (cx + p.0) * k, y + p.1 * k)).collect()); }
        cx += ADVANCE;
    }
    out
}
fn stroke_width(text: &str, h: f32) -> f32 {
    let k = h / GRID_H;
    let mut w = 0.0f32; let mut last_gap = 0.0f32;
    for c in text.chars() { if c == ' ' { w += SPACE; last_gap = 0.0; } else { w += ADVANCE; last_gap = ADVANCE - 4.0; } }
    (w - last_gap).max(0.0) * k
}

// ---------------------------------------------------------------- outline font
fn face() -> Option<&'static FontRef<'static>> {
    static FACE: OnceLock<Option<FontRef<'static>>> = OnceLock::new();
    FACE.get_or_init(|| FontRef::try_from_slice(epaint_default_fonts::UBUNTU_LIGHT).ok()).as_ref()
}
/// Height of capital H in font units.
fn cap_units(f: &FontRef<'static>) -> f32 {
    f.outline(f.glyph_id('H')).map(|o| (o.bounds.max.y - o.bounds.min.y).abs()).filter(|v| *v > 0.0).unwrap_or(f.ascent_unscaled() * 0.7)
}
/// egui font size (points) whose capital height is `cap` points (Ubuntu-Light, egui scales ascent - descent to the size).
pub fn screen_font_size(cap: f32) -> f32 {
    match face() { Some(f) => cap * (f.ascent_unscaled() - f.descent_unscaled()) / cap_units(f), None => cap / 0.7 }
}
fn outline_width(text: &str, h: f32) -> f32 {
    let Some(f) = face() else { return stroke_width(text, h); };
    let k = h / cap_units(f);
    text.chars().map(|c| f.h_advance_unscaled(f.glyph_id(c))).sum::<f32>() * k
}
/// Text width in mm. Burn text uses its font; notes are measured with the outline font (egui draws them with it).
pub fn text_width(text: &str, h: f32, font: TextFont, burn: bool) -> f32 {
    if burn && font == TextFont::Stroke { stroke_width(text, h) } else { outline_width(text, h) }
}
/// Outline glyph rings (closed, last point != first), anchor lower-left on the baseline, cap height h.
/// Fill with the even-odd rule.
pub fn outline_rings(text: &str, x: f32, y: f32, h: f32) -> Vec<Vec<Pt>> {
    let Some(f) = face() else { return vec![]; };
    let k = h / cap_units(f);
    let mut pen = 0.0f32;
    let mut rings: Vec<Vec<Pt>> = vec![];
    for c in text.chars() {
        let id = f.glyph_id(c);
        if let Some(o) = f.outline(id) {
            let map = |p: ab_glyph::Point| (x + (pen + p.x) * k, y + p.y * k);
            let mut ring: Vec<Pt> = vec![];
            let mut last: Option<ab_glyph::Point> = None;
            for curve in &o.curves {
                let (start, pts): (ab_glyph::Point, Vec<ab_glyph::Point>) = match *curve {
                    OutlineCurve::Line(a, b) => (a, vec![b]),
                    OutlineCurve::Quad(a, c1, b) => (a, (1..=8).map(|i| { let t = i as f32 / 8.0; let u = 1.0 - t;
                        ab_glyph::point(u * u * a.x + 2.0 * u * t * c1.x + t * t * b.x, u * u * a.y + 2.0 * u * t * c1.y + t * t * b.y) }).collect()),
                    OutlineCurve::Cubic(a, c1, c2, b) => (a, (1..=10).map(|i| { let t = i as f32 / 10.0; let u = 1.0 - t;
                        ab_glyph::point(u*u*u*a.x + 3.0*u*u*t*c1.x + 3.0*u*t*t*c2.x + t*t*t*b.x, u*u*u*a.y + 3.0*u*u*t*c1.y + 3.0*u*t*t*c2.y + t*t*t*b.y) }).collect()),
                };
                let continues = last.is_some_and(|l| (l.x - start.x).abs() < 1e-3 && (l.y - start.y).abs() < 1e-3);
                if !continues {
                    if ring.len() >= 3 { rings.push(std::mem::take(&mut ring)); } else { ring.clear(); }
                    ring.push(map(start));
                }
                for p in &pts { ring.push(map(*p)); }
                last = pts.last().copied();
            }
            if ring.len() >= 3 { rings.push(ring); }
        }
        pen += f.h_advance_unscaled(id);
    }
    // Drop the closing duplicate of each ring.
    for r in &mut rings { if r.len() > 1 { let (a, b) = (r[0], r[r.len() - 1]); if (a.0 - b.0).abs() < 1e-5 && (a.1 - b.1).abs() < 1e-5 { r.pop(); } } }
    rings
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stroke_glyphs_parse_and_scale() {
        for c in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,-+/:=%#()_*°!'\"<>×?".chars() { assert!(!parse(glyph(c)).is_empty(), "glyph {c}"); }
        let s = stroke_text("H", 10.0, 20.0, 3.0);
        assert_eq!(s.len(), 3);
        assert_eq!(s[0][0], (10.0, 20.0));
        assert_eq!(s[0][1], (10.0, 23.0));
        assert!((stroke_width("HH", 3.0) - (5.5 + 4.0) * 0.5).abs() < 1e-5);
    }
    #[test]
    fn outline_font_cap_height() {
        let rings = outline_rings("H", 0.0, 0.0, 5.0);
        assert!(!rings.is_empty());
        let top = rings.iter().flatten().map(|p| p.1).fold(f32::MIN, f32::max);
        assert!((top - 5.0).abs() < 0.05, "cap top {top}");
        assert!(text_width("HELLO", 5.0, TextFont::Outline, true) > 10.0);
    }
}
// =============================================================================
// End of file: font.rs
// Location:    laser-cnc-controller/src/font.rs
// Created:     2026-10-17 17:35:38 UTC
// =============================================================================
