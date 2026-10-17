// =============================================================================
// File:     lib.rs
// Location: laser-cnc-controller/crates/motion-planner/src/lib.rs
// Version:  v0.1.1
// Comments: no_std motion planner modelled on GRBL 0.9's planner.
//           Units: millimetres, seconds, mm/s, mm/s². Feed and max rate are
//           given in mm/min (G-code and GRBL $110/$111 units) and converted.
//           Allocation-free core (firmware-ready):
//           - MotionConfig: $110/$111 max rate, $120/$121 acceleration, $11
//             junction deviation, look-ahead depth; read from GRBL "$$" lines.
//           - kinematics(): length, unit vector, nominal speed and
//             acceleration of one straight move, limited per axis as GRBL's
//             limit_value_by_axis_maximum().
//           - junction_speed(): GRBL 0.9 junction-deviation corner speed.
//           - profile(): trapezoid (or triangle) for one move: entry, cruise,
//             exit speed, phase distances and times; distance/speed at time t.
//           With the "alloc" feature:
//           - plan(): whole-program planning with an optional look-ahead
//             window (GRBL's block buffer), backward and forward passes, full
//             stop before blocks flagged stop_before (GRBL 0.9 waits for the
//             planner to empty on M3/M4/M5, G92 and '$' lines).
//           - Trajectory::sample(t): position, speed, segment and laser state
//             at time t. Used for the on-screen head and the time estimate.
//           - gcode module: G-code lines to Blocks.
//           Hypotheses until checked against the GRBL 0.9 source / machine:
//           look-ahead depth 18 blocks; full stop on every M3/M4/M5 line.
//           v0.1.1:
//           - Profile::time_at(d): time at which a move has covered distance
//             d (inverse of Profile::at).
//           - Trajectory::locate(pos, around, window, weight): the time on the
//             trajectory whose position is nearest to a reported position,
//             searched within around ± window seconds; candidates further in
//             time from `around` cost `weight` mm per second, so parallel
//             passes close together do not swap. Used to keep an animation
//             clock in step with the controller's position reports.
// =============================================================================
#![no_std]
#[cfg(feature = "alloc")]
extern crate alloc;
#[cfg(test)]
extern crate std;

#[cfg(feature = "alloc")]
pub mod gcode;

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Number of planned axes (X, Y).
pub const AXES: usize = 2;
/// Moves shorter than this are dropped (mm).
pub const MIN_LENGTH: f32 = 1e-6;
/// Lowest feed used when a move has no usable feed (mm/min).
const MIN_FEED: f32 = 1.0;
/// GRBL clamps the junction cosine to keep the formula finite.
const COS_LIMIT: f32 = 0.999_999;

fn sqrt(v: f32) -> f32 {
    libm::sqrtf(v.max(0.0))
}

/// Machine limits used by the planner (GRBL $ settings).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionConfig {
    /// $110 / $111: maximum rate per axis, mm/min.
    pub max_rate: [f32; AXES],
    /// $120 / $121: acceleration per axis, mm/s².
    pub accel: [f32; AXES],
    /// $11: junction deviation, mm.
    pub junction_deviation: f32,
    /// Look-ahead depth in blocks (GRBL block buffer). 0 = whole program.
    pub lookahead: usize,
}
impl Default for MotionConfig {
    fn default() -> Self {
        Self { max_rate: [3000.0, 3000.0], accel: [10.0, 10.0], junction_deviation: 0.010, lookahead: 18 }
    }
}
impl MotionConfig {
    /// Values from GRBL "$$" lines (e.g. "$120=10.000 (x accel, mm/sec^2)").
    /// Settings that are missing or unreadable keep the value from `self`.
    pub fn with_grbl_settings<'a, I: IntoIterator<Item = &'a str>>(mut self, lines: I) -> Self {
        for line in lines {
            let Some(rest) = line.trim().strip_prefix('$') else { continue };
            let Some((key, value)) = rest.split_once('=') else { continue };
            let number = value.split_whitespace().next().unwrap_or("");
            let Ok(v) = number.parse::<f32>() else { continue };
            if !(v.is_finite() && v > 0.0) {
                continue;
            }
            match key.trim() {
                "110" => self.max_rate[0] = v,
                "111" => self.max_rate[1] = v,
                "120" => self.accel[0] = v,
                "121" => self.accel[1] = v,
                "11" => self.junction_deviation = v,
                _ => {}
            }
        }
        self
    }
}

/// Smallest value[i] / |unit[i]| over the axes that move (GRBL limit_value_by_axis_maximum).
fn limit_by_axis(values: &[f32; AXES], unit: &[f32; AXES]) -> f32 {
    let mut limit = f32::MAX;
    for i in 0..AXES {
        let u = unit[i].abs();
        if u > 1e-9 {
            limit = limit.min(values[i] / u);
        }
    }
    limit
}

/// Geometry and limits of one straight move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Kinematics {
    /// Length, mm.
    pub length: f32,
    /// Direction, unit vector.
    pub unit: [f32; AXES],
    /// Target (nominal) speed, mm/s: the feed, capped by the axis max rates.
    pub nominal: f32,
    /// Acceleration along the move, mm/s², limited per axis.
    pub accel: f32,
}

/// Kinematics of a move from `start` to `end`. `feed` in mm/min; a rapid uses the max rate.
/// None for a move shorter than MIN_LENGTH.
pub fn kinematics(cfg: &MotionConfig, start: [f32; AXES], end: [f32; AXES], feed: f32, rapid: bool) -> Option<Kinematics> {
    let mut delta = [0.0f32; AXES];
    let mut sq = 0.0f32;
    for i in 0..AXES {
        delta[i] = end[i] - start[i];
        sq += delta[i] * delta[i];
    }
    let length = sqrt(sq);
    if length.is_nan() || length < MIN_LENGTH {
        return None;
    }
    let mut unit = [0.0f32; AXES];
    for i in 0..AXES {
        unit[i] = delta[i] / length;
    }
    let rate_limit = limit_by_axis(&cfg.max_rate, &unit) / 60.0;
    let nominal = if rapid { rate_limit } else { (feed.max(MIN_FEED) / 60.0).min(rate_limit) };
    let accel = limit_by_axis(&cfg.accel, &unit).max(1e-6);
    Some(Kinematics { length, unit, nominal: nominal.max(1e-6), accel })
}

/// Highest speed (mm/s) through the corner between `prev` and `next` (GRBL 0.9 junction deviation),
/// capped by both moves' nominal speeds. A full reversal gives 0; a straight continuation is
/// limited only by the nominal speeds.
pub fn junction_speed(cfg: &MotionConfig, prev: &Kinematics, next: &Kinematics) -> f32 {
    let mut dot = 0.0f32;
    for i in 0..AXES {
        dot += prev.unit[i] * next.unit[i];
    }
    let cos_theta = -dot;
    if cos_theta > COS_LIMIT {
        return 0.0;
    }
    let cos_theta = cos_theta.max(-COS_LIMIT);
    let sin_half = sqrt(0.5 * (1.0 - cos_theta));
    let v_sq = next.accel * cfg.junction_deviation * sin_half / (1.0 - sin_half);
    sqrt(v_sq).min(prev.nominal).min(next.nominal)
}

/// Speed profile of one move: accelerate, cruise, decelerate (a phase may be empty).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Profile {
    pub length: f32,
    pub entry: f32,
    pub cruise: f32,
    pub exit: f32,
    pub d_accel: f32,
    pub d_cruise: f32,
    pub d_decel: f32,
    pub t_accel: f32,
    pub t_cruise: f32,
    pub t_decel: f32,
}

/// Time to cover `d` while the speed changes linearly from v0 to v1 (constant acceleration).
fn phase_time(d: f32, v0: f32, v1: f32) -> f32 {
    let s = v0 + v1;
    if d <= 0.0 || s <= 0.0 {
        0.0
    } else {
        2.0 * d / s
    }
}

/// Trapezoid for a move of `length` (mm) entered at `entry`, aiming for `nominal` and leaving at
/// `exit` (mm/s), with acceleration `accel` (mm/s²). When the move is too short to reach
/// `nominal` the profile is a triangle. Entry and exit are expected to be reachable (the planner
/// guarantees it); if they are not, the profile is clamped so distances and times stay consistent.
pub fn profile(length: f32, entry: f32, nominal: f32, exit: f32, accel: f32) -> Profile {
    let a = accel.max(1e-6);
    let length = length.max(0.0);
    let nominal = nominal.max(1e-6);
    let entry = entry.clamp(0.0, nominal);
    let exit = exit.clamp(0.0, nominal);
    let d_acc_full = (nominal * nominal - entry * entry) / (2.0 * a);
    let d_dec_full = (nominal * nominal - exit * exit) / (2.0 * a);
    let (cruise, d_accel, d_cruise, d_decel) = if d_acc_full + d_dec_full <= length {
        (nominal, d_acc_full, length - d_acc_full - d_dec_full, d_dec_full)
    } else {
        let peak = sqrt(0.5 * (entry * entry + exit * exit) + a * length).max(entry).max(exit);
        let d_accel = ((peak * peak - entry * entry) / (2.0 * a)).clamp(0.0, length);
        (peak, d_accel, 0.0, length - d_accel)
    };
    Profile {
        length,
        entry,
        cruise,
        exit,
        d_accel,
        d_cruise,
        d_decel,
        t_accel: phase_time(d_accel, entry, cruise),
        t_cruise: if cruise > 0.0 { d_cruise / cruise } else { 0.0 },
        t_decel: phase_time(d_decel, cruise, exit),
    }
}

impl Profile {
    /// Total time of the move, s.
    pub fn duration(&self) -> f32 {
        self.t_accel + self.t_cruise + self.t_decel
    }
    /// Distance travelled and speed at time `t` (s) into the move, clamped to the move.
    pub fn at(&self, t: f32) -> (f32, f32) {
        let t = t.max(0.0);
        // Within a phase the speed changes linearly: d = v0·τ + ½·k·τ², k = (v1 − v0) / phase time.
        let ramp = |tau: f32, dur: f32, v0: f32, v1: f32| -> (f32, f32) {
            if dur <= 0.0 {
                return (0.0, v1);
            }
            let k = (v1 - v0) / dur;
            (v0 * tau + 0.5 * k * tau * tau, v0 + k * tau)
        };
        if t < self.t_accel {
            return ramp(t, self.t_accel, self.entry, self.cruise);
        }
        let t2 = t - self.t_accel;
        if t2 < self.t_cruise {
            return (self.d_accel + self.cruise * t2, self.cruise);
        }
        let t3 = t2 - self.t_cruise;
        if t3 < self.t_decel {
            let (d, v) = ramp(t3, self.t_decel, self.cruise, self.exit);
            return ((self.d_accel + self.d_cruise + d).min(self.length), v);
        }
        (self.length, self.exit)
    }
    /// Time (s) at which the move has covered distance `d` (clamped to the move). Inverse of `at`.
    pub fn time_at(&self, d: f32) -> f32 {
        let d = d.clamp(0.0, self.length);
        // Within a ramp: d = v0·τ + ½·k·τ², solved for τ.
        let ramp = |d: f32, dur: f32, v0: f32, v1: f32| -> f32 {
            if dur <= 0.0 {
                return 0.0;
            }
            let k = (v1 - v0) / dur;
            if k.abs() < 1e-9 {
                return if v0 > 0.0 { (d / v0).min(dur) } else { 0.0 };
            }
            ((-v0 + sqrt(v0 * v0 + 2.0 * k * d)) / k).clamp(0.0, dur)
        };
        if d <= self.d_accel {
            return ramp(d, self.t_accel, self.entry, self.cruise);
        }
        let d2 = d - self.d_accel;
        if d2 <= self.d_cruise {
            return self.t_accel + if self.cruise > 0.0 { d2 / self.cruise } else { 0.0 };
        }
        self.t_accel + self.t_cruise + ramp(d2 - self.d_cruise, self.t_decel, self.cruise, self.exit)
    }
}

/// One move handed to the planner (program coordinates).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Block {
    pub start: [f32; AXES],
    pub end: [f32; AXES],
    /// Feed, mm/min (ignored for rapids).
    pub feed: f32,
    /// G0: runs at the axis max rate.
    pub rapid: bool,
    /// Laser (spindle) output on during this move.
    pub laser: bool,
    /// Laser power (S value) during this move.
    pub power: u32,
    /// The controller comes to a full stop before this move (planner emptied).
    pub stop_before: bool,
    /// Index of the program line this move came from.
    pub source: usize,
}

/// A planned move: the block, its kinematics, its speed profile and its start time.
#[cfg(feature = "alloc")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    pub block: Block,
    pub kin: Kinematics,
    pub profile: Profile,
    /// Time from the start of the program, s.
    pub t_start: f32,
}

/// State of the machine at one instant of a trajectory.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Position, program coordinates (mm).
    pub pos: [f32; AXES],
    /// Speed along the path, mm/s.
    pub speed: f32,
    /// Index of the segment being executed (segments.len() once finished).
    pub segment: usize,
    /// Laser output on at this instant.
    pub laser: bool,
    /// The program has finished.
    pub done: bool,
}

/// A planned program: every move with its profile and timing.
#[cfg(feature = "alloc")]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Trajectory {
    /// Position before the first move.
    pub start: [f32; AXES],
    pub segments: Vec<Segment>,
}

#[cfg(feature = "alloc")]
impl Trajectory {
    /// Total time of the program, s.
    pub fn duration(&self) -> f32 {
        self.segments.last().map_or(0.0, |s| s.t_start + s.profile.duration())
    }
    /// Position, speed, segment and laser state at time `t` (s) from the start.
    pub fn sample(&self, t: f32) -> Sample {
        let Some(last) = self.segments.last() else {
            return Sample { pos: self.start, speed: 0.0, segment: 0, laser: false, done: true };
        };
        if t >= self.duration() {
            return Sample { pos: last.block.end, speed: 0.0, segment: self.segments.len(), laser: false, done: true };
        }
        let t = t.max(0.0);
        // Last segment starting at or before t.
        let i = self.segments.partition_point(|s| s.t_start <= t).saturating_sub(1);
        let s = &self.segments[i];
        let (d, v) = s.profile.at(t - s.t_start);
        let mut pos = s.block.start;
        for (k, p) in pos.iter_mut().enumerate() {
            *p += s.kin.unit[k] * d;
        }
        Sample { pos, speed: v, segment: i, laser: s.block.laser, done: false }
    }
    /// Time on the trajectory whose position is nearest to `pos`, searched within `around` ± `window`
    /// seconds. Cost of a candidate = distance (mm) + `weight` × |time − around| (mm/s), so of two
    /// equally near points the one closer in time wins. Returns (time, distance) or None when no
    /// move lies in the window.
    pub fn locate(&self, pos: [f32; AXES], around: f32, window: f32, weight: f32) -> Option<(f32, f32)> {
        let (t0, t1) = (around - window, around + window);
        let first = self.segments.partition_point(|s| s.t_start + s.profile.duration() < t0);
        let mut best: Option<(f32, f32, f32)> = None;
        for s in self.segments[first..].iter().take_while(|s| s.t_start <= t1) {
            // Nearest point on the move's line, as distance along it.
            let mut along = 0.0f32;
            for k in 0..AXES {
                along += (pos[k] - s.block.start[k]) * s.kin.unit[k];
            }
            let along = along.clamp(0.0, s.kin.length);
            let mut sq = 0.0f32;
            for k in 0..AXES {
                let p = s.block.start[k] + s.kin.unit[k] * along;
                sq += (pos[k] - p) * (pos[k] - p);
            }
            let dist = sqrt(sq);
            let t = (s.t_start + s.profile.time_at(along)).clamp(t0.max(0.0), t1);
            let cost = dist + weight * (t - around).abs();
            if best.is_none_or(|b| cost < b.2) {
                best = Some((t, dist, cost));
            }
        }
        best.map(|b| (b.0, b.1))
    }
}

/// Plan a whole program. Zero-length blocks are dropped (a stop they carry moves to the next move).
/// Entry speed of each move is limited by: the corner (junction deviation), a full stop where
/// `stop_before` is set or at the start, decelerating to a stop at the end of the look-ahead
/// window (cfg.lookahead blocks, 0 = whole program), and what the previous move can accelerate to.
#[cfg(feature = "alloc")]
pub fn plan(cfg: &MotionConfig, blocks: &[Block]) -> Trajectory {
    let start = blocks.first().map_or([0.0; AXES], |b| b.start);
    let mut kept: Vec<(Block, Kinematics)> = Vec::with_capacity(blocks.len());
    let mut carry_stop = false;
    for b in blocks {
        match kinematics(cfg, b.start, b.end, b.feed, b.rapid) {
            Some(k) => {
                let mut b = *b;
                b.stop_before |= carry_stop;
                carry_stop = false;
                kept.push((b, k));
            }
            None => carry_stop |= b.stop_before,
        }
    }
    let n = kept.len();
    if n == 0 {
        return Trajectory { start, segments: Vec::new() };
    }
    // Corner / stop limit on each entry.
    let mut max_entry: Vec<f32> = Vec::with_capacity(n);
    for i in 0..n {
        let v = if i == 0 || kept[i].0.stop_before { 0.0 } else { junction_speed(cfg, &kept[i - 1].1, &kept[i].1) };
        max_entry.push(v);
    }
    // Backward passes: the machine must be able to stop by the end of the look-ahead window.
    let reach = |lim: f32, k: &Kinematics| sqrt(lim * lim + 2.0 * k.accel * k.length);
    let mut entry_limit = vec_f32(n);
    let window = if cfg.lookahead == 0 || cfg.lookahead >= n { n } else { cfg.lookahead };
    if window == n {
        let mut lim = 0.0f32;
        for j in (0..n).rev() {
            lim = max_entry[j].min(reach(lim, &kept[j].1));
            entry_limit[j] = lim;
        }
    } else {
        for (i, limit) in entry_limit.iter_mut().enumerate() {
            let last = (i + window - 1).min(n - 1);
            let mut lim = 0.0f32;
            for j in (i..=last).rev() {
                lim = max_entry[j].min(reach(lim, &kept[j].1));
            }
            *limit = lim;
        }
    }
    // Forward pass: entry speed is what the previous move can accelerate to.
    let mut entry = vec_f32(n);
    for i in 1..n {
        entry[i] = entry_limit[i].min(reach(entry[i - 1], &kept[i - 1].1));
    }
    let mut segments = Vec::with_capacity(n);
    let mut t = 0.0f32;
    for i in 0..n {
        let (block, kin) = kept[i];
        let exit = if i + 1 < n { entry[i + 1] } else { 0.0 };
        let p = profile(kin.length, entry[i], kin.nominal, exit, kin.accel);
        segments.push(Segment { block, kin, profile: p, t_start: t });
        t += p.duration();
    }
    Trajectory { start, segments }
}

#[cfg(feature = "alloc")]
fn vec_f32(n: usize) -> Vec<f32> {
    let mut v = Vec::with_capacity(n);
    v.resize(n, 0.0);
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;

    fn cfg() -> MotionConfig {
        MotionConfig { max_rate: [6000.0, 6000.0], accel: [10.0, 10.0], junction_deviation: 0.010, lookahead: 0 }
    }
    fn block(start: [f32; 2], end: [f32; 2], feed: f32) -> Block {
        Block { start, end, feed, rapid: false, laser: true, power: 300, stop_before: false, source: 0 }
    }
    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn right_angle_corner_speed() {
        let c = cfg();
        let a = kinematics(&c, [0.0, 0.0], [20.0, 0.0], 600.0, false).unwrap();
        let b = kinematics(&c, [20.0, 0.0], [20.0, 10.0], 600.0, false).unwrap();
        assert!(close(junction_speed(&c, &a, &b), 0.4913, 0.001));
    }

    #[test]
    fn reversal_and_straight() {
        let c = cfg();
        let a = kinematics(&c, [0.0, 0.0], [10.0, 0.0], 600.0, false).unwrap();
        let back = kinematics(&c, [10.0, 0.0], [0.0, 0.0], 600.0, false).unwrap();
        let on = kinematics(&c, [10.0, 0.0], [20.0, 0.0], 600.0, false).unwrap();
        assert_eq!(junction_speed(&c, &a, &back), 0.0);
        assert!(close(junction_speed(&c, &a, &on), 10.0, 1e-4));
    }

    #[test]
    fn rectangle_first_edge_matches_hand_calculation() {
        // 20 x 10 mm rectangle at F600, 10 mm/s², junction deviation 0.010 mm.
        let blocks = vec![
            block([0.0, 0.0], [20.0, 0.0], 600.0),
            block([20.0, 0.0], [20.0, 10.0], 600.0),
            block([20.0, 10.0], [0.0, 10.0], 600.0),
            block([0.0, 10.0], [0.0, 0.0], 600.0),
        ];
        let t = plan(&cfg(), &blocks);
        let p = t.segments[0].profile;
        assert!(close(p.entry, 0.0, 1e-6));
        assert!(close(p.cruise, 10.0, 1e-4));
        assert!(close(p.exit, 0.4913, 0.001));
        assert!(close(p.t_accel, 1.0, 1e-3));
        assert!(close(p.duration(), 2.952, 0.002));
        // Distances add up and the last move ends stopped.
        for s in &t.segments {
            assert!(close(s.profile.d_accel + s.profile.d_cruise + s.profile.d_decel, s.kin.length, 1e-3));
        }
        assert_eq!(t.segments[3].profile.exit, 0.0);
    }

    #[test]
    fn short_move_is_a_triangle() {
        let p = profile(2.0, 0.0, 10.0, 0.0, 10.0);
        assert!(close(p.cruise, sqrt(20.0), 1e-4));
        assert!(close(p.d_cruise, 0.0, 1e-6));
        assert!(close(p.duration(), 2.0 * sqrt(20.0) / 10.0, 1e-4));
    }

    #[test]
    fn sample_follows_the_path_and_ends_at_the_end() {
        let blocks = vec![block([0.0, 0.0], [20.0, 0.0], 600.0), block([20.0, 0.0], [20.0, 10.0], 600.0)];
        let t = plan(&cfg(), &blocks);
        let mid = t.sample(t.segments[0].profile.duration() * 0.5);
        assert!(close(mid.pos[1], 0.0, 1e-6) && mid.pos[0] > 5.0 && mid.pos[0] < 15.0);
        let corner = t.sample(t.segments[1].t_start);
        assert!(close(corner.pos[0], 20.0, 1e-3) && close(corner.pos[1], 0.0, 1e-3));
        let end = t.sample(t.duration() + 1.0);
        assert!(end.done && end.pos == [20.0, 10.0]);
    }

    #[test]
    fn stop_before_forces_a_full_stop() {
        let mut second = block([10.0, 0.0], [20.0, 0.0], 600.0);
        second.stop_before = true;
        let t = plan(&cfg(), &[block([0.0, 0.0], [10.0, 0.0], 600.0), second]);
        assert_eq!(t.segments[0].profile.exit, 0.0);
        assert_eq!(t.segments[1].profile.entry, 0.0);
    }

    #[test]
    fn short_lookahead_slows_a_long_straight() {
        // Twenty 1 mm collinear moves. With a 2-block window the machine must be able to stop
        // within 2 mm at all times, so it cruises slower than with the whole program planned.
        let blocks: std::vec::Vec<Block> = (0..20).map(|i| block([i as f32, 0.0], [(i + 1) as f32, 0.0], 3000.0)).collect();
        let mut c = cfg();
        let whole = plan(&c, &blocks).duration();
        c.lookahead = 2;
        let windowed = plan(&c, &blocks).duration();
        assert!(windowed > whole);
    }

    #[test]
    fn time_at_inverts_at() {
        let p = profile(20.0, 0.0, 10.0, 0.4913, 10.0);
        for i in 0..=40 {
            let d = 20.0 * i as f32 / 40.0;
            let (dd, _) = p.at(p.time_at(d));
            assert!(close(dd, d, 1e-3), "d {d} -> {dd}");
        }
    }

    #[test]
    fn locate_finds_the_reported_point_and_prefers_nearby_time() {
        // Two parallel passes 0.1 mm apart, one after the other.
        let mut back = block([20.0, 0.1], [0.0, 0.1], 600.0);
        back.stop_before = true;
        let t = plan(&cfg(), &[block([0.0, 0.0], [20.0, 0.0], 600.0), back]);
        let mid = t.segments[0].profile.duration() * 0.5;
        let s = t.sample(mid);
        let (found, dist) = t.locate(s.pos, mid, 5.0, 0.02).unwrap();
        assert!(close(found, mid, 1e-3) && dist < 1e-4);
        // A point on the second pass, searched around a time on the first pass that is
        // close in position: the time weight keeps the first pass unless the distance says otherwise.
        let second = t.segments[1].t_start + 0.5;
        let p2 = t.sample(second).pos;
        let (f2, _) = t.locate(p2, second - 0.2, 5.0, 0.02).unwrap();
        assert!(close(f2, second, 0.02));
    }

    #[test]
    fn settings_from_grbl() {
        let lines = ["$11=0.020 (junction deviation, mm)", "$110=5000.000 (x max rate, mm/min)", "$121=250.000 (y accel, mm/sec^2)", "$999=x"];
        let c = MotionConfig::default().with_grbl_settings(lines.iter().copied());
        assert_eq!(c.junction_deviation, 0.020);
        assert_eq!(c.max_rate[0], 5000.0);
        assert_eq!(c.accel[1], 250.0);
        assert_eq!(c.accel[0], 10.0);
    }

    #[test]
    fn diagonal_is_limited_by_the_slower_axis() {
        let c = MotionConfig { max_rate: [6000.0, 600.0], accel: [100.0, 10.0], ..cfg() };
        let k = kinematics(&c, [0.0, 0.0], [10.0, 10.0], 6000.0, false).unwrap();
        // Y is limited to 10 mm/s; along a 45° move that is 10/0.7071 mm/s.
        assert!(close(k.nominal, 10.0 / core::f32::consts::FRAC_1_SQRT_2, 1e-3));
        assert!(close(k.accel, 10.0 / core::f32::consts::FRAC_1_SQRT_2, 1e-3));
    }
}
// =============================================================================
// End of file: lib.rs
// Location:    laser-cnc-controller/crates/motion-planner/src/lib.rs
// Created:     2026-10-17 22:04:20 UTC
// =============================================================================
