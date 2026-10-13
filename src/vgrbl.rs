// =============================================================================
// File:     vgrbl.rs
// Location: laser-cnc-controller/src/vgrbl.rs
// Version:  v0.1.0
// Comments: Virtual GRBL 0.9i controller for Simulate mode. Stands in for the
//           serial port behind the same serial worker (src/machine.rs Port),
//           so streaming, flow control, status, jog, zero, trace, pause,
//           reset and unlock run through the real code path.
//           Model:
//           - Serial RX buffer 128 bytes (bytes beyond it are dropped and
//             counted as overflows). Planner 16 blocks.
//           - Lines leave the RX buffer when the planner has room. M3/M4/M5,
//             G92 and '$' commands wait for the planner to empty first.
//           - Motion at constant speed (no acceleration): G1 at F, G0 at the
//             $110 rate (3000 mm/min). Time scaled by the Simulate speed.
//           - G90/G91, G92 (X/Y), G0/G1, F, S, M3/M4/M5, G21. G10 and other
//             unsupported words answer "error: Unsupported command" (as the
//             real controller does for G10 L20).
//           - '$' read-only queries: $$, $#, $G, $I, $N, $X (unlock), $H
//             ("error: Setting disabled", no homing). Settings writes refused.
//           - Real-time: '?' status (0.9 format, MPos and WPos), '!' hold,
//             '@' safety door (laser off), '~' resume (laser restored),
//             Ctrl-X reset (banner; during motion: ALARM: Abort during cycle,
//             then Alarm lock until $X).
//           - Laser state and speed shared with the app (SimShared).
//           - No banner on open: like the Nano on Windows (DTR low), the app's
//             soft reset produces it.
// =============================================================================
use std::collections::VecDeque;
use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool,AtomicU32,Ordering};
use std::time::Instant;

const RX_SIZE:usize=128;
const PLANNER:usize=16;
const RAPID:f32=3000.0;
const BANNER:&str="Grbl 0.9i ['$' for help]";
const SETTINGS:[&str;31]=[
    "$0=10 (step pulse, usec)","$1=25 (step idle delay, msec)","$2=0 (step port invert mask:00000000)",
    "$3=0 (dir port invert mask:00000000)","$4=0 (step enable invert, bool)","$5=0 (limit pins invert, bool)",
    "$6=0 (probe pin invert, bool)","$10=3 (status report mask:00000011)","$11=0.010 (junction deviation, mm)",
    "$12=0.002 (arc tolerance, mm)","$13=0 (report inches, bool)","$20=0 (soft limits, bool)",
    "$21=0 (hard limits, bool)","$22=0 (homing cycle, bool)","$23=0 (homing dir invert mask:00000000)",
    "$24=25.000 (homing feed, mm/min)","$25=500.000 (homing seek, mm/min)","$26=250 (homing debounce, msec)",
    "$27=1.000 (homing pull-off, mm)","$100=80.000 (x, step/mm)","$101=80.000 (y, step/mm)",
    "$102=250.000 (z, step/mm)","$110=3000.000 (x max rate, mm/min)","$111=3000.000 (y max rate, mm/min)",
    "$112=500.000 (z max rate, mm/min)","$120=10.000 (x accel, mm/sec^2)","$121=10.000 (y accel, mm/sec^2)",
    "$122=10.000 (z accel, mm/sec^2)","$130=380.000 (x max travel, mm)","$131=280.000 (y max travel, mm)",
    "$132=200.000 (z max travel, mm)",
];

/// State shared between the virtual controller and the app.
#[derive(Clone)]
pub struct SimShared{pub laser:Arc<AtomicBool>, speed:Arc<AtomicU32>, pub overflows:Arc<AtomicU32>}
impl SimShared{
    pub fn new(speed:f32)->Self{Self{laser:Arc::new(AtomicBool::new(false)),speed:Arc::new(AtomicU32::new(speed.to_bits())),overflows:Arc::new(AtomicU32::new(0))}}
    pub fn set_speed(&self,s:f32){self.speed.store(s.to_bits(),Ordering::Relaxed);}
    fn speed(&self)->f32{f32::from_bits(self.speed.load(Ordering::Relaxed))}
}

struct Block{target:(f32,f32),feed:f32}

pub struct VirtualGrbl{
    shared:SimShared, last:Instant,
    rx:VecDeque<u8>, out:VecDeque<u8>, planner:VecDeque<Block>,
    pos:(f32,f32), planned:(f32,f32), g92:(f32,f32),
    rapid_mode:bool, absolute:bool, feed:f32, s:u32, spindle:bool, door_saved:bool,
    hold:bool, door:bool, alarm:bool,
}
impl VirtualGrbl{
    pub fn new(shared:SimShared)->Self{
        Self{shared,last:Instant::now(),rx:VecDeque::new(),out:VecDeque::new(),planner:VecDeque::new(),
            pos:(0.0,0.0),planned:(0.0,0.0),g92:(0.0,0.0),rapid_mode:true,absolute:true,feed:0.0,s:0,spindle:false,door_saved:false,
            hold:false,door:false,alarm:false}
    }
    fn say(&mut self,s:&str){self.out.extend(s.as_bytes());self.out.extend(b"\r\n");}
    fn set_laser(&mut self,on:bool){self.spindle=on;self.shared.laser.store(on&&!self.door,Ordering::Relaxed);}
    fn state(&self)->&'static str{
        if self.alarm{"Alarm"}else if self.door{"Door"}else if self.hold{"Hold"}else if !self.planner.is_empty(){"Run"}else{"Idle"}
    }
    fn status(&mut self){
        let (x,y)=self.pos;let (wx,wy)=(x-self.g92.0,y-self.g92.1);
        let s=format!("<{},MPos:{x:.3},{y:.3},0.000,WPos:{wx:.3},{wy:.3},0.000>",self.state());
        self.say(&s);
    }
    fn realtime(&mut self,b:u8){
        match b{
            b'?'=>self.status(),
            b'!'=>{if !self.planner.is_empty(){self.hold=true;}}
            b'@'=>{if !self.door{self.door=true;self.door_saved=self.spindle;self.shared.laser.store(false,Ordering::Relaxed);}}
            b'~'=>{if self.door{self.door=false;let on=self.door_saved;self.set_laser(on);}self.hold=false;}
            0x18=>self.reset(),
            _=>{}
        }
    }
    fn reset(&mut self){
        let moving=!self.planner.is_empty();
        self.rx.clear();self.planner.clear();self.planned=self.pos;
        self.hold=false;self.door=false;self.set_laser(false);
        self.rapid_mode=true;self.absolute=true;self.g92=(0.0,0.0);
        if moving{self.alarm=true;self.say("ALARM: Abort during cycle");}
        self.say(BANNER);
        if self.alarm{self.say("['$H'|'$X' to unlock]");}
    }
    /// Advance motion and parse buffered lines. Called whenever the worker polls.
    fn tick(&mut self){
        let now=Instant::now();
        let mut dt=(now-self.last).as_secs_f32()*self.shared.speed().max(0.1);
        self.last=now;
        if !(self.hold||self.door||self.alarm){
            while dt>0.0{
                let Some(b)=self.planner.front() else {break;};
                let (tx,ty)=b.target;let v=(b.feed/60.0).max(0.01);
                let (dx,dy)=(tx-self.pos.0,ty-self.pos.1);let d=(dx*dx+dy*dy).sqrt();
                let t=d/v;
                if t<=dt{self.pos=(tx,ty);self.planner.pop_front();dt-=t;}
                else{let k=v*dt/d;self.pos=(self.pos.0+dx*k,self.pos.1+dy*k);dt=0.0;}
            }
        }
        self.process_lines();
    }
    fn process_lines(&mut self){
        loop{
            let Some(end)=self.rx.iter().position(|&b|b==b'\n') else {return;};
            let raw:Vec<u8>=self.rx.iter().take(end).copied().collect();
            let line=clean(&String::from_utf8_lossy(&raw));
            let sync=line.starts_with('$')||line.contains("M3")||line.contains("M4")||line.contains("M5")||line.contains("G92");
            if sync&&!self.planner.is_empty(){return;}
            if self.planner.len()>=PLANNER{return;}
            self.rx.drain(..=end);
            let reply=self.execute(&line);
            self.say(&reply);
        }
    }
    fn execute(&mut self,line:&str)->String{
        if line.is_empty(){return "ok".into();}
        if let Some(cmd)=line.strip_prefix('$'){return self.system(cmd);}
        if self.alarm{return "error: Alarm lock".into();}
        // Words: letter + number.
        let mut words:Vec<(char,f32)>=vec![];
        let b:Vec<char>=line.chars().collect();let mut i=0;
        while i<b.len(){
            let l=b[i];if !l.is_ascii_alphabetic(){return "error: Expected command letter".into();}
            let mut j=i+1;while j<b.len()&&(b[j].is_ascii_digit()||b[j]=='.'||b[j]=='-'||b[j]=='+'){j+=1;}
            let n:String=b[i+1..j].iter().collect();
            let Ok(v)=n.parse::<f32>() else {return "error: Bad number format".into();};
            words.push((l,v));i=j;
        }
        let (mut x,mut y)=(None,None);let mut g92=false;let mut spindle=None;
        for &(l,v) in &words{
            match l{
                'G'=>match (v*10.0).round() as i32{
                    0=>self.rapid_mode=true, 10=>self.rapid_mode=false,
                    170|210|540|940=>{} 900=>self.absolute=true, 910=>self.absolute=false, 920=>g92=true,
                    _=>return "error: Unsupported command".into(),
                },
                'M'=>match v as i32{3|4=>spindle=Some(true),5=>spindle=Some(false),0|2|8|9|30=>{},_=>return "error: Unsupported command".into()},
                'X'=>x=Some(v),'Y'=>y=Some(v),'Z'=>{}
                'F'=>self.feed=v,'S'=>self.s=v.max(0.0) as u32,
                _=>return "error: Unsupported command".into(),
            }
        }
        if let Some(on)=spindle{self.set_laser(on);}
        if g92{
            // Work position of the current point becomes the given value.
            if let Some(v)=x{self.g92.0=self.planned.0-v;}
            if let Some(v)=y{self.g92.1=self.planned.1-v;}
            return "ok".into();
        }
        if x.is_some()||y.is_some(){
            if !self.rapid_mode&&self.feed<=0.0{return "error: Undefined feed rate".into();}
            let t=if self.absolute{(x.map_or(self.planned.0,|v|v+self.g92.0),y.map_or(self.planned.1,|v|v+self.g92.1))}
                  else{(self.planned.0+x.unwrap_or(0.0),self.planned.1+y.unwrap_or(0.0))};
            let feed=if self.rapid_mode{RAPID}else{self.feed};
            if (t.0-self.planned.0).abs()>1e-6||(t.1-self.planned.1).abs()>1e-6{self.planner.push_back(Block{target:t,feed});}
            self.planned=t;
        }
        "ok".into()
    }
    fn system(&mut self,cmd:&str)->String{
        match cmd{
            ""=>{self.say("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $C $X $H ~ ! ? ctrl-x]");}
            "$"=>{for s in SETTINGS{self.say(s);}}
            "#"=>{let (gx,gy)=self.g92;for k in ["G54","G55","G56","G57","G58","G59","G28","G30"]{self.say(&format!("[{k}:0.000,0.000,0.000]"));}
                  self.say(&format!("[G92:{gx:.3},{gy:.3},0.000]"));self.say("[TLO:0.000]");self.say("[PRB:0.000,0.000,0.000]");}
            "G"=>{let s=format!("[{} G54 G17 G21 {} G94 M0 {} M9 T0 F{}. S{}.]",if self.rapid_mode{"G0"}else{"G1"},if self.absolute{"G90"}else{"G91"},
                     if self.spindle{"M3"}else{"M5"},self.feed as i32,self.s);self.say(&s);}
            "I"=>self.say("[0.9i.20150620:]"),
            "N"=>{self.say("$N0=");self.say("$N1=");}
            "X"=>{if self.alarm{self.alarm=false;self.say("[Caution: Unlocked]");}}
            "H"=>return "error: Setting disabled".into(),
            _=>return "error: Unsupported command".into(),
        }
        "ok".into()
    }
}

/// Uppercase, spaces and comments removed.
fn clean(s:&str)->String{
    let mut out=String::new();let mut paren=false;
    for c in s.chars(){
        match c{'('=>paren=true,')'=>paren=false,';'=>break,'\r'|' '|'\t'=>{},_ if paren=>{},_=>out.push(c.to_ascii_uppercase())}
    }
    out
}

impl crate::machine::Port for VirtualGrbl{
    fn write_all(&mut self,bytes:&[u8])->io::Result<()>{
        self.tick();
        for &b in bytes{
            if matches!(b,b'?'|b'!'|b'~'|b'@'|0x18){self.realtime(b);continue;}
            if self.rx.len()>=RX_SIZE{self.shared.overflows.fetch_add(1,Ordering::Relaxed);continue;}
            self.rx.push_back(b);
        }
        self.process_lines();
        Ok(())
    }
    fn available(&mut self)->io::Result<usize>{self.tick();Ok(self.out.len())}
    fn read(&mut self,buf:&mut [u8])->io::Result<usize>{
        let n=buf.len().min(self.out.len());
        for (i,b) in self.out.drain(..n).enumerate(){buf[i]=b;}
        Ok(n)
    }
}
// =============================================================================
// End of file: vgrbl.rs
// Location:    laser-cnc-controller/src/vgrbl.rs
// Created:     2026-10-13 17:54:31 UTC
// =============================================================================
