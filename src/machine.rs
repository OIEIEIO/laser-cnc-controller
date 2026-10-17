// =============================================================================
// File:     machine.rs
// Location: laser-cnc-controller/src/machine.rs
// Version:  v0.1.6
// Comments: GRBL 0.9i serial transport.
//           v0.1.1:
//           - Machine state word parsed from status reports (<Run,...>,
//             <Hold,...>, <Door,...>, <Idle,...>, <Alarm,...>) into `state`.
//           - pause() sends '@' (safety door: decelerate, hold position,
//             spindle/laser off). resume() sends '~' (GRBL restores the
//             spindle state, then continues motion).
//           - Log keeps manual TX and non-routine RX (errors, alarms, banner,
//             messages). Status reports, per-line "ok" and streamed TX lines
//             are not logged; the stream view shows them by line state.
//           v0.1.2:
//           - journal: every command sent and every non-routine controller
//             reply, for the daily log and per-run serial.log.
//           v0.1.3 (matches LaserGRBL default "Buffered" streaming):
//           - Serial I/O runs on its own thread ("grbl-serial"); streaming no
//             longer depends on the UI repainting.
//           - Character-counting flow control: a line is sent when the bytes
//             in flight plus the line fit in the 127-byte GRBL RX buffer
//             model; each ok/error releases the oldest in-flight line.
//           - Lines are trimmed, spaces stripped (not for '$' commands), '\n'.
//           - Real-time bytes (? ! ~ @ Ctrl-X) bypass the count.
//           - Status query '?' every 250 ms.
//           - Line errors (error:...) are counted and streaming continues.
//             ALARM, a GRBL banner (reset) or Abort flush the stream.
//           - Connect: DTR held low, soft reset (Ctrl-X), wait for the
//             "Grbl " banner (one retry at 2.5 s, fail at 6 s). Then read-only
//             queries $I, $G, $#, $$ and '?' fill `info`. No settings written.
//           - Manual commands and jogs go through the same counted queue.
//           - unlock() sends $X.
//           - Write timeout 1000 ms; reads only when bytes are available.
//           v0.1.4: reset_count increments on every GRBL banner (connect,
//           Reset, alarm reset) so the app can drop session state that a
//           controller reset invalidates (work zero, spot toggle).
//           v0.1.5: Simulate mode.
//           - The worker talks to a Port: the serial port (HwPort) or the
//             virtual GRBL (src/vgrbl.rs). Everything above the Port is the
//             same code path.
//           - simulate (set while disconnected): Connect attaches the virtual
//             controller ("SIMULATOR"). sim_speed scales its time (1–50x),
//             changeable during a run. sim_laser() reports its laser state.
//           v0.1.6: sim_laser() removed (the app takes the laser state from
//           its motion-planner trajectory in both live and Simulate runs).
// =============================================================================
use std::collections::VecDeque;
use std::io::{Read,Write};
use std::sync::mpsc::{self,Receiver,Sender,TryRecvError};
use std::time::{Duration,Instant};
use crate::vgrbl::{SimShared,VirtualGrbl};

/// Byte transport under the serial worker: hardware serial port or the virtual controller.
pub trait Port:Send{
    fn write_all(&mut self,bytes:&[u8])->std::io::Result<()>;
    fn available(&mut self)->std::io::Result<usize>;
    fn read(&mut self,buf:&mut [u8])->std::io::Result<usize>;
    fn dtr_low(&mut self){}
    fn clear_input(&mut self){}
}
struct HwPort(Box<dyn serialport::SerialPort>);
impl Port for HwPort{
    fn write_all(&mut self,bytes:&[u8])->std::io::Result<()>{self.0.write_all(bytes)}
    fn available(&mut self)->std::io::Result<usize>{self.0.bytes_to_read().map(|n|n as usize).map_err(std::io::Error::from)}
    fn read(&mut self,buf:&mut [u8])->std::io::Result<usize>{self.0.read(buf)}
    fn dtr_low(&mut self){let _=self.0.write_data_terminal_ready(false);}
    fn clear_input(&mut self){let _=self.0.clear(serialport::ClearBuffer::Input);}
}

/// GRBL serial RX buffer model (LaserGRBL DEFAULT_BUFFER_SIZE).
pub const RX_BUFFER:usize=127;
const STATUS_PERIOD:Duration=Duration::from_millis(250);
const BANNER_RETRY:Duration=Duration::from_millis(2500);
const BANNER_TIMEOUT:Duration=Duration::from_millis(6000);
const WRITE_TIMEOUT:Duration=Duration::from_millis(1000);
const OFFSET_KEYS:[&str;11]=["G54","G55","G56","G57","G58","G59","G28","G30","G92","TLO","PRB"];

#[derive(Clone,Copy,PartialEq,Debug)]
pub enum Link{Disconnected,Connecting,Connected}

/// Read-only controller information gathered on connect.
#[derive(Clone,Default)]
pub struct ControllerInfo{
    pub banner:Option<String>,          // "Grbl 0.9i ['$' for help]"
    pub build:Option<String>,           // $I  "0.9i.20150620:"
    pub modes:Option<String>,           // $G  "G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F0. S0."
    pub offsets:Vec<String>,            // $#  "G54:0.000,0.000,0.000"
    pub settings:Vec<String>,           // $$  "$100=80.000 (x, step/mm)"
    pub mpos:Option<(f32,f32,f32)>,
    pub wpos:Option<(f32,f32,f32)>,
}
impl ControllerInfo{
    /// Spindle (laser) word and S value from the $G parser state.
    pub fn spindle(&self)->Option<(String,String)>{
        let m=self.modes.as_ref()?;
        let words:Vec<&str>=m.split_whitespace().collect();
        let mc=words.iter().find(|w|matches!(**w,"M3"|"M4"|"M5"))?;
        let s=words.iter().find(|w|w.starts_with('S')).map(|s|s.to_string()).unwrap_or_default();
        Some((mc.to_string(),s))
    }
}

// ---------------------------------------------------------------- UI <-> worker messages
enum Cmd{Job(Vec<String>),Line(String),Realtime(u8),Abort,Close}
enum Event{
    Connected(String),
    Failed(String),
    PortError(String),
    Status(String),
    Rx(String),
    Sent{job:Option<usize>,text:String},
    Reply{job:Option<usize>,text:String,reply:String},
    JobDone,
    JobStopped,
}

pub struct Machine {
    worker:Option<(Sender<Cmd>,Receiver<Event>)>,
    pub link:Link,
    pub ports:Vec<String>, pub selected_port:String, pub baud:u32,
    pub log:Vec<String>, pub status:String, pub state:String, pub position:(f32,f32),
    pub next_line:usize, pub running:bool, pub acknowledged:usize,
    pub error:Option<String>, pub error_line:Option<usize>,
    pub job_errors:usize, pub job_error:Option<String>, pub alarm:Option<String>,
    pub journal:Vec<String>, pub firmware:Option<String>, pub info:ControllerInfo,
    pub reset_count:u32,
    pub simulate:bool, pub sim_speed:f32, sim:Option<SimShared>,
}
impl Default for Machine {fn default()->Self{Self{worker:None,link:Link::Disconnected,ports:vec![],selected_port:String::new(),baud:115200,log:vec![],status:"Disconnected".into(),state:String::new(),position:(0.0,0.0),next_line:0,running:false,acknowledged:0,error:None,error_line:None,job_errors:0,job_error:None,alarm:None,journal:vec![],firmware:None,info:ControllerInfo::default(),reset_count:0,simulate:false,sim_speed:1.0,sim:None}}}
impl Machine {
    pub fn refresh(&mut self){if let Ok(ports)=serialport::available_ports(){self.ports=ports.into_iter().map(|p|p.port_name).collect();if self.selected_port.is_empty(){if let Some(s)=self.ports.first(){self.selected_port=s.clone();}}}}
    pub fn connected(&self)->bool{self.link==Link::Connected}
    pub fn set_sim_speed(&mut self,speed:f32){self.sim_speed=speed;if let Some(s)=&self.sim{s.set_speed(speed);}}
    /// Label of the port in use (hardware port name or SIMULATOR).
    pub fn port_label(&self)->String{if self.simulate{"SIMULATOR".into()}else{self.selected_port.clone()}}
    /// Open the port (or the virtual controller) and start the serial worker. Connected once the GRBL banner arrives.
    pub fn connect(&mut self){
        if self.worker.is_some(){return;}
        let opened:Result<Box<dyn Port>,String>=if self.simulate{
            let shared=SimShared::new(self.sim_speed);self.sim=Some(shared.clone());
            Ok(Box::new(VirtualGrbl::new(shared)))
        }else{
            self.sim=None;
            serialport::new(&self.selected_port,self.baud).timeout(WRITE_TIMEOUT).open().map(|p|Box::new(HwPort(p)) as Box<dyn Port>).map_err(|e|e.to_string())
        };
        match opened{
            Ok(port)=>{
                let (cmd_tx,cmd_rx)=mpsc::channel();
                let (ev_tx,ev_rx)=mpsc::channel();
                let worker=Worker{port,rx:cmd_rx,tx:ev_tx,queue:VecDeque::new(),pending:VecDeque::new(),used:0,job_active:false,
                    incoming:Vec::new(),connected:false,opened:Instant::now(),retried:false,last_status:None};
                if let Err(e)=std::thread::Builder::new().name("grbl-serial".into()).spawn(move||worker.run()){
                    self.error=Some(format!("Serial thread failed: {e}"));self.status="Connect failed".into();return;
                }
                self.worker=Some((cmd_tx,ev_rx));
                self.link=Link::Connecting;
                self.status="Connecting… waiting for GRBL banner".into();
                self.state.clear();self.error=None;self.firmware=None;self.info=ControllerInfo::default();
                self.running=false;self.next_line=0;self.acknowledged=0;self.error_line=None;
                self.log.push(format!("Connecting {} @ {}",self.port_label(),self.baud));
                self.journal.push(format!("CONNECT {} @ {}",self.port_label(),self.baud));
            }
            Err(e)=>{self.error=Some(e.clone());self.status=format!("Connect failed: {e}");}
        }
    }
    pub fn disconnect(&mut self){
        if self.connected(){self.abort();}
        self.send(Cmd::Close);
        self.drop_link();
        self.status="Disconnected".into();
        self.journal.push("DISCONNECT".into());
    }
    fn drop_link(&mut self){self.worker=None;self.sim=None;self.link=Link::Disconnected;self.running=false;self.state.clear();}
    fn send(&mut self,c:Cmd){if let Some((tx,_))=&self.worker{let _=tx.send(c);}}
    pub fn realtime(&mut self,b:u8){if self.worker.is_some(){self.send(Cmd::Realtime(b));}}
    /// Queue a manual line (G-code or $ command). Refused while a job streams.
    pub fn command(&mut self,s:&str){
        if self.running {self.error=Some("Controller busy: stop job before sending manual commands".into());return;}
        if !self.connected(){self.error=Some("Not connected".into());return;}
        self.send(Cmd::Line(s.to_string()));
    }
    /// Incremental move, then back to absolute. GRBL 0.9 has no $J.
    pub fn jog(&mut self,dx:f32,dy:f32){
        if self.running||!self.connected(){return;}
        self.send(Cmd::Line(format!("G91 G0 X{dx:.3} Y{dy:.3}")));
        self.send(Cmd::Line("G90".into()));
    }
    /// Clear the GRBL alarm lock.
    pub fn unlock(&mut self){self.command("$X");}
    /// Pause with laser off: GRBL 0.9 safety-door command '@'.
    pub fn pause(&mut self){if self.connected(){self.realtime(b'@');self.log.push("TX @ (pause, laser off)".into());self.journal.push("TX @ (pause, laser off)".into());}}
    /// Resume from Door/Hold: cycle start '~'. GRBL restores the spindle state before motion continues.
    pub fn resume(&mut self){if self.connected(){self.realtime(b'~');self.log.push("TX ~ (resume)".into());self.journal.push("TX ~ (resume)".into());}}
    pub fn abort(&mut self){
        if self.worker.is_some(){self.send(Cmd::Abort);self.log.push("TX ! + Ctrl-X (reset)".into());self.journal.push("TX ! + Ctrl-X (reset)".into());}
        self.running=false;self.next_line=0;self.status="Stopped / controller reset".into();
    }
    /// Stream a program.
    pub fn start(&mut self,lines:&[String]){
        if !self.connected(){self.error=Some("Connect before starting".into());return;}
        if self.running{self.error=Some("A job is already streaming".into());return;}
        self.running=true;self.next_line=0;self.acknowledged=0;self.error=None;self.error_line=None;
        self.job_errors=0;self.job_error=None;self.alarm=None;
        self.send(Cmd::Job(lines.to_vec()));
    }
    /// Apply worker events. Called once per UI frame; streaming itself runs on the worker thread.
    pub fn poll(&mut self){
        let mut events=Vec::new();let mut lost=false;
        if let Some((_,rx))=&self.worker{
            loop{match rx.try_recv(){Ok(e)=>events.push(e),Err(TryRecvError::Empty)=>break,Err(TryRecvError::Disconnected)=>{lost=true;break;}}}
        }
        for e in events{self.handle(e);}
        if lost&&self.worker.is_some(){self.drop_link();self.status="Serial worker stopped".into();}
        if self.log.len()>300 {let excess=self.log.len()-300;self.log.drain(..excess);}
    }
    fn handle(&mut self,e:Event){
        match e{
            Event::Connected(banner)=>{
                self.link=Link::Connected;self.status="Connected".into();self.reset_count+=1;
                self.firmware=Some(banner.clone());self.info.banner=Some(banner.clone());
                self.log.push(format!("RX {banner}"));self.journal.push(format!("RX {banner}"));
            }
            Event::Failed(reason)=>{
                self.drop_link();self.status=format!("Connect failed: {reason}");self.error=Some(reason.clone());
                self.log.push(format!("Connect failed: {reason}"));self.journal.push(format!("CONNECT FAILED {reason}"));
            }
            Event::PortError(err)=>{
                self.drop_link();self.status=format!("Port error: {err}");self.error=Some(err.clone());
                self.journal.push(format!("PORT ERROR {err}"));
            }
            Event::Status(s)=>self.parse_status(&s),
            Event::Rx(s)=>{
                self.log.push(format!("RX {s}"));self.journal.push(format!("RX {s}"));
                if s.starts_with("Grbl "){self.firmware=Some(s.clone());self.info.banner=Some(s.clone());self.running=false;self.reset_count+=1;}
                else if s.starts_with("ALARM"){
                    if self.running&&self.alarm.is_none(){self.alarm=Some(s.clone());}
                    self.error=Some(s.clone());self.running=false;
                }
                else {self.classify(&s);}
            }
            Event::Sent{job,text}=>{
                self.journal.push(format!("TX {text}"));
                match job{Some(i)=>self.next_line=self.next_line.max(i+1),None=>self.log.push(format!("TX {text}"))}
            }
            Event::Reply{job,text,reply}=>{
                if let Some(i)=job{self.acknowledged=self.acknowledged.max(i+1);}
                if reply.starts_with("error"){
                    self.log.push(format!("RX {reply}  ← {text}"));self.journal.push(format!("RX {reply}  <- {text}"));
                    match job{
                        Some(i)=>{self.job_errors+=1;if self.error_line.is_none(){self.error_line=Some(i);self.job_error=Some(reply.clone());}
                                  self.error=Some(format!("{reply} (line {}) — streaming continues",i+1));}
                        None=>self.error=Some(format!("{reply} ← {text}")),
                    }
                }
            }
            Event::JobDone=>{self.running=false;self.status="G-code accepted; waiting for physical idle".into();}
            Event::JobStopped=>{self.running=false;}
        }
    }
    fn parse_status(&mut self,s:&str){
        // State word: GRBL 0.9 "<Idle,MPos:...>"; GRBL 1.1 "<Idle|MPos:...>".
        self.state=s[1..].split(|c|c==','||c=='|'||c=='>').next().unwrap_or("").to_string();
        self.info.mpos=triple(s,"MPos:");
        self.info.wpos=triple(s,"WPos:");
        if let Some(p)=self.info.wpos.or(self.info.mpos){self.position=(p.0,p.1);}
    }
    /// Sort $I / $G / $# / $$ replies into `info`.
    fn classify(&mut self,s:&str){
        if let Some(inner)=s.strip_prefix('[').and_then(|t|t.strip_suffix(']')){
            let first=inner.chars().next().unwrap_or(' ');
            if first.is_ascii_digit(){self.info.build=Some(inner.to_string());}
            else if let Some((key,_))=inner.split_once(':'){
                if OFFSET_KEYS.contains(&key){upsert(&mut self.info.offsets,key,':',inner);}
            }
            else if first=='G'{self.info.modes=Some(inner.to_string());}
        } else if s.starts_with('$'){
            if let Some((key,_))=s.split_once('='){upsert(&mut self.info.settings,key,'=',s);}
        }
    }
}

/// Replace the entry with the same key (text before `sep`) or append.
fn upsert(list:&mut Vec<String>,key:&str,sep:char,value:&str){
    match list.iter_mut().find(|e|e.split(sep).next()==Some(key)){Some(e)=>*e=value.to_string(),None=>list.push(value.to_string())}
}
/// Three numbers following `key` in a status report.
fn triple(s:&str,key:&str)->Option<(f32,f32,f32)>{
    let rest=s.split(key).nth(1)?;
    let mut v=rest.split(|c|c==','||c=='|'||c=='>');
    let a=v.next()?.parse().ok()?;let b=v.next()?.parse().ok()?;let c=v.next()?.parse().ok()?;
    Some((a,b,c))
}

// ---------------------------------------------------------------- serial worker thread
struct Item{data:String,job:Option<usize>,text:String}
impl Item{
    /// Trim; strip spaces except for '$' commands (as LaserGRBL); terminate with '\n'.
    fn new(line:&str,job:Option<usize>)->Self{
        let t=line.trim_matches(|c|c=='\r'||c=='\n'||c==' ');
        let data=if t.starts_with('$'){format!("{t}\n")}else{format!("{}\n",t.replace(' ',""))};
        Self{data,job,text:t.to_string()}
    }
}
struct Worker{
    port:Box<dyn Port>, rx:Receiver<Cmd>, tx:Sender<Event>,
    queue:VecDeque<Item>, pending:VecDeque<Item>, used:usize, job_active:bool,
    incoming:Vec<u8>, connected:bool, opened:Instant, retried:bool, last_status:Option<Instant>,
}
impl Worker{
    fn emit(&self,e:Event){let _=self.tx.send(e);}
    fn run(mut self){
        // Handshake: DTR low (no hardware reset from us), discard stale input, soft reset.
        self.port.dtr_low();
        self.port.clear_input();
        if !self.write(&[0x18]){return;}
        let mut buf=[0u8;1024];
        loop{
            // ---- commands from the UI ----
            loop{
                match self.rx.try_recv(){
                    Ok(c)=>if !self.command(c){return;},
                    Err(TryRecvError::Empty)=>break,
                    Err(TryRecvError::Disconnected)=>return,
                }
            }
            // ---- banner wait / status query ----
            if !self.connected{
                let t=self.opened.elapsed();
                if t>BANNER_TIMEOUT{self.emit(Event::Failed("no GRBL banner within 6 s".into()));return;}
                if t>BANNER_RETRY&&!self.retried{self.retried=true;if !self.write(&[0x18]){return;}}
            } else if self.last_status.map_or(true,|t|t.elapsed()>=STATUS_PERIOD){
                if !self.write(b"?"){return;}
                self.last_status=Some(Instant::now());
            }
            // ---- receive ----
            let available=match self.port.available(){Ok(n)=>n,Err(e)=>{self.emit(Event::PortError(e.to_string()));return;}};
            let mut busy=false;
            if available>0{
                let n=available.min(buf.len());
                match self.port.read(&mut buf[..n]){
                    Ok(k)=>{busy=k>0;for &b in &buf[..k]{self.byte(b);}}
                    Err(ref e) if e.kind()==std::io::ErrorKind::TimedOut=>{}
                    Err(e)=>{self.emit(Event::PortError(e.to_string()));return;}
                }
            }
            // ---- transmit (character counting) ----
            if self.connected&&!self.pump(){return;}
            if !busy{std::thread::sleep(Duration::from_millis(1));}
        }
    }
    fn write(&mut self,bytes:&[u8])->bool{
        match self.port.write_all(bytes){Ok(())=>true,Err(e)=>{self.emit(Event::PortError(e.to_string()));false}}
    }
    /// Returns false when the worker must stop.
    fn command(&mut self,c:Cmd)->bool{
        match c{
            Cmd::Job(lines)=>{
                for (i,l) in lines.iter().enumerate(){self.queue.push_back(Item::new(l,Some(i)));}
                self.job_active=true;self.check_done();true
            }
            Cmd::Line(s)=>{self.queue.push_back(Item::new(&s,None));true}
            Cmd::Realtime(b)=>self.write(&[b]),
            Cmd::Abort=>{let ok=self.write(b"!")&&self.write(&[0x18]);self.flush();ok}
            Cmd::Close=>false,
        }
    }
    /// Send queued lines while they fit in the RX buffer model.
    fn pump(&mut self)->bool{
        while let Some(front)=self.queue.front(){
            let len=front.data.len();
            // An oversize line is sent only into an empty buffer.
            if self.used+len>RX_BUFFER&&!self.pending.is_empty(){break;}
            let item=self.queue.pop_front().unwrap();
            if !self.write(item.data.as_bytes()){return false;}
            self.used+=len;
            self.emit(Event::Sent{job:item.job,text:item.text.clone()});
            self.pending.push_back(item);
        }
        true
    }
    fn flush(&mut self){self.queue.clear();self.pending.clear();self.used=0;self.job_active=false;}
    fn check_done(&mut self){
        if self.job_active&&self.pending.is_empty()&&!self.queue.iter().any(|i|i.job.is_some()){self.job_active=false;self.emit(Event::JobDone);}
    }
    fn byte(&mut self,b:u8){
        if b==b'\n'||b==b'\r'{
            if !self.incoming.is_empty(){
                let line=String::from_utf8_lossy(&self.incoming).trim().to_string();
                self.incoming.clear();
                if !line.is_empty(){self.line(line);}
            }
        } else if self.incoming.len()<2048{self.incoming.push(b);}
    }
    fn line(&mut self,s:String){
        if s.starts_with('<'){self.emit(Event::Status(s));return;}
        if s=="ok"||s.starts_with("error"){
            let item=self.pending.pop_front();
            if let Some(i)=&item{self.used=self.used.saturating_sub(i.data.len());}
            let (job,text)=item.map(|i|(i.job,i.text)).unwrap_or((None,String::new()));
            self.emit(Event::Reply{job,text,reply:s});
            self.check_done();
            return;
        }
        if s.starts_with("Grbl "){
            // Banner: controller has (re)started; anything in flight is gone.
            let was=self.job_active;self.flush();
            if !self.connected{
                self.connected=true;self.last_status=None;
                self.emit(Event::Connected(s));
                for q in ["$I","$G","$#","$$"]{self.queue.push_back(Item::new(q,None));}
            } else {
                self.emit(Event::Rx(s));
                if was{self.emit(Event::JobStopped);}
            }
            return;
        }
        if s.starts_with("ALARM"){
            let was=self.job_active;self.flush();
            self.emit(Event::Rx(s));
            if was{self.emit(Event::JobStopped);}
            return;
        }
        self.emit(Event::Rx(s));
    }
}
// =============================================================================
// End of file: machine.rs
// Location:    laser-cnc-controller/src/machine.rs
// Created:     2026-10-16 21:27:22 UTC
// =============================================================================
