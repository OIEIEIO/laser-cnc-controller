// =============================================================================
// File:     storage.rs
// Location: laser-cnc-controller/src/storage.rs
// Version:  v0.1.1
// Comments: Local persistent redb storage.
//           - projects: name -> project JSON
//           - runs:     run id -> run record JSON (indexed by project id in the record)
//           - meta:     last_project
//           v0.1.1: delete(), exists(), save_run(), runs().
// =============================================================================
use redb::{Database, ReadableTable, TableDefinition};
use crate::geometry::Project;
use crate::runlog::RunRecord;
const PROJECTS:TableDefinition<&str,&[u8]> = TableDefinition::new("projects");
const RUNS:TableDefinition<&str,&[u8]> = TableDefinition::new("runs");
const META:TableDefinition<&str,&str> = TableDefinition::new("meta");
pub struct Store { db:Database }
impl Store {
    pub fn open()->Result<Self,String> {
        let root=std::env::var_os("LASER_CNC_DATA").map(std::path::PathBuf::from).unwrap_or_else(|| {
            let base=std::env::var_os(if cfg!(windows) {"APPDATA"} else {"HOME"}).map(std::path::PathBuf::from).unwrap_or_else(||std::path::PathBuf::from("."));
            if cfg!(windows) {base.join("LaserCNC")} else {base.join(".local/share/laser-cnc-controller")}
        });
        std::fs::create_dir_all(&root).map_err(|e|e.to_string())?;
        let db=Database::create(root.join("laser-cnc.redb")).map_err(|e|e.to_string())?;
        Ok(Self{db})
    }
    pub fn save(&self,p:&Project)->Result<(),String>{
        let bytes=serde_json::to_vec(p).map_err(|e|e.to_string())?;
        let txn=self.db.begin_write().map_err(|e|e.to_string())?;
        {let mut t=txn.open_table(PROJECTS).map_err(|e|e.to_string())?;t.insert(p.name.as_str(),bytes.as_slice()).map_err(|e|e.to_string())?;}
        txn.commit().map_err(|e|e.to_string())
    }
    pub fn load(&self,name:&str)->Result<Option<Project>,String>{
        let txn=self.db.begin_read().map_err(|e|e.to_string())?;
        let t=txn.open_table(PROJECTS).map_err(|e|e.to_string())?;
        let value=t.get(name).map_err(|e|e.to_string())?;
        match value {Some(v)=>serde_json::from_slice(v.value()).map(Some).map_err(|e|e.to_string()),None=>Ok(None)}
    }
    pub fn names(&self)->Result<Vec<String>,String>{
        let txn=self.db.begin_read().map_err(|e|e.to_string())?;
        let t=txn.open_table(PROJECTS).map_err(|e|e.to_string())?;
        t.iter().map_err(|e|e.to_string())?.map(|item|item.map(|(key,_)|key.value().to_owned()).map_err(|e|e.to_string())).collect()
    }
    /// Remove a saved project. Its run records are kept.
    pub fn delete(&self,name:&str)->Result<(),String>{
        let txn=self.db.begin_write().map_err(|e|e.to_string())?;
        {let mut t=txn.open_table(PROJECTS).map_err(|e|e.to_string())?;t.remove(name).map_err(|e|e.to_string())?;}
        txn.commit().map_err(|e|e.to_string())
    }
    pub fn remember(&self,name:&str)->Result<(),String>{
        let txn=self.db.begin_write().map_err(|e|e.to_string())?;
        {let mut t=txn.open_table(META).map_err(|e|e.to_string())?;t.insert("last_project",name).map_err(|e|e.to_string())?;}
        txn.commit().map_err(|e|e.to_string())
    }
    /// Insert or replace a run record.
    pub fn save_run(&self,r:&RunRecord)->Result<(),String>{
        let bytes=serde_json::to_vec(r).map_err(|e|e.to_string())?;
        let txn=self.db.begin_write().map_err(|e|e.to_string())?;
        {let mut t=txn.open_table(RUNS).map_err(|e|e.to_string())?;t.insert(r.id.as_str(),bytes.as_slice()).map_err(|e|e.to_string())?;}
        txn.commit().map_err(|e|e.to_string())
    }
    /// All run records, newest first.
    pub fn runs(&self)->Result<Vec<RunRecord>,String>{
        let txn=self.db.begin_read().map_err(|e|e.to_string())?;
        let t=match txn.open_table(RUNS){Ok(t)=>t,Err(redb::TableError::TableDoesNotExist(_))=>return Ok(vec![]),Err(e)=>return Err(e.to_string())};
        let mut out:Vec<RunRecord>=Vec::new();
        for item in t.iter().map_err(|e|e.to_string())?{
            let (_,v)=item.map_err(|e|e.to_string())?;
            if let Ok(r)=serde_json::from_slice::<RunRecord>(v.value()){out.push(r);}
        }
        out.sort_by(|a,b|b.started.cmp(&a.started));
        Ok(out)
    }
}
// =============================================================================
// End of file: storage.rs
// Location:    laser-cnc-controller/src/storage.rs
// Created:     2026-10-12 22:40:08 UTC
// =============================================================================
