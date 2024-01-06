#![no_std]

use alloc::vec::Vec;
use serde::{Deserialize, Serialize};

extern crate alloc;

#[derive(Deserialize,Serialize,Debug,Clone, Copy)]
pub enum ControlMessage {
    SteeringPosition(i32),
}

impl ControlMessage {
    pub fn to_bytes(&self)->Vec<u8> {
        postcard::to_allocvec(self).unwrap()
    }

    pub fn from_slice(data: &[u8])->Self {
        postcard::from_bytes(data).unwrap()
    }
}