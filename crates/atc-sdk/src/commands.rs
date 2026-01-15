//! Command receive and acknowledge helpers.

use atc_core::models::Command;
use serde::{Deserialize, Serialize};

/// Trait for handling commands from ATC.
pub trait CommandHandler {
    /// Called when a command is received.
    fn on_command(&mut self, command: &Command);
}

/// Acknowledgement for a received command.
#[derive(Debug, Serialize, Deserialize)]
pub struct CommandAck {
    pub command_id: String,
    pub drone_id: String,
    pub accepted: bool,
    pub reason: Option<String>,
}
