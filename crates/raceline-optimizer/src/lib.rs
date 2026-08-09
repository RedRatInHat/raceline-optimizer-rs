#![allow(
    clippy::explicit_auto_deref,
    clippy::needless_range_loop,
    clippy::too_many_arguments
)]

pub mod bike_dynamics_v1;
pub mod bike_mintime;
pub mod bike_mintime_v1;
pub mod car_mintime;
pub mod contracts;
pub mod csv;
pub mod dense_frenet;
pub mod ipopt;
pub mod json;
pub mod mintime;
pub mod mintime_common;
pub mod models;
pub mod point_mass;
pub mod section_frame;
pub mod solver_api;
pub mod station;
pub mod station_generation;
pub mod trajectory_quality;
pub mod vehicle_dynamics;

use std::fs;
use std::io;
use std::path::Path;

use json::{parse_json_str, JsonError, JsonValue};

pub type JsonObject = Vec<(String, JsonValue)>;

pub trait ToJsonValue {
    fn to_json_value(&self) -> JsonValue;
}

pub fn read_json_value(path: impl AsRef<Path>) -> Result<JsonValue, ContractIoError> {
    let path = path.as_ref();
    let body = fs::read_to_string(path)?;
    parse_json_str(&body).map_err(|source| ContractIoError::JsonRead {
        path: path.to_path_buf(),
        source,
    })
}

pub fn write_pretty_json_file<T: ToJsonValue>(
    path: impl AsRef<Path>,
    value: &T,
) -> Result<(), ContractIoError> {
    write_json_value(path, &value.to_json_value())
}

pub fn write_json_value(path: impl AsRef<Path>, value: &JsonValue) -> Result<(), ContractIoError> {
    let path = path.as_ref();
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut body = value.to_pretty_string();
    body.push('\n');
    fs::write(path, body)?;
    Ok(())
}

#[derive(Debug)]
pub enum ContractIoError {
    Io(io::Error),
    JsonRead {
        path: std::path::PathBuf,
        source: JsonError,
    },
    InvalidContract {
        path: std::path::PathBuf,
        message: String,
    },
}

impl std::fmt::Display for ContractIoError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "I/O error: {error}"),
            Self::JsonRead { path, source } => {
                write!(
                    formatter,
                    "failed to parse JSON {}: {source}",
                    path.display()
                )
            }
            Self::InvalidContract { path, message } => {
                write!(formatter, "invalid contract {}: {message}", path.display())
            }
        }
    }
}

impl std::error::Error for ContractIoError {}

impl From<io::Error> for ContractIoError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}
