use std::error::Error;
use std::fmt::{Display, Formatter};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MaptraxError {
    EmptyPolygon,
    InvalidPolygon(&'static str),
    InvalidMachineCount,
    MissingPart(usize),
    MissingField,
}

impl Display for MaptraxError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyPolygon => write!(f, "polygon has no vertices"),
            Self::InvalidPolygon(message) => write!(f, "invalid polygon: {message}"),
            Self::InvalidMachineCount => write!(f, "machine count must be greater than zero"),
            Self::MissingPart(index) => write!(f, "part index {index} is out of bounds"),
            Self::MissingField => write!(f, "field has not been set"),
        }
    }
}

impl Error for MaptraxError {}

pub type Result<T> = std::result::Result<T, MaptraxError>;
