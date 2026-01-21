//! Altitude reference handling utilities.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AltitudeReference {
    Wgs84,
    Amsl,
}

impl AltitudeReference {
    pub fn parse(value: &str) -> Option<Self> {
        match value.trim().to_lowercase().as_str() {
            "wgs84" | "w84" | "hae" | "ellipsoid" => Some(Self::Wgs84),
            "amsl" | "msl" => Some(Self::Amsl),
            _ => None,
        }
    }
}

pub fn altitude_to_amsl(altitude_m: f64, reference: AltitudeReference, geoid_offset_m: f64) -> f64 {
    match reference {
        AltitudeReference::Wgs84 => altitude_m - geoid_offset_m,
        AltitudeReference::Amsl => altitude_m,
    }
}
