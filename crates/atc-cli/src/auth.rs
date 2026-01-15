//! JWT authentication module for Flight Blender API.
//!
//! With BYPASS_AUTH_TOKEN_VERIFICATION=1 set in Blender's .env,
//! the token signature is not verified, but the token structure,
//! issuer, audience, and scopes are still checked.

use jsonwebtoken::{encode, EncodingKey, Header};
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};

/// JWT claims for Flight Blender authentication.
#[derive(Debug, Serialize, Deserialize)]
struct Claims {
    /// Issuer
    iss: String,
    /// Subject
    sub: String,
    /// Audience
    aud: String,
    /// Scope (space-separated)
    scope: String,
    /// Expiration time (Unix timestamp)
    exp: u64,
    /// Issued at (Unix timestamp)
    iat: u64,
}

/// Configuration for token generation.
#[derive(Debug, Clone)]
pub struct TokenConfig {
    /// Space-separated list of scopes
    pub scopes: String,
    /// Token audience (must match PASSPORT_AUDIENCE in Blender)
    pub audience: String,
    /// Token validity in hours
    pub expiry_hours: u64,
}

impl Default for TokenConfig {
    fn default() -> Self {
        Self {
            scopes: "flightblender.read flightblender.write".to_string(),
            audience: "testflight.flightblender.com".to_string(),
            expiry_hours: 24,
        }
    }
}

/// Generate a dummy JWT token for Flight Blender API access.
///
/// # Arguments
/// * `config` - Optional token configuration (uses defaults if None)
///
/// # Returns
/// A JWT token string
///
/// # Example
/// ```
/// use atc_cli::auth::generate_dummy_token;
///
/// let token = generate_dummy_token(None);
/// println!("Token: {}", token);
/// ```
pub fn generate_dummy_token(config: Option<TokenConfig>) -> String {
    let config = config.unwrap_or_default();

    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards")
        .as_secs();

    let claims = Claims {
        iss: "dummy".to_string(),
        sub: "atc-drone-system".to_string(),
        aud: config.audience,
        scope: config.scopes,
        exp: now + (config.expiry_hours * 3600),
        iat: now,
    };

    // With BYPASS_AUTH_TOKEN_VERIFICATION=1, signature isn't verified
    // but we still need a valid JWT structure
    encode(
        &Header::default(),
        &claims,
        &EncodingKey::from_secret(b"dummy-secret"),
    )
    .expect("Failed to encode JWT token")
}

#[cfg(test)]
mod tests {
    use super::*;
    use jsonwebtoken::{decode, DecodingKey, Validation};

    #[test]
    fn test_generate_token_is_valid_jwt() {
        let token = generate_dummy_token(None);

        // Should have 3 parts separated by dots
        let parts: Vec<_> = token.split('.').collect();
        assert_eq!(parts.len(), 3);
    }

    #[test]
    fn test_token_has_correct_claims() {
        let token = generate_dummy_token(None);

        let mut validation = Validation::default();
        validation.validate_exp = false;
        validation.set_audience(&["testflight.flightblender.com"]);
        validation.insecure_disable_signature_validation();

        let decoded = decode::<Claims>(
            &token,
            &DecodingKey::from_secret(b"dummy-secret"),
            &validation,
        )
        .expect("Failed to decode token");

        assert_eq!(decoded.claims.iss, "dummy");
        assert_eq!(decoded.claims.sub, "atc-drone-system");
        assert!(decoded.claims.scope.contains("flightblender.read"));
    }
}
