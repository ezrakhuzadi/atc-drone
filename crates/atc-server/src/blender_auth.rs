//! Blender authentication helper (static token or OAuth client credentials).

use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use reqwest::Client;
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;

use crate::config::{BlenderOAuthConfig, Config};
use atc_blender::BlenderClient;

const TOKEN_REFRESH_LEEWAY_SECS: u64 = 60;

#[derive(Debug, Clone)]
struct CachedToken {
    access_token: String,
    expires_at: Instant,
}

#[derive(Debug, Serialize)]
struct OAuthTokenRequest<'a> {
    grant_type: &'a str,
    client_id: &'a str,
    client_secret: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    scope: Option<&'a str>,
}

#[derive(Debug, Deserialize)]
struct OAuthTokenResponse {
    access_token: String,
    #[serde(default)]
    expires_in: Option<u64>,
}

pub struct BlenderAuthManager {
    oauth: Option<BlenderOAuthConfig>,
    static_token: String,
    allow_dummy: bool,
    client: Client,
    cached: RwLock<Option<CachedToken>>,
}

impl BlenderAuthManager {
    pub fn new(config: &Config) -> Self {
        Self {
            oauth: config.blender_oauth_config(),
            static_token: config.blender_auth_token.clone(),
            allow_dummy: config.allow_dummy_blender_auth,
            client: Client::new(),
            cached: RwLock::new(None),
        }
    }

    pub async fn apply(&self, blender: &mut BlenderClient) -> Result<()> {
        let token = self.resolve_token().await?;
        if token.is_none() && !self.allow_dummy {
            anyhow::bail!("Blender auth token missing and dummy auth disabled");
        }
        blender.set_auth_token(token);
        Ok(())
    }

    async fn resolve_token(&self) -> Result<Option<String>> {
        if let Some(oauth) = &self.oauth {
            if let Some(token) = self.cached_token().await {
                return Ok(Some(token));
            }
            let fresh = self.fetch_oauth_token(oauth).await?;
            let token = fresh.access_token.clone();
            let mut guard = self.cached.write().await;
            *guard = Some(fresh);
            return Ok(Some(token));
        }

        let token = self.static_token.trim();
        if token.is_empty() {
            return Ok(None);
        }
        Ok(Some(token.to_string()))
    }

    async fn cached_token(&self) -> Option<String> {
        let guard = self.cached.read().await;
        guard.as_ref().and_then(|cached| {
            if cached.expires_at > Instant::now() {
                Some(cached.access_token.clone())
            } else {
                None
            }
        })
    }

    async fn fetch_oauth_token(&self, oauth: &BlenderOAuthConfig) -> Result<CachedToken> {
        let request = OAuthTokenRequest {
            grant_type: "client_credentials",
            client_id: oauth.client_id.as_str(),
            client_secret: oauth.client_secret.as_str(),
            scope: oauth.scope.as_deref(),
        };

        let response = self
            .client
            .post(oauth.token_url.as_str())
            .form(&request)
            .send()
            .await
            .context("OAuth token request failed")?
            .error_for_status()
            .context("OAuth token request returned error status")?;

        let payload: OAuthTokenResponse = response
            .json()
            .await
            .context("OAuth token response parse failed")?;

        let ttl = payload
            .expires_in
            .unwrap_or(3600)
            .saturating_sub(TOKEN_REFRESH_LEEWAY_SECS)
            .max(60);

        Ok(CachedToken {
            access_token: payload.access_token,
            expires_at: Instant::now() + Duration::from_secs(ttl),
        })
    }
}
