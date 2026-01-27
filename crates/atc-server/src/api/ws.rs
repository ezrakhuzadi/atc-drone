//! WebSocket streaming for real-time updates.
use crate::state::AppState;
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        Query, State,
    },
    http::{header::AUTHORIZATION, HeaderMap, StatusCode},
    response::IntoResponse,
};
use serde::Deserialize;
use std::sync::Arc;

/// Handler for WebSocket connections.
pub async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Query(params): Query<WsQuery>,
) -> axum::response::Response {
    let config = state.config();
    let provided = params.token.clone().or_else(|| extract_bearer(&headers));

    if config.require_ws_token {
        let expected = config.ws_token.as_deref().unwrap_or_default();
        if provided.as_deref() != Some(expected) {
            return StatusCode::UNAUTHORIZED.into_response();
        }
    } else if let Some(expected) = config.ws_token.as_deref() {
        if let Some(token) = provided.as_deref() {
            if token != expected {
                return StatusCode::UNAUTHORIZED.into_response();
            }
        }
    }

    let owner_filter = params.owner_id.clone();
    let drone_filter = params.drone_id.clone();
    ws.on_upgrade(move |socket| handle_socket(socket, state, owner_filter, drone_filter))
        .into_response()
}

#[derive(Debug, Deserialize, Default)]
pub struct WsQuery {
    token: Option<String>,
    owner_id: Option<String>,
    drone_id: Option<String>,
}

fn extract_bearer(headers: &HeaderMap) -> Option<String> {
    let value = headers.get(AUTHORIZATION)?.to_str().ok()?;
    let token = value.strip_prefix("Bearer ")?;
    let trimmed = token.trim();
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed.to_string())
    }
}

async fn handle_socket(
    mut socket: WebSocket,
    state: Arc<AppState>,
    owner_filter: Option<String>,
    drone_filter: Option<String>,
) {
    let mut rx = state.tx.subscribe();

    loop {
        tokio::select! {
            incoming = socket.recv() => {
                match incoming {
                    Some(Ok(Message::Ping(payload))) => {
                        if socket.send(Message::Pong(payload)).await.is_err() {
                            break;
                        }
                    }
                    Some(Ok(Message::Close(_))) => break,
                    Some(Ok(_)) => {}
                    Some(Err(_)) | None => break,
                }
            }
            event = rx.recv() => {
                match event {
                    Ok(msg) => {
                        if let Some(owner_id) = owner_filter.as_deref() {
                            if msg.owner_id.as_deref() != Some(owner_id) {
                                continue;
                            }
                        }
                        if let Some(drone_id) = drone_filter.as_deref() {
                            if msg.drone_id != drone_id {
                                continue;
                            }
                        }
                        if socket.send(Message::Text(msg.payload.as_ref().to_owned())).await.is_err() {
                            break;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                        // Drop missed updates; a newer snapshot will arrive soon.
                        continue;
                    }
                    Err(_) => break,
                }
            }
        }
    }
}
