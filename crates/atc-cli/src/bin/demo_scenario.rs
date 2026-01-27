//! Golden Demo Scenario - Realistic drone lifecycle with collision avoidance.
//!
//! This script demonstrates the complete drone lifecycle:
//!
//! 1. PREFLIGHT: Drones start on ground (0m altitude)
//! 2. TAKEOFF: Ascend to flight altitude over 5 seconds
//! 3. CRUISE: Fly toward destination, handle collision avoidance
//! 4. LANDING: Descend to ground at destination
//! 5. LANDED: Stay on ground, keep sending telemetry
//!
//! Usage:
//!   cargo run -p atc-cli --bin demo_scenario -- --url http://localhost:3000 --owner guest
//!
//! If running against the unified Docker stack, default tokens match `docker-compose.unified.yml`:
//!   --registration-token change-me --admin-token change-me-admin

use atc_core::models::{Command, CommandType};
use atc_core::spatial::{bearing, haversine_distance, offset_by_bearing};
use atc_sdk::AtcClient;
use clap::Parser;
use serde_json::json;
use std::env;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::sync::mpsc::error::TryRecvError;
use tokio::time;

/// Irvine coordinates (flight hub)
const IRVINE_LAT: f64 = 33.6846;
const IRVINE_LON: f64 = -117.8265;

/// Demo parameters
const CRUISE_ALTITUDE_M: f64 = 50.0;
const DRONE_SPEED_MPS: f64 = 10.0;
const UPDATE_RATE_HZ: f64 = 1.0;
const BATTERY_CAPACITY_MIN: f64 = 12.0;
const BATTERY_RESERVE_MIN: f64 = 2.0;
const BATTERY_HOVER_DRAIN_FACTOR: f64 = 0.3;
const MAX_TURN_RATE_DEG_PER_S: f64 = 25.0;
const MAX_ACCEL_MPS2: f64 = 2.0;
const WAYPOINT_REACHED_M: f64 = 5.0;
const COMMAND_CHANNEL_SIZE: usize = 32;

/// ~300m offset from the hub
const OFFSET_M: f64 = 300.0;

/// Lifecycle timing (seconds)
const PREFLIGHT_DURATION: f64 = 3.0;
const TAKEOFF_DURATION: f64 = 5.0;
const LANDING_DURATION: f64 = 8.0;
const LANDED_HOVER_DURATION: f64 = 10.0;

/// Flight lifecycle phases
#[derive(Debug, Clone, Copy, PartialEq)]
enum FlightPhase {
    Preflight, // On ground, preparing
    Takeoff,   // Ascending to cruise altitude
    Cruise,    // Flying to destination
    Landing,   // Descending at destination
    Landed,    // On ground at destination
}

impl std::fmt::Display for FlightPhase {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FlightPhase::Preflight => write!(f, "PREFLIGHT"),
            FlightPhase::Takeoff => write!(f, "TAKEOFF"),
            FlightPhase::Cruise => write!(f, "CRUISE"),
            FlightPhase::Landing => write!(f, "LANDING"),
            FlightPhase::Landed => write!(f, "LANDED"),
        }
    }
}

/// Golden Demo Scenario
#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "Golden Demo: Realistic drone lifecycle with collision"
)]
struct Args {
    /// ATC Server URL
    #[arg(long, default_value = "http://localhost:3000")]
    url: String,

    /// Reset state before running
    #[arg(long, default_value_t = true)]
    reset: bool,

    /// Shared registration token for `/v1/drones/register` (sent as `X-Registration-Token`)
    #[arg(long)]
    registration_token: Option<String>,

    /// Admin token for `/v1/admin/reset` (sent as `Authorization: Bearer <token>`)
    #[arg(long)]
    admin_token: Option<String>,

    /// Owner ID for user-specific drone tracking (e.g., 'guest')
    #[arg(long, default_value = "guest")]
    owner: String,
}

/// Drone state during simulation
struct DemoState {
    drone_id: String,
    client: AtcClient,
    command_rx: Option<mpsc::Receiver<Command>>,
    // Positions
    start_lat: f64,
    start_lon: f64,
    end_lat: f64,
    end_lon: f64,
    // Current actual position (updated after reroute)
    current_lat: f64,
    current_lon: f64,
    // Current state
    phase: FlightPhase,
    phase_start_time: f64,
    heading_deg: f64,
    current_speed_mps: f64,
    current_alt: f64,
    // Rerouting state
    is_rerouting: bool,
    reroute_waypoints: Vec<(f64, f64, f64)>,
    reroute_index: usize,
    // Hold state
    is_holding: bool,
    hold_until: Option<time::Instant>,
    // Altitude offset from ALTITUDE_CHANGE commands
    altitude_offset_m: f64,
    // Battery state (minutes)
    battery_capacity_min: f64,
    battery_reserve_min: f64,
    battery_remaining_min: f64,
    battery_warned: bool,
}

impl DemoState {
    fn new(
        drone_id: &str,
        client: AtcClient,
        command_rx: Option<mpsc::Receiver<Command>>,
        start: (f64, f64),
        end: (f64, f64),
    ) -> Self {
        let heading_rad = bearing(start.0, start.1, end.0, end.1);
        let mut heading = heading_rad.to_degrees();
        if heading < 0.0 {
            heading += 360.0;
        }

        Self {
            drone_id: drone_id.to_string(),
            client,
            command_rx,
            start_lat: start.0,
            start_lon: start.1,
            end_lat: end.0,
            end_lon: end.1,
            current_lat: start.0,
            current_lon: start.1,
            phase: FlightPhase::Preflight,
            phase_start_time: 0.0,
            heading_deg: heading,
            current_speed_mps: 0.0,
            current_alt: 0.0, // Start on ground
            is_rerouting: false,
            reroute_waypoints: Vec::new(),
            reroute_index: 0,
            is_holding: false,
            hold_until: None,
            altitude_offset_m: 0.0,
            battery_capacity_min: BATTERY_CAPACITY_MIN,
            battery_reserve_min: BATTERY_RESERVE_MIN,
            battery_remaining_min: BATTERY_CAPACITY_MIN,
            battery_warned: false,
        }
    }

    /// Update phase based on elapsed time and current state
    fn update_phase(&mut self, elapsed: f64) {
        let phase_elapsed = elapsed - self.phase_start_time;

        match self.phase {
            FlightPhase::Preflight => {
                if phase_elapsed >= PREFLIGHT_DURATION {
                    self.phase = FlightPhase::Takeoff;
                    self.phase_start_time = elapsed;
                }
            }
            FlightPhase::Takeoff => {
                // Altitude increases during takeoff
                let takeoff_progress = (phase_elapsed / TAKEOFF_DURATION).clamp(0.0, 1.0);
                self.current_alt = takeoff_progress * CRUISE_ALTITUDE_M;

                if phase_elapsed >= TAKEOFF_DURATION {
                    self.phase = FlightPhase::Cruise;
                    self.phase_start_time = elapsed;
                    self.current_alt = CRUISE_ALTITUDE_M;
                }
            }
            FlightPhase::Cruise => {
                // Cruise completion is determined by distance, checked in main loop
            }
            FlightPhase::Landing => {
                // Altitude decreases during landing
                let landing_progress = (phase_elapsed / LANDING_DURATION).clamp(0.0, 1.0);
                self.current_alt = CRUISE_ALTITUDE_M * (1.0 - landing_progress);

                if phase_elapsed >= LANDING_DURATION {
                    self.phase = FlightPhase::Landed;
                    self.phase_start_time = elapsed;
                    self.current_alt = 0.0;
                }
            }
            FlightPhase::Landed => {
                self.current_alt = 0.0;
            }
        }
    }

    fn update_motion(
        &mut self,
        target_lat: f64,
        target_lon: f64,
        dt_s: f64,
        target_speed_mps: f64,
    ) {
        if dt_s <= 0.0 {
            return;
        }

        let speed_delta = target_speed_mps - self.current_speed_mps;
        let max_speed_delta = MAX_ACCEL_MPS2 * dt_s;
        self.current_speed_mps += speed_delta.clamp(-max_speed_delta, max_speed_delta);
        if self.current_speed_mps.abs() < 0.01 {
            self.current_speed_mps = 0.0;
        }

        if self.current_speed_mps <= 0.0 {
            return;
        }

        let desired_heading =
            bearing(self.current_lat, self.current_lon, target_lat, target_lon).to_degrees();
        let desired_heading = if desired_heading < 0.0 {
            desired_heading + 360.0
        } else {
            desired_heading
        };
        let heading_delta = Self::shortest_heading_delta(desired_heading, self.heading_deg);
        let max_turn = MAX_TURN_RATE_DEG_PER_S * dt_s;
        self.heading_deg = Self::normalize_heading_deg(
            self.heading_deg + heading_delta.clamp(-max_turn, max_turn),
        );

        let remaining =
            haversine_distance(self.current_lat, self.current_lon, target_lat, target_lon);
        let travel = (self.current_speed_mps * dt_s).min(remaining);
        if travel <= 0.0 {
            return;
        }
        let (lat, lon) = offset_by_bearing(
            self.current_lat,
            self.current_lon,
            travel,
            self.heading_deg.to_radians(),
        );
        self.current_lat = lat;
        self.current_lon = lon;
    }

    fn normalize_heading_deg(heading: f64) -> f64 {
        let mut heading = heading % 360.0;
        if heading < 0.0 {
            heading += 360.0;
        }
        heading
    }

    fn shortest_heading_delta(target: f64, current: f64) -> f64 {
        let mut delta = target - current;
        while delta > 180.0 {
            delta -= 360.0;
        }
        while delta < -180.0 {
            delta += 360.0;
        }
        delta
    }

    fn drain_battery(&mut self, dt_s: f64, speed_mps: f64, airborne: bool) {
        if dt_s <= 0.0 {
            return;
        }
        let drain_factor = if speed_mps > 0.0 {
            (speed_mps / DRONE_SPEED_MPS).max(0.0)
        } else if airborne {
            BATTERY_HOVER_DRAIN_FACTOR
        } else {
            0.0
        };
        let drain_min = (dt_s / 60.0) * drain_factor;
        self.battery_remaining_min = (self.battery_remaining_min - drain_min).max(0.0);
    }
}

async fn spawn_command_listener(
    drone_id: &str,
    client: &AtcClient,
) -> Option<mpsc::Receiver<Command>> {
    let mut stream = match client.connect_command_stream().await {
        Ok(stream) => stream,
        Err(err) => {
            eprintln!(
                "[CMD] {} WebSocket unavailable, falling back to polling: {}",
                drone_id, err
            );
            return None;
        }
    };

    let (tx, rx) = mpsc::channel(COMMAND_CHANNEL_SIZE);
    let drone_id = drone_id.to_string();
    tokio::spawn(async move {
        loop {
            match stream.next_command().await {
                Ok(Some(cmd)) => {
                    if tx.send(cmd).await.is_err() {
                        break;
                    }
                }
                Ok(None) => {
                    eprintln!("[CMD] {} command stream closed", drone_id);
                    break;
                }
                Err(err) => {
                    eprintln!("[CMD] {} command stream error: {}", drone_id, err);
                    break;
                }
            }
        }
    });

    Some(rx)
}

async fn handle_command(drone: &mut DemoState, cmd: Command) {
    println!("\n  ╔════════════════════════════════════════════════════════════╗");
    println!("  ║  COMMAND: {:?}", cmd.command_type);
    println!("  ╚════════════════════════════════════════════════════════════╝");

    match cmd.command_type {
        CommandType::Hold { duration_secs } => {
            drone.is_holding = true;
            drone.hold_until =
                Some(time::Instant::now() + Duration::from_secs(duration_secs as u64));
            println!("  [CMD] {} HOLD for {}s\n", drone.drone_id, duration_secs);
        }
        CommandType::Resume => {
            drone.is_holding = false;
            drone.is_rerouting = false;
            drone.altitude_offset_m = 0.0; // Reset altitude modifications
            println!("  [CMD] {} RESUME\n", drone.drone_id);
        }
        CommandType::Reroute { waypoints, reason } => {
            drone.is_rerouting = true;
            drone.reroute_waypoints = waypoints
                .iter()
                .map(|wp| (wp.lat, wp.lon, wp.altitude_m))
                .collect();
            drone.reroute_index = 0;
            println!(
                "  [CMD] {} REROUTE ({} waypoints)",
                drone.drone_id,
                waypoints.len()
            );
            println!("        Reason: {}\n", reason.as_deref().unwrap_or("none"));
        }
        CommandType::AltitudeChange { target_altitude_m } => {
            // Calculate offset from current cruise altitude
            drone.altitude_offset_m = target_altitude_m - CRUISE_ALTITUDE_M;
            println!(
                "  [CMD] {} ALT_CHANGE → {}m (offset: {:+.0}m)\n",
                drone.drone_id, target_altitude_m, drone.altitude_offset_m
            );
        }
    }

    let _ = drone.client.ack_command(&cmd.command_id).await;
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let registration_token = args
        .registration_token
        .clone()
        .or_else(|| env::var("ATC_REGISTRATION_TOKEN").ok())
        .unwrap_or_else(|| "change-me".to_string());
    let admin_token = args
        .admin_token
        .clone()
        .or_else(|| env::var("ATC_ADMIN_TOKEN").ok())
        .unwrap_or_else(|| "change-me-admin".to_string());

    println!("╔═══════════════════════════════════════════════════════════════╗");
    println!("║    ATC-DRONE: REALISTIC LIFECYCLE DEMO                        ║");
    println!("║    Ground → Takeoff → Cruise → Collision → Land               ║");
    println!("╚═══════════════════════════════════════════════════════════════╝");
    println!();

    // Reset server state
    if args.reset {
        println!("[SETUP] Resetting server state...");
        let client = reqwest::Client::new();
        let response = client
            .post(format!("{}/v1/admin/reset", args.url))
            .bearer_auth(&admin_token)
            .json(&json!({ "confirm": "RESET", "require_idle": false }))
            .send()
            .await;

        match response {
            Ok(resp) if resp.status().is_success() => println!("[SETUP] ✓ Server state cleared"),
            Ok(resp) => {
                let status = resp.status();
                let body = resp.text().await.unwrap_or_default();
                println!("[SETUP] ⚠ Reset status: {} {}", status, body.trim());
            }
            Err(e) => println!(
                "[SETUP] ⚠ Could not reset: {} (check --admin-token / ATC_ADMIN_TOKEN)",
                e
            ),
        }
    }

    println!();
    println!("[SCENARIO]");
    println!("  • Alpha: Starts on ground WEST of center → flies EAST → lands EAST");
    println!("  • Beta:  Starts on ground EAST of center → flies WEST → lands WEST");
    println!("  • Collision expected mid-flight → server issues REROUTE");
    println!();

    // Create and register drones
    let mut alpha_client = AtcClient::new(&args.url);
    let mut beta_client = AtcClient::new(&args.url);

    println!("[REGISTER] Registering drones...");
    alpha_client.set_registration_token(Some(registration_token.clone()));
    beta_client.set_registration_token(Some(registration_token.clone()));
    alpha_client
        .register_with_owner(Some("demo-alpha"), Some(&args.owner))
        .await?;
    beta_client
        .register_with_owner(Some("demo-beta"), Some(&args.owner))
        .await?;
    println!("[REGISTER] ✓ Both drones registered\n");

    let alpha_command_rx = spawn_command_listener("demo-alpha", &alpha_client).await;
    let beta_command_rx = spawn_command_listener("demo-beta", &beta_client).await;

    // Create demo states
    let (alpha_start_lat, alpha_start_lon) =
        offset_by_bearing(IRVINE_LAT, IRVINE_LON, OFFSET_M, 270.0_f64.to_radians());
    let (alpha_end_lat, alpha_end_lon) =
        offset_by_bearing(IRVINE_LAT, IRVINE_LON, OFFSET_M, 90.0_f64.to_radians());
    let mut alpha = DemoState::new(
        "demo-alpha",
        alpha_client,
        alpha_command_rx,
        (alpha_start_lat, alpha_start_lon),
        (alpha_end_lat, alpha_end_lon),
    );

    let (beta_start_lat, beta_start_lon) =
        offset_by_bearing(IRVINE_LAT, IRVINE_LON, OFFSET_M, 90.0_f64.to_radians());
    let (beta_end_lat, beta_end_lon) =
        offset_by_bearing(IRVINE_LAT, IRVINE_LON, OFFSET_M, 270.0_f64.to_radians());
    let mut beta = DemoState::new(
        "demo-beta",
        beta_client,
        beta_command_rx,
        (beta_start_lat, beta_start_lon),
        (beta_end_lat, beta_end_lon),
    );

    // Calculate cruise duration
    let distance_m = haversine_distance(
        alpha.start_lat,
        alpha.start_lon,
        alpha.end_lat,
        alpha.end_lon,
    );
    let cruise_duration = distance_m / DRONE_SPEED_MPS;

    let total_duration = PREFLIGHT_DURATION
        + TAKEOFF_DURATION
        + cruise_duration
        + LANDING_DURATION
        + LANDED_HOVER_DURATION;

    println!("╔═══════════════════════════════════════════════════════════════╗");
    println!("║  SIMULATION START                                             ║");
    println!(
        "║  Distance: {:.0}m | Cruise: {:.1}s | Total: {:.0}s              ║",
        distance_m, cruise_duration, total_duration
    );
    println!("╚═══════════════════════════════════════════════════════════════╝\n");

    let start_time = time::Instant::now();
    let owner_id = args.owner.clone(); // For telemetry owner tracking
    let mut update_count = 0u32;
    let mut interval = time::interval(Duration::from_secs_f64(1.0 / UPDATE_RATE_HZ));
    let mut last_elapsed = 0.0;

    loop {
        interval.tick().await;
        let elapsed = start_time.elapsed().as_secs_f64();
        let dt = (elapsed - last_elapsed).max(0.0);
        last_elapsed = elapsed;

        if elapsed > total_duration {
            println!("\n╔═══════════════════════════════════════════════════════════════╗");
            println!(
                "║  DEMO COMPLETE - {} updates | Both drones landed             ║",
                update_count
            );
            println!("╚═══════════════════════════════════════════════════════════════╝");
            break;
        }

        update_count += 1;

        for drone in [&mut alpha, &mut beta] {
            // Check hold expiry
            if let Some(until) = drone.hold_until {
                if time::Instant::now() >= until {
                    drone.is_holding = false;
                    drone.hold_until = None;
                }
            }

            // Update phase
            drone.update_phase(elapsed);

            let (target_lat, target_lon, target_alt) =
                if drone.is_rerouting && drone.reroute_index < drone.reroute_waypoints.len() {
                    drone.reroute_waypoints[drone.reroute_index]
                } else {
                    (drone.end_lat, drone.end_lon, drone.current_alt)
                };

            let target_speed = if drone.is_holding
                || matches!(
                    drone.phase,
                    FlightPhase::Preflight
                        | FlightPhase::Takeoff
                        | FlightPhase::Landing
                        | FlightPhase::Landed
                ) {
                0.0
            } else {
                DRONE_SPEED_MPS
            };
            drone.update_motion(target_lat, target_lon, dt, target_speed);

            let lat = drone.current_lat;
            let lon = drone.current_lon;

            if drone.is_rerouting && drone.reroute_index < drone.reroute_waypoints.len() {
                let target = drone.reroute_waypoints[drone.reroute_index];
                let dist_to_target = haversine_distance(lat, lon, target.0, target.1);
                if dist_to_target <= WAYPOINT_REACHED_M {
                    drone.current_lat = target.0;
                    drone.current_lon = target.1;
                    drone.current_alt = target.2;

                    drone.reroute_index += 1;
                    if drone.reroute_index >= drone.reroute_waypoints.len() {
                        drone.is_rerouting = false;
                        drone.reroute_waypoints.clear();
                        drone.reroute_index = 0;
                        println!(
                            "[{:3}] {} reroute complete, continuing from new position",
                            update_count, drone.drone_id
                        );
                    }
                }
            }

            // Check if we reached destination (distance-based transition)
            if drone.phase == FlightPhase::Cruise && !drone.is_rerouting && !drone.is_holding {
                let dist_to_end = haversine_distance(lat, lon, drone.end_lat, drone.end_lon);
                if dist_to_end < 10.0 {
                    // Within 10 meters
                    drone.phase = FlightPhase::Landing;
                    drone.phase_start_time = elapsed;
                    println!(
                        "[{:3}] {} arrived at destination, starting landing",
                        update_count, drone.drone_id
                    );
                }
            }

            // Calculate altitude with offset applied during cruise
            let alt = if drone.is_rerouting && drone.reroute_index < drone.reroute_waypoints.len() {
                // During reroute: use waypoint altitude + any additional offset
                target_alt + drone.altitude_offset_m
            } else if drone.phase == FlightPhase::Cruise {
                // Normal cruise: apply offset to cruise altitude
                drone.current_alt + drone.altitude_offset_m
            } else {
                // Takeoff/landing: use phase altitude (no offset during transitions)
                drone.current_alt
            };

            let speed = drone.current_speed_mps;

            let airborne = !matches!(drone.phase, FlightPhase::Preflight | FlightPhase::Landed);
            drone.drain_battery(dt, speed, airborne);

            if drone.battery_remaining_min <= drone.battery_reserve_min {
                if !drone.battery_warned {
                    drone.battery_warned = true;
                    println!(
                        "[{:3}] {} battery reserve reached ({:.1} min left), forcing landing",
                        update_count, drone.drone_id, drone.battery_remaining_min
                    );
                }
                if matches!(
                    drone.phase,
                    FlightPhase::Cruise | FlightPhase::Takeoff | FlightPhase::Preflight
                ) {
                    drone.is_holding = false;
                    drone.is_rerouting = false;
                    drone.phase = FlightPhase::Landing;
                    drone.phase_start_time = elapsed;
                }
            }

            // Send telemetry with owner ID
            match drone
                .client
                .send_position_with_owner(
                    lat,
                    lon,
                    alt,
                    drone.heading_deg,
                    speed,
                    Some(owner_id.clone()),
                )
                .await
            {
                Ok(_) => {
                    let status = if drone.is_rerouting {
                        format!(
                            "REROUTE[{}/{}]",
                            drone.reroute_index + 1,
                            drone.reroute_waypoints.len()
                        )
                    } else if drone.is_holding {
                        "HOLD".to_string()
                    } else {
                        format!("{}", drone.phase)
                    };

                    println!(
                        "[{:3}] {}: ({:.6}, {:.6}) @ {:.0}m | {}",
                        update_count, drone.drone_id, lat, lon, alt, status
                    );
                }
                Err(e) => eprintln!("[{:3}] {} ERROR: {}", update_count, drone.drone_id, e),
            }

            let mut poll_fallback = drone.command_rx.is_none();
            let mut received_commands = Vec::new();
            let mut disconnected = false;

            if let Some(rx) = drone.command_rx.as_mut() {
                loop {
                    match rx.try_recv() {
                        Ok(cmd) => received_commands.push(cmd),
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => {
                            disconnected = true;
                            break;
                        }
                    }
                }
            }

            if disconnected {
                eprintln!(
                    "[CMD] {} command stream disconnected, falling back to polling",
                    drone.drone_id
                );
                drone.command_rx = None;
                poll_fallback = true;
            }

            for cmd in received_commands {
                handle_command(drone, cmd).await;
            }

            if poll_fallback {
                if let Ok(Some(cmd)) = drone.client.get_next_command().await {
                    handle_command(drone, cmd).await;
                }
            }
        }
    }

    Ok(())
}

// Removed local haversine_distance. Using atc_core::spatial::haversine_distance instead.
