//! Golden Demo Scenario - Deterministic collision test.
//!
//! This script runs a scripted scenario with two drones on a guaranteed
//! collision course. It demonstrates the full ATC loop:
//!
//! 1. Drones register and fly toward each other
//! 2. Server detects conflict at ~T=20s
//! 3. Server issues REROUTE command to lower-priority drone
//! 4. Drone diverts, conflict resolves
//!
//! Usage:
//!   cargo run -p atc-cli --bin demo_scenario

use atc_core::models::CommandType;
use atc_sdk::AtcClient;
use clap::Parser;
use std::time::Duration;
use tokio::time;

/// Irvine coordinates (flight hub)
const IRVINE_LAT: f64 = 33.6846;
const IRVINE_LON: f64 = -117.8265;

/// Demo parameters
const FLIGHT_ALTITUDE_M: f64 = 50.0;
const DRONE_SPEED_MPS: f64 = 10.0;
const DEMO_DURATION_SECS: u64 = 90;
const UPDATE_RATE_HZ: f64 = 1.0;

/// ~300m offset in degrees (collision at center after ~30s)
const OFFSET_DEG: f64 = 0.003;

/// Golden Demo Scenario
#[derive(Parser, Debug)]
#[command(author, version, about = "Golden Demo: Deterministic collision scenario")]
struct Args {
    /// ATC Server URL
    #[arg(long, default_value = "http://localhost:3000")]
    url: String,

    /// Reset state before running
    #[arg(long, default_value_t = true)]
    reset: bool,
}

/// Drone state during simulation
struct DemoState {
    drone_id: String,
    client: AtcClient,
    // Starting position
    start_lat: f64,
    start_lon: f64,
    // Ending position
    end_lat: f64,
    end_lon: f64,
    // Heading (degrees, 0 = north)
    heading: f64,
    // Current state
    is_rerouting: bool,
    reroute_waypoints: Vec<(f64, f64, f64)>, // (lat, lon, alt)
    reroute_index: usize,
    is_holding: bool,
    hold_until: Option<time::Instant>,
}

impl DemoState {
    fn new(drone_id: &str, client: AtcClient, start: (f64, f64), end: (f64, f64)) -> Self {
        // Calculate heading from start to end
        let dlat = end.0 - start.0;
        let dlon = end.1 - start.1;
        let heading_rad = dlon.atan2(dlat);
        let mut heading = heading_rad.to_degrees();
        if heading < 0.0 {
            heading += 360.0;
        }

        Self {
            drone_id: drone_id.to_string(),
            client,
            start_lat: start.0,
            start_lon: start.1,
            end_lat: end.0,
            end_lon: end.1,
            heading,
            is_rerouting: false,
            reroute_waypoints: Vec::new(),
            reroute_index: 0,
            is_holding: false,
            hold_until: None,
        }
    }

    /// Get position along flight path at progress [0.0, 1.0]
    fn get_position(&self, progress: f64) -> (f64, f64, f64) {
        let p = progress.clamp(0.0, 1.0);
        let lat = self.start_lat + p * (self.end_lat - self.start_lat);
        let lon = self.start_lon + p * (self.end_lon - self.start_lon);
        (lat, lon, FLIGHT_ALTITUDE_M)
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    println!("╔═══════════════════════════════════════════════════════════════╗");
    println!("║          ATC-DRONE: GOLDEN DEMO SCENARIO                      ║");
    println!("║          Deterministic Collision + Resolution                 ║");
    println!("╚═══════════════════════════════════════════════════════════════╝");
    println!();

    // Reset server state if requested
    if args.reset {
        println!("[SETUP] Resetting server state...");
        let client = reqwest::Client::new();
        match client.post(format!("{}/v1/admin/reset", args.url)).send().await {
            Ok(resp) if resp.status().is_success() => {
                println!("[SETUP] ✓ Server state cleared");
            }
            Ok(resp) => {
                println!("[SETUP] ⚠ Reset returned status: {}", resp.status());
            }
            Err(e) => {
                println!("[SETUP] ⚠ Could not reset server: {} (continuing anyway)", e);
            }
        }
    }

    println!();
    println!("[SCENARIO] Two drones flying head-on collision course");
    println!("           Alpha: West → East (priority = older)");
    println!("           Beta:  East → West (will receive REROUTE)");
    println!("           Collision expected at center at T≈30s");
    println!();

    // Create clients for both drones
    let mut alpha_client = AtcClient::new(&args.url);
    let mut beta_client = AtcClient::new(&args.url);

    // Register drones with fixed IDs for reproducibility
    println!("[REGISTER] Registering demo-alpha...");
    alpha_client.register(Some("demo-alpha")).await?;
    println!("[REGISTER] Registering demo-beta...");
    beta_client.register(Some("demo-beta")).await?;
    println!("[REGISTER] ✓ Both drones registered");
    println!();

    // Create demo states
    // Alpha: West to East (starts 300m west of center)
    let mut alpha = DemoState::new(
        "demo-alpha",
        alpha_client,
        (IRVINE_LAT, IRVINE_LON - OFFSET_DEG), // start: west
        (IRVINE_LAT, IRVINE_LON + OFFSET_DEG), // end: east
    );

    // Beta: East to West (starts 300m east of center) - will collide with alpha
    let mut beta = DemoState::new(
        "demo-beta",
        beta_client,
        (IRVINE_LAT, IRVINE_LON + OFFSET_DEG), // start: east
        (IRVINE_LAT, IRVINE_LON - OFFSET_DEG), // end: west
    );

    println!("╔═══════════════════════════════════════════════════════════════╗");
    println!("║  STARTING SIMULATION - Watch for conflict + reroute           ║");
    println!("╚═══════════════════════════════════════════════════════════════╝");
    println!();

    let start_time = time::Instant::now();
    let mut update_count = 0u32;
    let mut interval = time::interval(Duration::from_secs_f64(1.0 / UPDATE_RATE_HZ));

    // Calculate total flight time based on distance and speed
    let distance_m = haversine_distance(alpha.start_lat, alpha.start_lon, alpha.end_lat, alpha.end_lon);
    let flight_duration_secs = distance_m / DRONE_SPEED_MPS;
    println!("[INFO] Flight distance: {:.0}m, duration: {:.1}s", distance_m, flight_duration_secs);
    println!();

    loop {
        interval.tick().await;

        let elapsed = start_time.elapsed().as_secs_f64();
        if elapsed > DEMO_DURATION_SECS as f64 {
            println!();
            println!("╔═══════════════════════════════════════════════════════════════╗");
            println!("║  DEMO COMPLETE - {} updates sent                           ║", update_count);
            println!("╚═══════════════════════════════════════════════════════════════╝");
            break;
        }

        update_count += 1;
        let progress = (elapsed / flight_duration_secs).min(1.0);

        // Update both drones
        for drone in [&mut alpha, &mut beta] {
            // Check HOLD expiry
            if let Some(until) = drone.hold_until {
                if time::Instant::now() >= until {
                    drone.is_holding = false;
                    drone.hold_until = None;
                    println!("[{:3}] {} HOLD expired, resuming", update_count, drone.drone_id);
                }
            }

            // Determine current position
            let (lat, lon, alt, speed) = if drone.is_rerouting && drone.reroute_index < drone.reroute_waypoints.len() {
                let wp = drone.reroute_waypoints[drone.reroute_index];
                (wp.0, wp.1, wp.2, DRONE_SPEED_MPS)
            } else if drone.is_holding {
                // Frozen position
                let pos = drone.get_position(progress);
                (pos.0, pos.1, pos.2, 0.0)
            } else {
                let pos = drone.get_position(progress);
                (pos.0, pos.1, pos.2, DRONE_SPEED_MPS)
            };

            // Send telemetry
            match drone.client.send_position(lat, lon, alt, drone.heading, speed).await {
                Ok(_) => {
                    let status = if drone.is_rerouting { 
                        format!("REROUTE[{}/{}]", drone.reroute_index + 1, drone.reroute_waypoints.len())
                    } else if drone.is_holding { 
                        "HOLD".to_string()
                    } else { 
                        format!("{:.0}%", progress * 100.0)
                    };
                    println!(
                        "[{:3}] {}: ({:.6}, {:.6}) @ {:.0}m -> {}",
                        update_count, drone.drone_id, lat, lon, alt, status
                    );
                }
                Err(e) => {
                    eprintln!("[{:3}] {} ERROR: {}", update_count, drone.drone_id, e);
                }
            }

            // Poll for commands
            if let Ok(Some(cmd)) = drone.client.get_next_command().await {
                println!();
                println!("  ╔════════════════════════════════════════════════════════════╗");
                println!("  ║  COMMAND RECEIVED: {:?}", cmd.command_type);
                println!("  ╚════════════════════════════════════════════════════════════╝");
                
                match cmd.command_type {
                    CommandType::Hold { duration_secs } => {
                        drone.is_holding = true;
                        drone.hold_until = Some(time::Instant::now() + Duration::from_secs(duration_secs as u64));
                        println!("  [CMD] {} executing HOLD for {}s", drone.drone_id, duration_secs);
                    }
                    CommandType::Resume => {
                        drone.is_holding = false;
                        drone.is_rerouting = false;
                        println!("  [CMD] {} RESUME", drone.drone_id);
                    }
                    CommandType::Reroute { waypoints, reason } => {
                        drone.is_rerouting = true;
                        drone.reroute_waypoints = waypoints.iter()
                            .map(|wp| (wp.lat, wp.lon, wp.altitude_m))
                            .collect();
                        drone.reroute_index = 0;
                        println!("  [CMD] {} REROUTE with {} waypoints", drone.drone_id, waypoints.len());
                        println!("        Reason: {}", reason.as_deref().unwrap_or("none"));
                    }
                    CommandType::AltitudeChange { target_altitude_m } => {
                        println!("  [CMD] {} ALTITUDE_CHANGE to {}m", drone.drone_id, target_altitude_m);
                    }
                }
                println!();

                // Acknowledge command
                let _ = drone.client.ack_command(&cmd.command_id).await;
            }

            // Advance reroute waypoint
            if drone.is_rerouting && update_count % 5 == 0 {
                drone.reroute_index += 1;
                if drone.reroute_index >= drone.reroute_waypoints.len() {
                    drone.is_rerouting = false;
                    drone.reroute_waypoints.clear();
                    drone.reroute_index = 0;
                    println!("[{:3}] {} REROUTE complete", update_count, drone.drone_id);
                }
            }
        }
    }

    Ok(())
}

/// Calculate distance between two points in meters (Haversine formula)
fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0;
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
}
