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
const CRUISE_ALTITUDE_M: f64 = 50.0;
const DRONE_SPEED_MPS: f64 = 10.0;
const UPDATE_RATE_HZ: f64 = 1.0;

/// ~300m offset in degrees
const OFFSET_DEG: f64 = 0.003;

/// Lifecycle timing (seconds)
const PREFLIGHT_DURATION: f64 = 3.0;
const TAKEOFF_DURATION: f64 = 5.0;
const LANDING_DURATION: f64 = 8.0;
const LANDED_HOVER_DURATION: f64 = 10.0;

/// Flight lifecycle phases
#[derive(Debug, Clone, Copy, PartialEq)]
enum FlightPhase {
    Preflight,  // On ground, preparing
    Takeoff,    // Ascending to cruise altitude
    Cruise,     // Flying to destination
    Landing,    // Descending at destination
    Landed,     // On ground at destination
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
#[command(author, version, about = "Golden Demo: Realistic drone lifecycle with collision")]
struct Args {
    /// ATC Server URL
    #[arg(long, default_value = "http://localhost:3000")]
    url: String,

    /// Reset state before running
    #[arg(long, default_value_t = true)]
    reset: bool,

    /// Owner ID for user-specific drone tracking (e.g., 'guest')
    #[arg(long, default_value = "guest")]
    owner: String,
}

/// Drone state during simulation
struct DemoState {
    drone_id: String,
    client: AtcClient,
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
    cruise_start_time: f64,  // When cruise phase started (reset after reroute)
    heading: f64,
    current_alt: f64,
    // Rerouting state
    is_rerouting: bool,
    reroute_waypoints: Vec<(f64, f64, f64)>,
    reroute_index: usize,
    reroute_segment_start_time: f64,  // When we started flying towards current waypoint
    reroute_segment_start_pos: (f64, f64, f64), // Position when segment started (lat, lon, alt)
    // Hold state
    is_holding: bool,
    hold_until: Option<time::Instant>,
    // Altitude offset from ALTITUDE_CHANGE commands
    altitude_offset_m: f64,
}

impl DemoState {
    fn new(drone_id: &str, client: AtcClient, start: (f64, f64), end: (f64, f64)) -> Self {
        let dlat = end.0 - start.0;
        let dlon = end.1 - start.1;
        let heading_rad = dlon.atan2(dlat);
        let mut heading = heading_rad.to_degrees();
        if heading < 0.0 { heading += 360.0; }

        Self {
            drone_id: drone_id.to_string(),
            client,
            start_lat: start.0,
            start_lon: start.1,
            end_lat: end.0,
            end_lon: end.1,
            current_lat: start.0,
            current_lon: start.1,
            phase: FlightPhase::Preflight,
            phase_start_time: 0.0,
            cruise_start_time: 0.0,
            heading,
            current_alt: 0.0, // Start on ground
            is_rerouting: false,
            reroute_waypoints: Vec::new(),
            reroute_index: 0,
            reroute_segment_start_time: 0.0,
            reroute_segment_start_pos: (0.0, 0.0, 0.0),
            is_holding: false,
            hold_until: None,
            altitude_offset_m: 0.0,
        }
    }

    /// Update phase based on elapsed time and current state
    fn update_phase(&mut self, elapsed: f64, cruise_duration: f64) {
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

    /// Get current position based on phase and progress
    fn get_position(&self, cruise_elapsed: f64, cruise_duration: f64) -> (f64, f64) {
        match self.phase {
            FlightPhase::Preflight | FlightPhase::Takeoff => {
                // Stay at start position
                (self.start_lat, self.start_lon)
            }
            FlightPhase::Cruise => {
                if self.is_rerouting && self.reroute_index < self.reroute_waypoints.len() {
                    // Physics-based interpolation towards current waypoint
                    let target = self.reroute_waypoints[self.reroute_index];
                    let start_pos = self.reroute_segment_start_pos;
                    
                    let segment_dist = haversine_distance(start_pos.0, start_pos.1, target.0, target.1);
                    let segment_time = segment_dist / DRONE_SPEED_MPS;
                    let segment_elapsed = cruise_elapsed - self.reroute_segment_start_time;
                    let segment_progress = if segment_time > 0.01 {
                        (segment_elapsed / segment_time).clamp(0.0, 1.0)
                    } else {
                        1.0 // Already at waypoint
                    };
                    
                    let lat = start_pos.0 + segment_progress * (target.0 - start_pos.0);
                    let lon = start_pos.1 + segment_progress * (target.1 - start_pos.1);
                    (lat, lon)
                } else {
                    // Use current position as start (updated after reroute completes)
                    let remaining_distance = haversine_distance(
                        self.current_lat, self.current_lon,
                        self.end_lat, self.end_lon
                    );
                    let total_distance = haversine_distance(
                        self.start_lat, self.start_lon,
                        self.end_lat, self.end_lon
                    );
                    let remaining_time = remaining_distance / DRONE_SPEED_MPS;
                    let progress = (cruise_elapsed / remaining_time.max(0.1)).clamp(0.0, 1.0);
                    let lat = self.current_lat + progress * (self.end_lat - self.current_lat);
                    let lon = self.current_lon + progress * (self.end_lon - self.current_lon);
                    (lat, lon)
                }
            }
            FlightPhase::Landing | FlightPhase::Landed => {
                // At destination
                (self.end_lat, self.end_lon)
            }
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    println!("╔═══════════════════════════════════════════════════════════════╗");
    println!("║    ATC-DRONE: REALISTIC LIFECYCLE DEMO                        ║");
    println!("║    Ground → Takeoff → Cruise → Collision → Land               ║");
    println!("╚═══════════════════════════════════════════════════════════════╝");
    println!();

    // Reset server state
    if args.reset {
        println!("[SETUP] Resetting server state...");
        let client = reqwest::Client::new();
        match client.post(format!("{}/v1/admin/reset", args.url)).send().await {
            Ok(resp) if resp.status().is_success() => println!("[SETUP] ✓ Server state cleared"),
            Ok(resp) => println!("[SETUP] ⚠ Reset status: {}", resp.status()),
            Err(e) => println!("[SETUP] ⚠ Could not reset: {}", e),
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
    alpha_client.register(Some("demo-alpha")).await?;
    beta_client.register(Some("demo-beta")).await?;
    println!("[REGISTER] ✓ Both drones registered\n");

    // Create demo states
    let mut alpha = DemoState::new(
        "demo-alpha",
        alpha_client,
        (IRVINE_LAT, IRVINE_LON - OFFSET_DEG),
        (IRVINE_LAT, IRVINE_LON + OFFSET_DEG),
    );

    let mut beta = DemoState::new(
        "demo-beta",
        beta_client,
        (IRVINE_LAT, IRVINE_LON + OFFSET_DEG),
        (IRVINE_LAT, IRVINE_LON - OFFSET_DEG),
    );

    // Calculate cruise duration
    let distance_m = haversine_distance(alpha.start_lat, alpha.start_lon, alpha.end_lat, alpha.end_lon);
    let cruise_duration = distance_m / DRONE_SPEED_MPS;
    
    let total_duration = PREFLIGHT_DURATION + TAKEOFF_DURATION + cruise_duration + LANDING_DURATION + LANDED_HOVER_DURATION;

    println!("╔═══════════════════════════════════════════════════════════════╗");
    println!("║  SIMULATION START                                             ║");
    println!("║  Distance: {:.0}m | Cruise: {:.1}s | Total: {:.0}s              ║", distance_m, cruise_duration, total_duration);
    println!("╚═══════════════════════════════════════════════════════════════╝\n");

    let start_time = time::Instant::now();
    let owner_id = args.owner.clone();  // For telemetry owner tracking
    let mut update_count = 0u32;
    let mut interval = time::interval(Duration::from_secs_f64(1.0 / UPDATE_RATE_HZ));

    loop {
        interval.tick().await;
        let elapsed = start_time.elapsed().as_secs_f64();

        if elapsed > total_duration {
            println!("\n╔═══════════════════════════════════════════════════════════════╗");
            println!("║  DEMO COMPLETE - {} updates | Both drones landed             ║", update_count);
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

            // Calculate cruise elapsed time (relative to current segment)
            // If in preflight/takeoff, cruise_start_time is the end of takeoff phase
            let effective_start_time = if drone.cruise_start_time > 0.0 {
                drone.cruise_start_time
            } else {
                PREFLIGHT_DURATION + TAKEOFF_DURATION
            };
            
            let cruise_elapsed = (elapsed - effective_start_time).max(0.0);

            // Update phase
            drone.update_phase(elapsed, cruise_duration);

            // Get position
            let (lat, lon) = drone.get_position(cruise_elapsed, cruise_duration);

            // Check if we reached destination (distance-based transition)
            if drone.phase == FlightPhase::Cruise && !drone.is_rerouting && !drone.is_holding {
                let dist_to_end = haversine_distance(lat, lon, drone.end_lat, drone.end_lon);
                if dist_to_end < 10.0 { // Within 10 meters
                    drone.phase = FlightPhase::Landing;
                    drone.phase_start_time = elapsed;
                    println!("[{:3}] {} arrived at destination, starting landing", update_count, drone.drone_id);
                }
            }
            
            // Calculate altitude with offset applied during cruise
            let alt = if drone.is_rerouting && drone.reroute_index < drone.reroute_waypoints.len() {
                // During reroute: use waypoint altitude + any additional offset
                drone.reroute_waypoints[drone.reroute_index].2 + drone.altitude_offset_m
            } else if drone.phase == FlightPhase::Cruise {
                // Normal cruise: apply offset to cruise altitude
                drone.current_alt + drone.altitude_offset_m
            } else {
                // Takeoff/landing: use phase altitude (no offset during transitions)
                drone.current_alt
            };

            let speed = if drone.is_holding || drone.phase == FlightPhase::Preflight || drone.phase == FlightPhase::Landed {
                0.0
            } else {
                DRONE_SPEED_MPS
            };

            // Send telemetry with owner ID
            match drone.client.send_position_with_owner(lat, lon, alt, drone.heading, speed, Some(owner_id.clone())).await {
                Ok(_) => {
                    let status = if drone.is_rerouting {
                        format!("REROUTE[{}/{}]", drone.reroute_index + 1, drone.reroute_waypoints.len())
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

            // Poll for commands
            if let Ok(Some(cmd)) = drone.client.get_next_command().await {
                println!("\n  ╔════════════════════════════════════════════════════════════╗");
                println!("  ║  COMMAND: {:?}", cmd.command_type);
                println!("  ╚════════════════════════════════════════════════════════════╝");

                match cmd.command_type {
                    CommandType::Hold { duration_secs } => {
                        drone.is_holding = true;
                        drone.hold_until = Some(time::Instant::now() + Duration::from_secs(duration_secs as u64));
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
                        drone.reroute_waypoints = waypoints.iter()
                            .map(|wp| (wp.lat, wp.lon, wp.altitude_m))
                            .collect();
                        drone.reroute_index = 0;
                        // Initialize segment tracking with current position
                        drone.reroute_segment_start_time = cruise_elapsed;
                        drone.reroute_segment_start_pos = (lat, lon, alt);
                        println!("  [CMD] {} REROUTE ({} waypoints)", drone.drone_id, waypoints.len());
                        println!("        Reason: {}\n", reason.as_deref().unwrap_or("none"));
                    }
                    CommandType::AltitudeChange { target_altitude_m } => {
                        // Calculate offset from current cruise altitude
                        drone.altitude_offset_m = target_altitude_m - CRUISE_ALTITUDE_M;
                        println!("  [CMD] {} ALT_CHANGE → {}m (offset: {:+.0}m)\n", 
                            drone.drone_id, target_altitude_m, drone.altitude_offset_m);
                    }
                }

                let _ = drone.client.ack_command(&cmd.command_id).await;
            }

            // Advance reroute based on PHYSICS (distance/speed), not fixed time
            if drone.is_rerouting && drone.reroute_index < drone.reroute_waypoints.len() {
                let target = drone.reroute_waypoints[drone.reroute_index];
                let start_pos = drone.reroute_segment_start_pos;
                
                let segment_dist = haversine_distance(start_pos.0, start_pos.1, target.0, target.1);
                let segment_time = segment_dist / DRONE_SPEED_MPS;
                let segment_elapsed = cruise_elapsed - drone.reroute_segment_start_time;
                
                // Check if we've reached this waypoint
                if segment_elapsed >= segment_time {
                    // Update current position to this waypoint
                    drone.current_lat = target.0;
                    drone.current_lon = target.1;
                    drone.current_alt = target.2;
                    
                    drone.reroute_index += 1;
                    
                    if drone.reroute_index >= drone.reroute_waypoints.len() {
                        // All waypoints done, resume normal cruise
                        drone.cruise_start_time = elapsed;
                        drone.is_rerouting = false;
                        drone.reroute_waypoints.clear();
                        drone.reroute_index = 0;
                        println!("[{:3}] {} reroute complete, continuing from new position", update_count, drone.drone_id);
                    } else {
                        // Start next segment
                        drone.reroute_segment_start_time = cruise_elapsed;
                        drone.reroute_segment_start_pos = (target.0, target.1, target.2);
                    }
                }
            }
        }
    }

    Ok(())
}

fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0;
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
}
