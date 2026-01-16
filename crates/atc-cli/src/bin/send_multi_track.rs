use atc_cli::sim::{
    create_converging_scenario, create_crossing_scenario, create_parallel_scenario,
};
use atc_core::models::{CommandType, Waypoint};
use atc_sdk::AtcClient;
use clap::{Parser, ValueEnum};
use std::collections::HashMap;
use std::time::Duration;
use tokio::time;

/// Available test scenarios
#[derive(Debug, Clone, ValueEnum)]
enum ScenarioType {
    /// Two drones on collision course
    Crossing,
    /// Two drones flying parallel paths
    Parallel,
    /// Multiple drones converging on a point
    Converging,
}

/// Drone control state
#[derive(Debug, Clone, Default)]
struct DroneControl {
    is_holding: bool,
    hold_until: Option<time::Instant>,
    frozen_position: Option<(f64, f64, f64)>,
    // Reroute support
    is_rerouting: bool,
    reroute_waypoints: Vec<Waypoint>,
    reroute_index: usize,
    reroute_start_time: Option<time::Instant>,
    // Altitude change support
    altitude_offset_m: f64,
}

/// Multi-drone simulator with conflict scenarios
#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// ATC Server URL
    #[arg(long, default_value = "http://localhost:3000")]
    url: String,

    /// Scenario to simulate
    #[arg(long, value_enum, default_value = "crossing")]
    scenario: ScenarioType,

    /// Center latitude (default: Irvine, CA)
    #[arg(long, default_value_t = 33.6846)]
    lat: f64,

    /// Center longitude (default: Irvine, CA)
    #[arg(long, default_value_t = -117.8265)]
    lon: f64,

    /// Duration in seconds
    #[arg(long, default_value_t = 60)]
    duration: u64,

    /// Update rate in Hz
    #[arg(long, default_value_t = 1.0)]
    rate: f64,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    // Create scenario
    let scenario = match args.scenario {
        ScenarioType::Crossing => create_crossing_scenario(args.lat, args.lon),
        ScenarioType::Parallel => create_parallel_scenario(args.lat, args.lon),
        ScenarioType::Converging => create_converging_scenario(args.lat, args.lon),
    };

    println!("\nScenario: {}", scenario.name);
    println!("Connecting to ATC Server at {}...", args.url);
    
    // Create and register a client for each drone
    let mut clients: HashMap<String, AtcClient> = HashMap::new();
    let mut controls: HashMap<String, DroneControl> = HashMap::new();
    
    for (drone_id, _) in &scenario.drones {
        let mut client = AtcClient::new(&args.url);
        match client.register(Some(drone_id)).await {
            Ok(resp) => {
                println!("  Registered: {}", resp.drone_id);
                clients.insert(drone_id.clone(), client);
                controls.insert(drone_id.clone(), DroneControl::default());
            }
            Err(e) => {
                eprintln!("  Failed to register {}: {}", drone_id, e);
            }
        }
    }

    if clients.is_empty() {
        anyhow::bail!("No drones registered successfully");
    }

    println!("\nStarting multi-drone simulation (with command handling)");
    println!("  Drones: {}", clients.len());
    println!("  Duration: {}s, Update rate: {}Hz\n", args.duration, args.rate);

    let start = time::Instant::now();
    let mut update_count = 0u32;
    let mut interval = time::interval(Duration::from_secs_f64(1.0 / args.rate));

    loop {
        interval.tick().await;

        let elapsed = start.elapsed().as_secs_f64();
        if elapsed > args.duration as f64 {
            break;
        }

        update_count += 1;

        // Update all drones
        for (drone_id, path) in &scenario.drones {
            if let Some(client) = clients.get(drone_id) {
                let control = controls.get_mut(drone_id).unwrap();
                
                // Check if HOLD has expired
                if let Some(until) = control.hold_until {
                    if time::Instant::now() >= until {
                        control.is_holding = false;
                        control.hold_until = None;
                        control.frozen_position = None;
                        println!("[{:3}] {} HOLD expired, resuming", update_count, drone_id);
                    }
                }

                // Get position based on state: rerouting > holding > normal path
                let (lat, lon, alt, heading, speed) = if control.is_rerouting && control.reroute_index < control.reroute_waypoints.len() {
                    // Copy waypoint data to avoid borrow issues
                    let wp_lat = control.reroute_waypoints[control.reroute_index].lat;
                    let wp_lon = control.reroute_waypoints[control.reroute_index].lon;
                    let wp_alt = control.reroute_waypoints[control.reroute_index].altitude_m;
                    let wp_speed = control.reroute_waypoints[control.reroute_index].speed_mps.unwrap_or(10.0);
                    let wp_count = control.reroute_waypoints.len();
                    
                    let elapsed_reroute = control.reroute_start_time
                        .map(|t| t.elapsed().as_secs_f64())
                        .unwrap_or(0.0);
                    
                    // Check if time to advance to next waypoint (5 sec per waypoint or close enough)
                    if elapsed_reroute > 5.0 {
                        control.reroute_index += 1;
                        control.reroute_start_time = Some(time::Instant::now());
                        
                        if control.reroute_index >= wp_count {
                            // Reroute complete
                            control.is_rerouting = false;
                            control.reroute_waypoints.clear();
                            control.reroute_index = 0;
                            println!("[{:3}] {} REROUTE complete, resuming normal path", update_count, drone_id);
                        }
                    }
                    
                    // Calculate heading toward waypoint
                    let curr_pos = path.get_position(elapsed);
                    let delta_lat = wp_lat - curr_pos.0;
                    let delta_lon = wp_lon - curr_pos.1;
                    let hdg = delta_lon.atan2(delta_lat).to_degrees();
                    
                    (wp_lat, wp_lon, wp_alt, hdg, wp_speed)
                } else if control.is_rerouting {
                    // Reroute index out of bounds - complete reroute
                    control.is_rerouting = false;
                    control.reroute_waypoints.clear();
                    control.reroute_index = 0;
                    let pos = path.get_position(elapsed);
                    (pos.0, pos.1, pos.2, path.get_heading(elapsed), path.get_speed_mps())
                } else if control.is_holding {
                    let pos = control.frozen_position.unwrap_or_else(|| path.get_position(elapsed));
                    (pos.0, pos.1, pos.2 + control.altitude_offset_m, path.get_heading(elapsed), 0.0)
                } else {
                    let pos = path.get_position(elapsed);
                    (pos.0, pos.1, pos.2 + control.altitude_offset_m, path.get_heading(elapsed), path.get_speed_mps())
                };

                // Send telemetry
                match client.send_position(lat, lon, alt, heading, speed).await {
                    Ok(_) => {
                        let status = if control.is_rerouting { 
                            format!("REROUTE[{}]", control.reroute_index) 
                        } else if control.is_holding { 
                            "HOLD".to_string() 
                        } else if control.altitude_offset_m.abs() > 0.1 {
                            format!("ALT{:+.0}m", control.altitude_offset_m)
                        } else { 
                            "OK".to_string() 
                        };
                        println!(
                            "[{:3}] {}: ({:.6}, {:.6}) -> {}",
                            update_count, drone_id, lat, lon, status
                        );
                    }
                    Err(e) => {
                        eprintln!("[{}] Error: {}", drone_id, e);
                    }
                }

                // Poll for commands every 2 updates
                if update_count.is_multiple_of(2) {
                    match client.get_next_command().await {
                        Ok(Some(cmd)) => {
                            println!("  [CMD] {} received: {:?}", drone_id, cmd.command_type);
                            
                            match cmd.command_type {
                                CommandType::Hold { duration_secs } => {
                                    control.is_holding = true;
                                    control.is_rerouting = false; // Cancel any reroute
                                    control.hold_until = Some(time::Instant::now() + Duration::from_secs(duration_secs as u64));
                                    control.frozen_position = Some((lat, lon, alt));
                                    println!("  [CMD] {} executing HOLD for {}s", drone_id, duration_secs);
                                }
                                CommandType::Resume => {
                                    control.is_holding = false;
                                    control.is_rerouting = false;
                                    control.hold_until = None;
                                    control.frozen_position = None;
                                    control.reroute_waypoints.clear();
                                    control.reroute_index = 0;
                                    println!("  [CMD] {} resuming flight", drone_id);
                                }
                                CommandType::Reroute { waypoints, reason } => {
                                    control.is_rerouting = true;
                                    control.is_holding = false;
                                    control.reroute_waypoints = waypoints.clone();
                                    control.reroute_index = 0;
                                    control.reroute_start_time = Some(time::Instant::now());
                                    println!(
                                        "  [CMD] {} executing REROUTE with {} waypoints ({})", 
                                        drone_id, 
                                        waypoints.len(),
                                        reason.as_deref().unwrap_or("no reason")
                                    );
                                }
                                CommandType::AltitudeChange { target_altitude_m } => {
                                    // Compute offset from current path altitude
                                    let current_alt = path.get_position(elapsed).2;
                                    control.altitude_offset_m = target_altitude_m - current_alt;
                                    println!(
                                        "  [CMD] {} ALTITUDE_CHANGE to {}m (offset: {:+.0}m)", 
                                        drone_id, target_altitude_m, control.altitude_offset_m
                                    );
                                }
                            }
                            
                            // Acknowledge the command
                            if let Err(e) = client.ack_command(&cmd.command_id).await {
                                eprintln!("  Failed to ack command: {}", e);
                            }
                        }
                        Ok(None) => {} // No command pending
                        Err(e) => {
                            eprintln!("  [CMD] Error polling: {}", e);
                        }
                    }
                }
            }
        }
    }

    let total_updates = update_count as usize * clients.len();
    println!("\nSimulation complete. Sent {} total updates.", total_updates);
    Ok(())
}

