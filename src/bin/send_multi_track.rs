//! CLI tool for multi-drone simulation with conflict scenarios.

use atc_drone::auth::generate_dummy_token;
use atc_drone::sim::{
    create_converging_scenario, create_crossing_scenario, create_parallel_scenario, BlenderClient,
};
use clap::{Parser, ValueEnum};
use std::thread;
use std::time::{Duration, Instant};

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

/// Multi-drone simulator with conflict scenarios
#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// Flight Blender URL
    #[arg(long, default_value = "http://localhost:8000")]
    url: String,

    /// Session UUID
    #[arg(long, default_value = "00000000-0000-0000-0000-000000000000")]
    session: String,

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

fn main() {
    let args = Args::parse();

    println!("Generating authentication token...");
    let token = generate_dummy_token(None);

    let client = BlenderClient::new(&args.url, &args.session, &token);

    // Create scenario
    let scenario = match args.scenario {
        ScenarioType::Crossing => create_crossing_scenario(args.lat, args.lon),
        ScenarioType::Parallel => create_parallel_scenario(args.lat, args.lon),
        ScenarioType::Converging => create_converging_scenario(args.lat, args.lon),
    };

    println!("\nScenario: {}", scenario.name);
    println!("Starting multi-drone simulation");
    println!("  Drones: {}", scenario.drones.len());
    println!("  Duration: {}s, Update rate: {}Hz\n", args.duration, args.rate);

    for (drone_id, _) in &scenario.drones {
        println!("  - {}", drone_id);
    }
    println!();

    let start = Instant::now();
    let mut update_count = 0u32;
    let update_interval = Duration::from_secs_f64(1.0 / args.rate);

    while start.elapsed() < Duration::from_secs(args.duration) {
        let elapsed = start.elapsed().as_secs_f64();
        update_count += 1;

        // Update all drones
        for (drone_id, path) in &scenario.drones {
            let (lat, lon, alt) = path.get_position(elapsed);
            let heading = path.get_heading(elapsed);
            let speed = path.get_speed_mps();

            match client.send_observation(drone_id, lat, lon, alt, heading, speed) {
                Ok(status) => {
                    println!(
                        "[{:3}] {}: ({:.6}, {:.6}) -> {}",
                        update_count, drone_id, lat, lon, status
                    );
                }
                Err(e) => {
                    eprintln!("[{}] Error: {}", drone_id, e);
                }
            }
        }

        thread::sleep(update_interval);
    }

    let total_updates = update_count as usize * scenario.drones.len();
    println!("\nSimulation complete. Sent {} total updates.", total_updates);
}
