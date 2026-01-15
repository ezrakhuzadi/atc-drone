//! CLI tool for multi-drone simulation with conflict scenarios.
//! 
//! Uses AtcClient SDK to send telemetry through atc-server (not directly to Blender).

use atc_cli::sim::{
    create_converging_scenario, create_crossing_scenario, create_parallel_scenario, FlightPath,
};
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
    
    for (drone_id, _) in &scenario.drones {
        let mut client = AtcClient::new(&args.url);
        match client.register(Some(drone_id)).await {
            Ok(resp) => {
                println!("  Registered: {}", resp.drone_id);
                clients.insert(drone_id.clone(), client);
            }
            Err(e) => {
                eprintln!("  Failed to register {}: {}", drone_id, e);
            }
        }
    }

    if clients.is_empty() {
        anyhow::bail!("No drones registered successfully");
    }

    println!("\nStarting multi-drone simulation");
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
                let (lat, lon, alt) = path.get_position(elapsed);
                let heading = path.get_heading(elapsed);
                let speed = path.get_speed_mps();

                match client.send_position(lat, lon, alt, heading, speed).await {
                    Ok(_) => {
                        println!(
                            "[{:3}] {}: ({:.6}, {:.6}) -> OK",
                            update_count, drone_id, lat, lon
                        );
                    }
                    Err(e) => {
                        eprintln!("[{}] Error: {}", drone_id, e);
                    }
                }
            }
        }
    }

    let total_updates = update_count as usize * clients.len();
    println!("\nSimulation complete. Sent {} total updates.", total_updates);
    Ok(())
}
