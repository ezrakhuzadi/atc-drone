//! CLI tool to send simulated drone telemetry to ATC Server.
//!
//! Simulates a single drone flying in a circle.

use atc_cli::sim::{CircularPath, FlightPath};
use atc_sdk::AtcClient;
use clap::Parser;
use std::time::Duration;
use tokio::time;

/// Send drone telemetry to ATC Server (single drone, circular path)
#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// ATC Server URL
    #[arg(long, default_value = "http://localhost:3000")]
    url: String,

    /// Drone ICAO address/identifier
    #[arg(long, default_value = "DRONE001")]
    icao: String,

    /// Center latitude (default: UCI)
    #[arg(long, default_value_t = 33.6846)]
    lat: f64,

    /// Center longitude (default: UCI)
    #[arg(long, default_value_t = -117.8265)]
    lon: f64,

    /// Circle radius in meters
    #[arg(long, default_value_t = 200.0)]
    radius: f64,

    /// Altitude in meters
    #[arg(long, default_value_t = 50.0)]
    altitude: f64,

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

    println!("Connecting to ATC Server at {}...", args.url);
    let mut client = AtcClient::new(&args.url);

    // Register drone
    match client.register(Some(&args.icao)).await {
        Ok(resp) => println!("Registered drone: {}", resp.drone_id),
        Err(e) => {
            eprintln!("Failed to register: {}", e);
            // If registration fails, we can't really proceed with SDK as it expects drone_id
            return Err(e);
        }
    }

    let path = CircularPath::new(
        args.lat,
        args.lon,
        args.radius,
        args.altitude,
        10.0, // speed_mps
        0.0,  // start_angle
        false, // counterclockwise
    );

    println!("Starting circular flight simulation for {}", args.icao);
    println!("  Center: ({}, {})", args.lat, args.lon);
    println!("  Radius: {}m, Altitude: {}m", args.radius, args.altitude);
    println!("  Duration: {}s, Update rate: {}Hz", args.duration, args.rate);
    println!();

    let start = time::Instant::now();
    let mut update_count = 0u32;
    let mut interval = time::interval(Duration::from_secs_f64(1.0 / args.rate));

    loop {
        interval.tick().await; // Wait for next tick

        let elapsed = start.elapsed().as_secs_f64();
        if elapsed > args.duration as f64 {
            break;
        }

        let (lat, lon, alt) = path.get_position(elapsed);
        let heading = path.get_heading(elapsed);
        let speed = path.get_speed_mps();

        match client.send_position(lat, lon, alt, heading, speed).await {
            Ok(_) => {
                update_count += 1;
                println!(
                    "[{:3}] Sent position ({:.6}, {:.6}) -> OK",
                    update_count, lat, lon
                );
            }
            Err(e) => {
                eprintln!("Error sending telemetry: {}", e);
            }
        }
    }

    println!("\nSimulation complete. Sent {} position updates.", update_count);
    Ok(())
}
