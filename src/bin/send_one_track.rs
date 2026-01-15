//! CLI tool to send simulated drone telemetry to Flight Blender.
//!
//! Simulates a single drone flying in a circle.

use atc_drone::auth::generate_dummy_token;
use atc_drone::sim::{BlenderClient, CircularPath, FlightPath};
use clap::Parser;
use std::thread;
use std::time::{Duration, Instant};

/// Send drone telemetry to Flight Blender (single drone, circular path)
#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// Flight Blender URL
    #[arg(long, default_value = "http://localhost:8000")]
    url: String,

    /// Session UUID
    #[arg(long, default_value = "00000000-0000-0000-0000-000000000001")]
    session: String,

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

fn main() {
    let args = Args::parse();

    println!("Generating authentication token...");
    let token = generate_dummy_token(None);

    let client = BlenderClient::new(&args.url, &args.session, token);

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

    let start = Instant::now();
    let mut update_count = 0u32;
    let update_interval = Duration::from_secs_f64(1.0 / args.rate);

    while start.elapsed() < Duration::from_secs(args.duration) {
        let elapsed = start.elapsed().as_secs_f64();
        let (lat, lon, alt) = path.get_position(elapsed);
        let heading = path.get_heading(elapsed);
        let speed = path.get_speed_mps();

        match client.send_observation(&args.icao, lat, lon, alt, heading, speed) {
            Ok(status) => {
                update_count += 1;
                println!(
                    "[{:3}] Sent position ({:.6}, {:.6}) -> Status: {}",
                    update_count, lat, lon, status
                );
            }
            Err(e) => {
                eprintln!("Error sending observation: {}", e);
            }
        }

        thread::sleep(update_interval);
    }

    println!("\nSimulation complete. Sent {} position updates.", update_count);
}
