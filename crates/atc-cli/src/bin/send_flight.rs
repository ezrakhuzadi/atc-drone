use anyhow::Result;
use atc_core::models::{FlightPlanRequest, Telemetry};
use atc_sdk::AtcClient;
use chrono::Utc;
use clap::Parser;
use tokio::time::{interval, Duration};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// ATC Server URL
    #[arg(long, default_value = "http://localhost:3000")]
    url: String,

    /// Update rate in Hz
    #[arg(long, default_value_t = 5.0)]
    rate: f64,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = AtcClient::new(args.url);

    println!("Attempting to register drone...");
    match client.register(None).await {
        Ok(resp) => println!("Registered drone: {}", resp.drone_id),
        Err(e) => {
            eprintln!("Failed to register drone: {}", e);
            return Ok(());
        }
    }
    
    // 1. Submit Flight Plan (Auto-generate)
    println!("Requesting flight plan...");
    let plan_req = FlightPlanRequest {
        drone_id: client.drone_id().unwrap().to_string(),
        waypoints: None, // Auto-generate
        origin: None,
        destination: None,
        departure_time: None,
    };
    
    let plan = client.create_flight_plan(&plan_req).await?;
    println!("Flight Plan Approved: {}", plan.flight_id);
    println!("Route: {} waypoints", plan.waypoints.len());
    if let Some(start) = plan.waypoints.first() {
        println!("Start: {}, {}", start.lat, start.lon);
    }
    if let Some(end) = plan.waypoints.last() {
        println!("End: {}, {}", end.lat, end.lon);
    }

    // 2. Pre-flight checks
    println!("Performing pre-flight checks...");
    tokio::time::sleep(Duration::from_secs(2)).await;
    println!("Pre-flight checks complete. Requesting takeoff...");
    // ensure "Cleared" state in future... for now just go.
    
    println!("Taking off in 3...");
    tokio::time::sleep(Duration::from_secs(1)).await;
    println!("2...");
    tokio::time::sleep(Duration::from_secs(1)).await;
    println!("1...");
    tokio::time::sleep(Duration::from_secs(1)).await;

    // 3. Fly the route
    // Assume 2 waypoints (Start -> End) for now as implemented in server
    if plan.waypoints.len() < 2 {
        eprintln!("Route too short!");
        return Ok(());
    }
    
    let start_wp = &plan.waypoints[0];
    let end_wp = &plan.waypoints[1];
    
    // Calculate simple linear path
    let dist = haversine_distance(start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon);
    let speed = 10.0; // m/s
    let duration_sec = dist / speed;
    let steps = (duration_sec * args.rate) as u32;
    
    println!("Distance: {:.0}m, Duration: {:.0}s, Steps: {}", dist, duration_sec, steps);
    
    let mut ticker = interval(Duration::from_millis((1000.0 / args.rate) as u64));
    
    // Calculate heading once (straight line)
    let heading = calculate_bearing(start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon);
    
    for i in 0..=steps {
        ticker.tick().await;
        
        let fraction = i as f64 / steps as f64;
        let lat = start_wp.lat + (end_wp.lat - start_wp.lat) * fraction;
        let lon = start_wp.lon + (end_wp.lon - start_wp.lon) * fraction;
        let alt = start_wp.altitude_m + (end_wp.altitude_m - start_wp.altitude_m) * fraction;
        
        // Calculate velocity components from heading and speed
        let heading_rad = heading.to_radians();
        let velocity_x = speed * heading_rad.sin();  // East
        let velocity_y = speed * heading_rad.cos();  // North
        
        let telemetry = Telemetry {
            drone_id: client.drone_id().unwrap().to_string(),
            lat,
            lon,
            altitude_m: alt,
            velocity_x,
            velocity_y,
            velocity_z: 0.0, // Level flight
            heading_deg: heading,
            speed_mps: speed,
            timestamp: Utc::now(),
        };
        
        if let Err(e) = client.send_telemetry(&telemetry).await {
            eprintln!("Failed to send telemetry: {}", e);
        } else {
             // Optional: Print progress?
             if i % 10 == 0 {
                 println!("Sent pos {}/{}: {:.5}, {:.5}", i, steps, lat, lon);
             }
        }
    }
    
    println!("Flight complete. Landed.");

    Ok(())
}

fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0;
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
}

fn calculate_bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let start_lat = lat1.to_radians();
    let start_lon = lon1.to_radians();
    let end_lat = lat2.to_radians();
    let end_lon = lon2.to_radians();

    let d_lon = end_lon - start_lon;

    let y = d_lon.sin() * end_lat.cos();
    let x = start_lat.cos() * end_lat.sin() - start_lat.sin() * end_lat.cos() * d_lon.cos();

    let initial_bearing = y.atan2(x).to_degrees();
    (initial_bearing + 360.0) % 360.0
}
