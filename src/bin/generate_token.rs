//! CLI tool to generate a dummy JWT token for Flight Blender API access.

use atc_drone::auth::{generate_dummy_token, TokenConfig};
use clap::Parser;

/// Generate a dummy JWT token for Flight Blender API access
#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// Space-separated scopes
    #[arg(long, default_value = "flightblender.read flightblender.write")]
    scopes: String,

    /// Token audience (must match PASSPORT_AUDIENCE in Blender)
    #[arg(long, default_value = "testflight.flightblender.com")]
    audience: String,

    /// Token expiry in hours
    #[arg(long, default_value_t = 24)]
    expiry: u64,
}

fn main() {
    let args = Args::parse();

    let config = TokenConfig {
        scopes: args.scopes,
        audience: args.audience,
        expiry_hours: args.expiry,
    };

    let token = generate_dummy_token(Some(config));
    println!("{}", token);
}
