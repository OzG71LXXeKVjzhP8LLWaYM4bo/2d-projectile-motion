use std::error::Error;
use std::f64::consts::PI;

fn read_env_var_f64(key: &str, default: f64) -> f64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(default)
}

fn main() -> Result<(), Box<dyn Error>> {
    dotenvy::dotenv().ok();

    // parameters
    let v0 = read_env_var_f64("V0", 50.0);                // initial speed (m/s)
    let angle_deg = read_env_var_f64("ANGLE_DEG", 45.0);  // launch angle
    let y0 = read_env_var_f64("Y0", 0.0);                 // initial height
    let g = read_env_var_f64("G", 9.81);                  // gravity
    let m = read_env_var_f64("M", 1.0);                   // mass
    let c = read_env_var_f64("C", 0.05);                  // drag coefficient (0 = vacuum)
    let dt = read_env_var_f64("DT", 0.01);                // timestep

    // initial state
    let theta = angle_deg * PI / 180.0; // radians
    let mut x = 0.0;
    let mut y = y0;
    let mut vx = v0 * theta.cos();
    let mut vy = v0 * theta.sin();
    let mut t = 0.0;

    // create csv writer
    let mut wtr = csv::Writer::from_path("results/trajectory.csv")?;
    wtr.write_record(&["time (s)", "pos-x (m)", "pos-y (m)"])?;

    // write first row
    wtr.write_record(&[t.to_string(), x.to_string(), y.to_string()])?;

    // simulate until projectile lands
    while y >= 0.0 {
        let speed = (vx * vx + vy * vy).sqrt(); // speed = sqrt(vx^2 + vy^2)
        let ax = -(c / m) * speed * vx; // x acceleration   
        let ay = -g - (c / m) * speed * vy; // y acceleration

        vx += ax * dt; // adding acceleration to velocity
        vy += ay * dt; // adding acceleration to velocity
        x  += vx * dt; // adding velocity to position
        y  += vy * dt; // adding velocity to position
        t  += dt; // adding timestep
        if y > 0.0 {
            wtr.write_record(&[t.to_string(), x.to_string(), y.to_string()])?;
        } else {
            break;
        }
    }

    wtr.flush()?; // make sure file is saved
    println!("Trajectory written to trajectory.csv");

    Ok(())
}
