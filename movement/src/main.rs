use std::io::{self, Write};
use std::net::UdpSocket;
use std::thread::sleep;
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

mod ik;

fn get_user_input(prompt: &str) -> f64 {
    print!("{}", prompt);
    io::stdout().flush().unwrap();

    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .expect("Failed to read line");

    input
        .trim()
        .parse::<f64>()
        .expect("Please enter a valid number!")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Ariadne Tracked Robot Controller Setup ===");
    let track_width = get_user_input("Enter Track Width W (in meters, e.g., 0.36): ");

    let socket = UdpSocket::bind("127.0.0.1:0")?;
    let sim_target = "127.0.0.1:5555";

    println!("\n[Movement] Tracked controller online.");
    println!("[Movement] Track width set to: {} m", track_width);
    println!("[Movement] UDP actuator sender: 18 x f64 = 144 bytes");
    println!("[Movement] Sending commands to 127.0.0.1:5555 at 100 Hz.");
    println!("[Movement] Keyboard controls:");
    println!("W = Forward | S = Backward | A = Left | D = Right | + / - = Speed Up/Down | Space = Stop | Q = Quit\n");

    enable_raw_mode()?;

    let mut base_speed: f64 = 2.0;
    let mut linear_v: f64 = 0.0;
    let mut angular_w: f64 = 0.0;
    let mut running = true;

    while running {
        if event::poll(Duration::from_millis(1))? {
            if let Event::Key(key_event) = event::read()? {
                if key_event.kind == KeyEventKind::Press {
                   match key_event.code {
    // Increase speed
    KeyCode::Char('+') | KeyCode::Char('=') | KeyCode::Up => {
        base_speed = (base_speed + 0.5).min(10.0);
        // Automatically update active motion if moving
        if linear_v > 0.0 {
            linear_v = base_speed;
        } else if linear_v < 0.0 {
            linear_v = -base_speed;
        }
        println!("\r[Speed] Target speed set to: {:.1} m/s", base_speed);
    }
    // Decrease speed
    KeyCode::Char('-') | KeyCode::Char('_') | KeyCode::Down => {
        base_speed = (base_speed - 0.5).max(0.5);
        // Automatically update active motion if moving
        if linear_v > 0.0 {
            linear_v = base_speed;
        } else if linear_v < 0.0 {
            linear_v = -base_speed;
        }
        println!("\r[Speed] Target speed set to: {:.1} m/s", base_speed);
    }

    // Movement controls
    KeyCode::Char('w') | KeyCode::Char('W') => {
        linear_v = base_speed;
        angular_w = 0.0;
    }
    KeyCode::Char('s') | KeyCode::Char('S') => {
        linear_v = -base_speed;
        angular_w = 0.0;
    }
    KeyCode::Char('a') | KeyCode::Char('A') => {
        linear_v = 0.0;
        angular_w = -2.5;
    }
    KeyCode::Char('d') | KeyCode::Char('D') => {
        linear_v = 0.0;
        angular_w = 2.5;
    }
    KeyCode::Char(' ') => {
        linear_v = 0.0;
        angular_w = 0.0;
    }
    KeyCode::Char('q') | KeyCode::Char('Q') => {
        running = false;
    }
    _ => {}
}

                    if running {
                        let track_vels = ik::calculate_track_velocities(linear_v, angular_w, track_width);
                        println!(
                            "\r[Cmd] v = {:.2} m/s, w = {:.2} rad/s -> Left Track: {:.2} m/s | Right Track: {:.2} m/s",
                            linear_v, angular_w, track_vels.v_left, track_vels.v_right
                        );
                    }
                }
            }
        }

        let track_vels = ik::calculate_track_velocities(linear_v, angular_w, track_width);

        let actuator_values: [f64; 18] = [
            track_vels.v_left,
            track_vels.v_right,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
        ];

        let mut buffer = Vec::with_capacity(144);
        for value in actuator_values {
            buffer.extend_from_slice(&value.to_le_bytes());
        }

        socket.send_to(&buffer, sim_target)?;
        sleep(Duration::from_millis(10));
    }

    disable_raw_mode()?;
    println!("\r\nController shut down safely.");
    Ok(())
}