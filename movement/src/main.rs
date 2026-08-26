use std::net::UdpSocket;
use std::thread::sleep;
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let socket = UdpSocket::bind("127.0.0.1:0")?;
    let sim_target = "127.0.0.1:5555";

    println!("[Movement] Tracked controller online.");
    println!("[Movement] UDP actuator sender: 18 x f64 = 144 bytes");
    println!("[Movement] Sending commands to 127.0.0.1:5555 at 100 Hz.");
    println!("[Movement] Keyboard controls:");
    println!("W = Forward");
    println!("S = Backward");
    println!("A = Left");
    println!("D = Right");
    println!("Space = Stop");
    println!("Q = Quit\n");

    enable_raw_mode()?;

    let track_width = 0.36;
    let mut linear_v: f64 = 0.0;
    let mut angular_w: f64 = 0.0;

    loop {
        if event::poll(Duration::from_millis(1))? {
            if let Event::Key(key_event) = event::read()? {
                if key_event.kind != KeyEventKind::Press {
                    continue;
                }

                match key_event.code {
                    KeyCode::Char('w') | KeyCode::Char('W') => {
                        linear_v = 2.0;
                        angular_w = 0.0;
                        println!("Forward");
                    }

                    KeyCode::Char('s') | KeyCode::Char('S') => {
                        linear_v = -2.0;
                        angular_w = 0.0;
                        println!("Backward");
                    }

                    KeyCode::Char('a') | KeyCode::Char('A') => {
                        linear_v = 0.0;
                        angular_w = -2.5;
                        println!("Left");
                    }

                    KeyCode::Char('d') | KeyCode::Char('D') => {
                        linear_v = 0.0;
                        angular_w = 2.5;
                        println!("Right");
                    }

                    KeyCode::Char(' ') => {
                        linear_v = 0.0;
                        angular_w = 0.0;
                        println!("Stop");
                    }

                    KeyCode::Char('q') | KeyCode::Char('Q') => {
                        break;
                    }

                    _ => {}
                }
            }
        }

        let v_left = linear_v - (angular_w * track_width / 2.0);
        let v_right = linear_v + (angular_w * track_width / 2.0);

        // 18 actuator values, each an f64 (8 bytes).
        // 18 × 8 = 144 bytes.
        let actuator_values: [f64; 18] = [
            v_left, v_right,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
        ];

        let mut buffer = Vec::with_capacity(144);

        for value in actuator_values {
            buffer.extend_from_slice(&value.to_le_bytes());
        }

        debug_assert_eq!(buffer.len(), 144);

        socket.send_to(&buffer, sim_target)?;

        // 100 Hz = one transmission every 10 milliseconds.
        sleep(Duration::from_millis(10));
    }

    disable_raw_mode()?;

    Ok(())
}