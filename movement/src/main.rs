use std::net::UdpSocket;
use std::thread::sleep;
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
mod mov;

use mov::{
    decide_movement,
    Obstacle,
    Position,
    Pose,
};

fn main()  {
    let socket = UdpSocket::bind("127.0.0.1:0")?;
    let sim_target = "127.0.0.1:5555";
    
    // These are the dummy values
    // Current / initial robot position
    let current_pose = Pose {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    };

    // Final / target position
    let target = Position {
        x: 5.0,
        y: 3.0,
    };

    // =================================================
    // OBJECT DETECTION INPUT
    // =================================================

    let obstacle = Obstacle {
        ob_found: true,
        ob_distance: 0.5,
        ob_angle: 30.0,
    };

    loop {
         let (state, command) =
        decide_movement(
            current_pose,
            target,
            obstacle,
        );

    // =================================================
    // OUTPUT
    // =================================================

    println!("================================");
    println!("       MOVEMENT OUTPUT");
    println!("================================");

    println!("Movement State: {:?}", state);

    println!(
        "Linear Velocity: {:.2} m/s",
        command.linear_velocity
    );

    println!(
        "Angular Velocity: {:.2} rad/s",
        command.angular_velocity
    );

    println!("================================");
    
}
      
                }
            
}
