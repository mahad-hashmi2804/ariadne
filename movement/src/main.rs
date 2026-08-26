use std::net::UdpSocket;
use std::thread::sleep;
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

use serde_json;
use std::io::{self, Write};

mod nav;

use nav::{MovementCommand, MovementState, ObjectDetection, MovementManager, Position, Robot};
fn read_number(prompt: &str) -> f64 {
    loop {
        print!("{}", prompt);

        io::stdout()
            .flush()
            .expect("Failed to flush stdout");

        let mut input = String::new();

        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read input");

        match input.trim().parse::<f64>() {
            Ok(value) => return value,

            Err(_) => {
                println!("Please enter a valid number.");
            }
        }
    }
}
fn read_position(name: &str) -> Position {
    println!();
    println!("Enter {} position:", name);

    let x = read_number("x = ");
    let y = read_number("y = ");

    Position::new(x, y)
}
fn read_detection() -> ObjectDetection {
    println!();
    println!("Enter object detection JSON.");

    println!(
        r#"Example:
{{"object_position":2.0,"object_angle":10.0,"found":true}}"#
    );

    print!("JSON: ");

    io::stdout()
        .flush()
        .expect("Failed to flush stdout");

    let mut input = String::new();

    io::stdin()
        .read_line(&mut input)
        .expect("Failed to read JSON");

    match serde_json::from_str::<ObjectDetection>(input.trim()) {
        Ok(data) => data,

        Err(error) => {
            println!("Invalid JSON: {}", error);

            ObjectDetection {
                object_position: 999.0,
                object_angle: 0.0,
                found: false,
            }
        }
    }
}

// ---------------------------------------------------------
// PRINT COMMAND AS JSON
// ---------------------------------------------------------

fn print_command(command: &MovementCommand) {
    match serde_json::to_string_pretty(command) {
        Ok(json) => {
            println!();
            println!("========== MOVEMENT OUTPUT ==========");
            println!("{}", json);
            println!("=====================================");
        }

        Err(error) => {
            println!("Could not serialize command: {}", error);
        }
    }
}
fn main() {
    // =================================================
    // UDP
    // =================================================

    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();

    let sim_target = "127.0.0.1:5555";

   println!("========================================");
    println!("       ARIADNE MOVEMENT MANAGER");
    println!("========================================");

    // ---------------------------------------------
    // INPUT
    // ---------------------------------------------

    let initial_position = read_position("INITIAL");

    let final_position = read_position("FINAL");

    let initial_heading =
        read_number("Initial robot heading in degrees = ");

    // ---------------------------------------------
    // CREATE ROBOT
    // ---------------------------------------------

    let mut robot =
        Robot::new(initial_position, initial_heading);

    // ---------------------------------------------
    // CREATE MOVEMENT MANAGER
    // ---------------------------------------------

    let mut manager =
        MovementManager::new(final_position);

    println!();
    println!("Initial robot:");
    println!(
        "Position: ({:.2}, {:.2})",
        robot.position.x,
        robot.position.y,
    );

    println!("Heading: {:.2}°", robot.heading);

    println!(
        "Target: ({:.2}, {:.2})",
        final_position.x,
        final_position.y
    );

    // ---------------------------------------------
    // SIMULATION
    // ---------------------------------------------

    println!();
    println!("Starting simulation...");
    println!("Press ENTER after every detection update.");
    println!();

    let dt = 0.1;

    for step in 0..1000 {
        println!();
        println!("--------------- STEP {} ---------------", step);

        // -----------------------------------------
        // Detection department input
        // -----------------------------------------

        let detection_json = r#"
    {
        "object_position": 2.0,
        "object_angle": 0.0,
        "found": true
    }
    "#;

        let detection: ObjectDetection =
            serde_json::from_str(detection_json)
                .expect("Invalid JSON");

        // -----------------------------------------
        // Movement decision
        // -----------------------------------------

        let command =
            manager.update(&mut robot, &detection);

        // -----------------------------------------
        // Output
        // -----------------------------------------

        print_command(&command);

        println!(
            "Robot position: ({:.2}, {:.2})",
            robot.position.x,
            robot.position.y
        );

        println!(
            "Robot heading: {:.2}°",
            robot.heading
        );

        println!(
            "Distance to target: {:.2} m",
            robot.position.distance_to(&final_position)
        );

        // -----------------------------------------
        // Target reached
        // -----------------------------------------

        if matches!(
            command.state,
            MovementState::Reached
        ) {
            println!();
            println!("========================================");
            println!("        TARGET REACHED");
            println!("========================================");

            break;
        }

        // -----------------------------------------
        // Simulate wheel movement
        // -----------------------------------------

        robot.simulate_motion(
            command.left_velocity,
            command.right_velocity,
            dt,
        );
    }
}

    
