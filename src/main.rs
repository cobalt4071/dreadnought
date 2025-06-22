use macroquad::prelude::*;
use nalgebra::{Vector2, Point2};
use std::collections::VecDeque;

//=============================================================================
// II. Essential Defining Values (Structs & Constants)
//=============================================================================

const METERS_PER_NAUTICAL_MILE: f32 = 1852.0;
const KNOTS_TO_MPS: f32 = METERS_PER_NAUTICAL_MILE / 3600.0; // Meters per second

//--- Simulation/Environment Parameters ---
struct Simulation {
    current_sim_time_s: f32,
    time_step_s: f32,
    time_bearing_graph_history_s: f32,
}

//--- Bearing History ---
// A record of a target's relative bearing at a specific time.
#[derive(Clone, Copy)]
struct BearingRecord {
    timestamp_s: f32,
    relative_bearing_deg: f32,
}

//--- Own Ship ---
struct OwnShip {
    id: u32,
    position_m: Point2<f32>,
    speed_knots: f32,
    heading_deg: f32,
    // A history of bearings for each target, identified by the target's ID.
    // Here we use a VecDeque to efficiently manage the history window.
    recorded_bearings: Vec<(u32, VecDeque<BearingRecord>)>,
}

//--- Target ---
struct Target {
    id: u32,
    position_m: Point2<f32>,
    speed_knots: f32,
    heading_deg: f32,
    track_history: VecDeque<Point2<f32>>,
}

//=============================================================================
// III. High-Level Implementation Steps (Functions)
//=============================================================================

/// Step 2: Implement update methods for objects.
/// Updates the position of any object with position, speed, and heading.
fn update_position(pos: &mut Point2<f32>, speed_knots: f32, heading_deg: f32, dt: f32) {
    let speed_mps = speed_knots * KNOTS_TO_MPS;
    let heading_rad = heading_deg.to_radians();
    let velocity = Vector2::new(heading_rad.sin(), heading_rad.cos()) * speed_mps;
    *pos += velocity * dt;
}

/// Step 3: Core Simulation Logic - Bearing Calculation.
/// Normalizes an angle to the 0-360 degree range.
fn normalize_degrees(mut angle: f32) -> f32 {
    while angle < 0.0 {
        angle += 360.0;
    }
    while angle >= 360.0 {
        angle -= 360.0;
    }
    angle
}

/// Step 3: Core Simulation Logic - Bearing Calculation.
/// Calculates the true bearing from the observer to the target.
fn calculate_true_bearing_deg(observer_pos: &Point2<f32>, target_pos: &Point2<f32>) -> f32 {
    let delta = target_pos - observer_pos;
    // atan2(dx, dy) gives angle from +Y axis (North).
    let angle_rad = delta.x.atan2(delta.y);
    normalize_degrees(angle_rad.to_degrees())
}

/// Step 5: User Interaction.
/// Handles keyboard input to control the Own Ship.
fn handle_input(own_ship: &mut OwnShip) {
    let turn_rate_deg_sec = 2.0;
    let speed_change_knots = 5.0;
    let dt = get_frame_time();

    if is_key_down(KeyCode::Left) {
        own_ship.heading_deg -= turn_rate_deg_sec * dt * 30.0; // Make turning faster
    }
    if is_key_down(KeyCode::Right) {
        own_ship.heading_deg += turn_rate_deg_sec * dt * 30.0;
    }
    own_ship.heading_deg = normalize_degrees(own_ship.heading_deg);

    if is_key_pressed(KeyCode::Up) {
        own_ship.speed_knots += speed_change_knots;
    }
    if is_key_pressed(KeyCode::Down) {
        own_ship.speed_knots = (own_ship.speed_knots - speed_change_knots).max(0.0);
    }
}

/// Step 4: Visual Display - 2D Map View.
/// Handles drawing the top-down tactical map.
fn draw_map_view(own_ship: &OwnShip, targets: &Vec<Target>, screen_w: f32) {
    // World-to-screen conversion
    let world_scale = 0.05; // 1 meter in world = 0.05 pixels on screen
    let center_screen = Vec2::new(screen_w / 2.0, screen_height() / 2.0);

    // Draw Own Ship
    let os_screen_pos = center_screen; // Always draw Own Ship at the center
    draw_triangle(
        os_screen_pos + Vec2::new(0.0, -15.0).rotate(Vec2::from_angle(own_ship.heading_deg.to_radians())),
        os_screen_pos + Vec2::new(-10.0, 10.0).rotate(Vec2::from_angle(own_ship.heading_deg.to_radians())),
        os_screen_pos + Vec2::new(10.0, 10.0).rotate(Vec2::from_angle(own_ship.heading_deg.to_radians())),
        GREEN,
    );

    // Draw Targets and their tracks
    for target in targets {
        let relative_pos = target.position_m - own_ship.position_m;
        // CORRECTED LINE: Use .x and .y instead of .coords
        let target_screen_pos = center_screen + vec2(relative_pos.x, relative_pos.y) * world_scale;

        // Draw Target Track
        if target.track_history.len() > 1 {
            let screen_points: Vec<Vec2> = target.track_history.iter()
                // CORRECTED LINE: Use .x and .y instead of .coords
                .map(|p| {
                    let relative_vec = p - own_ship.position_m;
                    center_screen + vec2(relative_vec.x, relative_vec.y) * world_scale
                })
                .collect();
            for i in 0..screen_points.len() - 1 {
                draw_line(screen_points[i].x, screen_points[i].y, screen_points[i+1].x, screen_points[i+1].y, 1.0, RED);
            }
        }
        // Draw Target Icon
        draw_circle(target_screen_pos.x, target_screen_pos.y, 8.0, RED);
    }
}

/// Step 4: Visual Display - Bearing Rate Graph.
/// Handles drawing the specialized time-bearing plot.
fn draw_bearing_rate_graph(
    own_ship: &OwnShip,
    sim: &Simulation,
    graph_rect: Rect,
) {
    // Draw graph background and axes
    draw_rectangle(graph_rect.x, graph_rect.y, graph_rect.w, graph_rect.h, Color::new(0.0, 0.0, 0.1, 1.0));
    draw_rectangle_lines(graph_rect.x, graph_rect.y, graph_rect.w, graph_rect.h, 2.0, WHITE);

    // Y-Axis (Bearing)
    draw_text("360", graph_rect.x + 5.0, graph_rect.y + 20.0, 20.0, LIGHTGRAY);
    draw_text("180", graph_rect.x + 5.0, graph_rect.y + graph_rect.h / 2.0, 20.0, LIGHTGRAY);
    draw_text("000", graph_rect.x + 5.0, graph_rect.y + graph_rect.h - 5.0, 20.0, LIGHTGRAY);

    // X-Axis (Time)
    draw_text("Now", graph_rect.x + graph_rect.w - 40.0, graph_rect.y + graph_rect.h + 20.0, 20.0, LIGHTGRAY);
    let time_label = format!("-{}s", sim.time_bearing_graph_history_s as i32);
    draw_text(&time_label, graph_rect.x, graph_rect.y + graph_rect.h + 20.0, 20.0, LIGHTGRAY);
    
    // Draw a vertical line for the current time
    let now_x = graph_rect.x + graph_rect.w;
    draw_line(now_x, graph_rect.y, now_x, graph_rect.y + graph_rect.h, 2.0, YELLOW);


    // For each target, plot its bearing history
    // CORRECTED LINE: Prefixed target_id with _ to silence the unused variable warning
    for (_target_id, bearings) in &own_ship.recorded_bearings {
        if bearings.len() < 2 { continue; }

        let mut points_to_draw: Vec<(f32, f32)> = Vec::new();

        // Convert (time, bearing) data to screen coordinates
        for i in 0..bearings.len() {
            let record = bearings[i];
            let time_ago = sim.current_sim_time_s - record.timestamp_s;
            
            // Only draw points within the graph's time window
            if time_ago <= sim.time_bearing_graph_history_s {
                let x = graph_rect.x + graph_rect.w * (1.0 - time_ago / sim.time_bearing_graph_history_s);
                let y = graph_rect.y + graph_rect.h * (1.0 - record.relative_bearing_deg / 360.0);
                points_to_draw.push((x, y));
            }
        }
        
        // --- Crucial Step: Handle 0/360 Degree Wrap-Around ---
        if points_to_draw.len() > 1 {
            for i in 0..points_to_draw.len() - 1 {
                let p1 = points_to_draw[i];
                let p2 = points_to_draw[i+1];
                let record1_bearing = bearings[i].relative_bearing_deg;
                let record2_bearing = bearings[i+1].relative_bearing_deg;

                let bearing_diff = (record1_bearing - record2_bearing).abs();

                // If the bearing difference is large (e.g., > 300), it's likely a wrap-around
                if bearing_diff > 300.0 {
                    // We need to draw two segments to show the wrap
                    let (mut p1_wrap, mut p2_wrap) = (p1, p2);
                    if record1_bearing > record2_bearing { // e.g., from 358 to 2 degrees
                         p1_wrap.1 = graph_rect.y + graph_rect.h; // Draw from p1 down to 360
                         p2_wrap.1 = graph_rect.y; // Draw from p2 up to 0
                    } else { // e.g., from 2 to 358 degrees
                         p1_wrap.1 = graph_rect.y; // Draw from p1 up to 0
                         p2_wrap.1 = graph_rect.y + graph_rect.h; // Draw from p2 down to 360
                    }
                    draw_line(p1.0, p1.1, p2_wrap.0, p1_wrap.1, 2.0, ORANGE);
                    draw_line(p2.0, p2.1, p1_wrap.0, p2_wrap.1, 2.0, ORANGE);

                } else {
                    // Normal segment, no wrap-around
                    draw_line(p1.0, p1.1, p2.0, p2.1, 2.0, ORANGE);
                }
            }
        }
    }
}

/// Step 6: Refinements - Display on-screen text.
fn draw_info_text(own_ship: &OwnShip, true_bearing: f32, relative_bearing: f32) {
    let text_y = 30.0;
    let line_height = 25.0;
    let font_size = 24.0;
    
    draw_text(
        &format!("Own Ship Heading: {:.1}°", own_ship.heading_deg),
        20.0, text_y, font_size, WHITE
    );
    draw_text(
        &format!("Own Ship Speed:   {:.1} kts", own_ship.speed_knots),
        20.0, text_y + line_height, font_size, WHITE
    );
    draw_text(
        &format!("Target True Bearing: {:.1}°", true_bearing),
        20.0, text_y + line_height * 2.0, font_size, WHITE
    );
    draw_text(
        &format!("Target Rel. Bearing: {:.1}°", relative_bearing),
        20.0, text_y + line_height * 3.0, font_size, GREEN
    );
    draw_text(
        "Controls: Arrow Keys (Up/Down for Speed, Left/Right for Heading)",
        20.0, screen_height() - 30.0, 20.0, LIGHTGRAY,
    );
}


#[macroquad::main("2D Submarine Bearing Rate Graph")]
async fn main() {
    //=========================================================================
    // 1. Environment Setup & Initialization
    //=========================================================================
    let mut sim = Simulation {
        current_sim_time_s: 0.0,
        time_step_s: 1.0, // Update simulation logic once per second
        time_bearing_graph_history_s: 300.0, // Show last 5 minutes on graph
    };

    let mut own_ship = OwnShip {
        id: 0,
        position_m: Point2::new(5000.0, 5000.0),
        speed_knots: 5.0,
        heading_deg: 45.0,
        recorded_bearings: vec![(1, VecDeque::new())], // Initialize for target ID 1
    };

    let mut targets = vec![
        Target {
            id: 1,
            position_m: Point2::new(10000.0, 2000.0),
            speed_knots: 10.0,
            heading_deg: 225.0,
            track_history: VecDeque::new(),
        }
    ];

    let mut time_accumulator: f32 = 0.0;
    let mut last_true_bearing = 0.0;
    let mut last_relative_bearing = 0.0;


    //=========================================================================
    // Main Game Loop
    //=========================================================================
    loop {
        clear_background(BLACK);

        let dt = get_frame_time();
        time_accumulator += dt;

        // --- Handle User Input ---
        handle_input(&mut own_ship);

        // --- Simulation Logic Update (runs at a fixed time step) ---
        if time_accumulator >= sim.time_step_s {
            time_accumulator -= sim.time_step_s;
            sim.current_sim_time_s += sim.time_step_s;

            // Step 3: Update positions
            update_position(
                &mut own_ship.position_m,
                own_ship.speed_knots,
                own_ship.heading_deg,
                sim.time_step_s,
            );

            for target in &mut targets {
                update_position(
                    &mut target.position_m,
                    target.speed_knots,
                    target.heading_deg,
                    sim.time_step_s,
                );
                // Record target's position for its track history
                target.track_history.push_back(target.position_m);
                if target.track_history.len() > 200 {
                    target.track_history.pop_front();
                }
            }

            // Step 3: Calculate and record bearings for each target
            for target in &targets {
                let true_bearing = calculate_true_bearing_deg(
                    &own_ship.position_m, &target.position_m
                );
                
                let relative_bearing = normalize_degrees(true_bearing - own_ship.heading_deg);
                
                last_true_bearing = true_bearing;
                last_relative_bearing = relative_bearing;

                // Find the bearing history for this target and add the new record
                if let Some((_, history)) = own_ship.recorded_bearings.iter_mut().find(|(id, _)| *id == target.id) {
                    history.push_back(BearingRecord {
                        timestamp_s: sim.current_sim_time_s,
                        relative_bearing_deg: relative_bearing,
                    });
                    // Prune old records
                    while let Some(front) = history.front() {
                        if sim.current_sim_time_s - front.timestamp_s > sim.time_bearing_graph_history_s {
                            history.pop_front();
                        } else {
                            break;
                        }
                    }
                }
            }
        }
        
        // --- Visual Display (runs every frame) ---
        let screen_w = screen_width();
        let screen_h = screen_height();
        
        // Define areas for the two views
        let map_view_width = screen_w * 0.5;
        let graph_rect = Rect::new(map_view_width, 0.0, screen_w - map_view_width, screen_h * 0.8);
        
        // Step 4: Draw the views
        draw_map_view(&own_ship, &targets, map_view_width);
        draw_bearing_rate_graph(&own_ship, &sim, graph_rect);
        draw_info_text(&own_ship, last_true_bearing, last_relative_bearing);

        next_frame().await
    }
}
