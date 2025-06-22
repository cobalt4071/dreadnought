use macroquad::prelude::*;
use nalgebra::{Vector2, Point2};
use std::collections::VecDeque;

//=============================================================================
// II. Essential Defining Values (Structs & Constants)
//=============================================================================

const METERS_PER_NAUTICAL_MILE: f32 = 1852.0;
const KNOTS_TO_MPS: f32 = METERS_PER_NAUTICAL_MILE / 3600.0;

struct Simulation {
    current_sim_time_s: f32,
    time_step_s: f32,
    time_bearing_graph_history_s: f32,
}

#[derive(Clone, Copy)]
struct BearingRecord {
    timestamp_s: f32,
    relative_bearing_deg: f32,
}

struct OwnShip {
    id: u32,
    position_m: Point2<f32>,
    speed_knots: f32,
    heading_deg: f32,
    recorded_bearings: Vec<(u32, VecDeque<BearingRecord>)>,
}

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

fn update_position(pos: &mut Point2<f32>, speed_knots: f32, heading_deg: f32, dt: f32) {
    let speed_mps = speed_knots * KNOTS_TO_MPS;
    let heading_rad = heading_deg.to_radians();
    let velocity = Vector2::new(heading_rad.sin(), heading_rad.cos()) * speed_mps;
    *pos += velocity * dt;
}

fn normalize_degrees(mut angle: f32) -> f32 {
    while angle < 0.0 {
        angle += 360.0;
    }
    while angle >= 360.0 {
        angle -= 360.0;
    }
    angle
}

fn calculate_true_bearing_deg(observer_pos: &Point2<f32>, target_pos: &Point2<f32>) -> f32 {
    let delta = target_pos - observer_pos;
    let angle_rad = delta.x.atan2(delta.y);
    normalize_degrees(angle_rad.to_degrees())
}

fn handle_input(own_ship: &mut OwnShip) {
    let turn_rate_deg_sec = 2.0;
    let speed_change_knots = 5.0;
    let dt = get_frame_time();

    if is_key_down(KeyCode::Left) {
        own_ship.heading_deg -= turn_rate_deg_sec * dt * 30.0;
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


//=============================================================================
// IV. Visual Display (Four Panels)
//=============================================================================

/// Panel 1: 2D Map View (Top-Left)
fn draw_map_view(panel: Rect, own_ship: &OwnShip, targets: &Vec<Target>) {
    // Set a camera for this panel to easily manage coordinates
    set_camera(&Camera2D {
        zoom: vec2(1.0 / panel.w, 1.0 / panel.h),
        target: vec2(panel.w/2.0, panel.h/2.0),
        ..Default::default()
    });

    // Draw panel background and border
    draw_rectangle(panel.x, panel.y, panel.w, panel.h, Color::new(0.0, 0.1, 0.1, 1.0));
    draw_rectangle_lines(panel.x, panel.y, panel.w, panel.h, 3.0, WHITE);
    
    let world_scale = 0.03; // Adjusted scale for smaller panel
    let center_panel = vec2(panel.w / 2.0, panel.h / 2.0);

    // Draw Own Ship at the center of the panel
    draw_triangle(
        center_panel + Vec2::new(0.0, -15.0).rotate(Vec2::from_angle(own_ship.heading_deg.to_radians())),
        center_panel + Vec2::new(-10.0, 10.0).rotate(Vec2::from_angle(own_ship.heading_deg.to_radians())),
        center_panel + Vec2::new(10.0, 10.0).rotate(Vec2::from_angle(own_ship.heading_deg.to_radians())),
        GREEN,
    );

    // Draw Targets
    for target in targets {
        let relative_pos = target.position_m - own_ship.position_m;
        let target_screen_pos = center_panel + vec2(relative_pos.x, relative_pos.y) * world_scale;
        
        // Draw Track History
        if target.track_history.len() > 1 {
            for i in 0..target.track_history.len() - 1 {
                let p1 = target.track_history[i];
                let p2 = target.track_history[i+1];
                let v1 = p1 - own_ship.position_m;
                let v2 = p2 - own_ship.position_m;
                draw_line(
                    center_panel.x + v1.x * world_scale, center_panel.y + v1.y * world_scale,
                    center_panel.x + v2.x * world_scale, center_panel.y + v2.y * world_scale,
                    1.0, RED
                );
            }
        }
        draw_circle(target_screen_pos.x, target_screen_pos.y, 8.0, RED);
    }
    set_default_camera(); // Reset camera
}


/// Panel 2: Bearing Rate Graph (Top-Right)
fn draw_bearing_rate_graph(panel: Rect, own_ship: &OwnShip, sim: &Simulation) {
    draw_rectangle(panel.x, panel.y, panel.w, panel.h, Color::new(0.0, 0.0, 0.1, 1.0));
    draw_rectangle_lines(panel.x, panel.y, panel.w, panel.h, 3.0, WHITE);

    let font_size = 20.0;
    let padding = 5.0;

    // Y-Axis (Bearing)
    draw_text("360", panel.x + padding, panel.y + font_size, font_size, LIGHTGRAY);
    draw_text("180", panel.x + padding, panel.y + panel.h / 2.0, font_size, LIGHTGRAY);
    draw_text("000", panel.x + padding, panel.y + panel.h - padding, font_size, LIGHTGRAY);

    // X-Axis (Time)
    draw_text("Now", panel.x + panel.w - 40.0, panel.y + panel.h + 20.0, font_size, LIGHTGRAY);
    let time_label = format!("-{}s", sim.time_bearing_graph_history_s as i32);
    draw_text(&time_label, panel.x, panel.y + panel.h + 20.0, font_size, LIGHTGRAY);
    
    let now_x = panel.x + panel.w;
    draw_line(now_x, panel.y, now_x, panel.y + panel.h, 2.0, YELLOW);

    for (_target_id, bearings) in &own_ship.recorded_bearings {
        if bearings.len() < 2 { continue; }
        for i in 0..bearings.len() - 1 {
            let r1 = bearings[i];
            let r2 = bearings[i+1];
            let time_ago1 = sim.current_sim_time_s - r1.timestamp_s;
            let time_ago2 = sim.current_sim_time_s - r2.timestamp_s;
            
            if time_ago1 > sim.time_bearing_graph_history_s { continue; }

            let x1 = panel.x + panel.w * (1.0 - time_ago1 / sim.time_bearing_graph_history_s);
            let y1 = panel.y + panel.h * (1.0 - r1.relative_bearing_deg / 360.0);
            let x2 = panel.x + panel.w * (1.0 - time_ago2 / sim.time_bearing_graph_history_s);
            let y2 = panel.y + panel.h * (1.0 - r2.relative_bearing_deg / 360.0);

            if (r1.relative_bearing_deg - r2.relative_bearing_deg).abs() > 300.0 {
                // Handle wrap-around drawing
                let (y_start_wrap, y_end_wrap) = if r1.relative_bearing_deg > r2.relative_bearing_deg {
                    (panel.y + panel.h, panel.y)
                } else {
                    (panel.y, panel.y + panel.h)
                };
                draw_line(x1, y1, x2, y_start_wrap, 2.0, ORANGE);
                draw_line(x1, y_end_wrap, x2, y2, 2.0, ORANGE);
            } else {
                draw_line(x1, y1, x2, y2, 2.0, ORANGE);
            }
        }
    }
}


/// Panel 3: Numerical Info (Bottom-Left)
fn draw_info_panel(panel: Rect, own_ship: &OwnShip, true_bearing: f32, relative_bearing: f32) {
    draw_rectangle(panel.x, panel.y, panel.w, panel.h, Color::new(0.1, 0.1, 0.0, 1.0));
    draw_rectangle_lines(panel.x, panel.y, panel.w, panel.h, 3.0, WHITE);

    let font_size = 24.0;
    let start_x = panel.x + 20.0;
    let start_y = panel.y + 40.0;
    let line_height = 30.0;

    draw_text("TELEMETRY", start_x, panel.y + 25.0, 22.0, YELLOW);
    
    draw_text(&format!("Own Ship Heading: {:.1}°", own_ship.heading_deg), start_x, start_y, font_size, WHITE);
    draw_text(&format!("Own Ship Speed:   {:.1} kts", own_ship.speed_knots), start_x, start_y + line_height, font_size, WHITE);
    draw_text(&format!("Target True Bearing: {:.1}°", true_bearing), start_x, start_y + line_height * 2.5, font_size, WHITE);
    draw_text(&format!("Target Rel. Bearing: {:.1}°", relative_bearing), start_x, start_y + line_height * 3.5, font_size, GREEN);
}

/// Panel 4: Controls Help (Bottom-Right)
fn draw_controls_panel(panel: Rect) {
    draw_rectangle(panel.x, panel.y, panel.w, panel.h, Color::new(0.1, 0.0, 0.1, 1.0));
    draw_rectangle_lines(panel.x, panel.y, panel.w, panel.h, 3.0, WHITE);

    let font_size = 24.0;
    let start_x = panel.x + 20.0;
    let start_y = panel.y + 40.0;
    let line_height = 30.0;

    draw_text("CONTROLS", start_x, panel.y + 25.0, 22.0, YELLOW);

    draw_text("Turn Left:", start_x, start_y, font_size, WHITE);
    draw_text("LEFT ARROW", start_x + 200.0, start_y, font_size, LIGHTGRAY);

    draw_text("Turn Right:", start_x, start_y + line_height, font_size, WHITE);
    draw_text("RIGHT ARROW", start_x + 200.0, start_y + line_height, font_size, LIGHTGRAY);

    draw_text("Increase Speed:", start_x, start_y + line_height * 2.0, font_size, WHITE);
    draw_text("UP ARROW", start_x + 200.0, start_y + line_height * 2.0, font_size, LIGHTGRAY);

    draw_text("Decrease Speed:", start_x, start_y + line_height * 3.0, font_size, WHITE);
    draw_text("DOWN ARROW", start_x + 200.0, start_y + line_height * 3.0, font_size, LIGHTGRAY);
}


#[macroquad::main("4-Panel Submarine Bearing Graph")]
async fn main() {
    let mut sim = Simulation {
        current_sim_time_s: 0.0,
        time_step_s: 1.0,
        time_bearing_graph_history_s: 300.0,
    };

    let mut own_ship = OwnShip {
        id: 0,
        position_m: Point2::new(5000.0, 5000.0),
        speed_knots: 5.0,
        heading_deg: 45.0,
        recorded_bearings: vec![(1, VecDeque::new())],
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

    loop {
        clear_background(BLACK);

        let dt = get_frame_time();
        time_accumulator += dt;

        handle_input(&mut own_ship);

        if time_accumulator >= sim.time_step_s {
            time_accumulator -= sim.time_step_s;
            sim.current_sim_time_s += sim.time_step_s;

            update_position(&mut own_ship.position_m, own_ship.speed_knots, own_ship.heading_deg, sim.time_step_s);
            
            for target in &mut targets {
                update_position(&mut target.position_m, target.speed_knots, target.heading_deg, sim.time_step_s);
                target.track_history.push_back(target.position_m);
                if target.track_history.len() > 200 { target.track_history.pop_front(); }
            }

            for target in &targets {
                let true_bearing = calculate_true_bearing_deg(&own_ship.position_m, &target.position_m);
                let relative_bearing = normalize_degrees(true_bearing - own_ship.heading_deg);
                
                last_true_bearing = true_bearing;
                last_relative_bearing = relative_bearing;

                if let Some((_, history)) = own_ship.recorded_bearings.iter_mut().find(|(id, _)| *id == target.id) {
                    history.push_back(BearingRecord {
                        timestamp_s: sim.current_sim_time_s,
                        relative_bearing_deg: relative_bearing,
                    });
                    while let Some(front) = history.front() {
                        if sim.current_sim_time_s - front.timestamp_s > sim.time_bearing_graph_history_s {
                            history.pop_front();
                        } else { break; }
                    }
                }
            }
        }
        
        // --- Define the 4-panel layout ---
        let screen_w = screen_width();
        let screen_h = screen_height();
        let half_w = screen_w / 2.0;
        let half_h = screen_h / 2.0;

        let map_panel = Rect::new(0.0, 0.0, half_w, half_h);
        let graph_panel = Rect::new(half_w, 0.0, half_w, half_h);
        let info_panel = Rect::new(0.0, half_h, half_w, half_h);
        let controls_panel = Rect::new(half_w, half_h, half_w, half_h);
        
        // --- Draw the four panels ---
        draw_map_view(map_panel, &own_ship, &targets);
        draw_bearing_rate_graph(graph_panel, &own_ship, &sim);
        draw_info_panel(info_panel, &own_ship, last_true_bearing, last_relative_bearing);
        draw_controls_panel(controls_panel);

        next_frame().await
    }
}
