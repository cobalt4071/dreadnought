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

enum AppView {
    Map,
    BearingGraph,
    Info,
    Controls,
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

/// Panel 1: 2D Map View
fn draw_map_view(panel: Rect, own_ship: &OwnShip, targets: &Vec<Target>) {
    let world_view_size = 20000.0; // Visible area in meters (20km)

    set_camera(&Camera2D {
        zoom: vec2(1.0 / world_view_size, -1.0 / world_view_size),
        target: vec2(own_ship.position_m.x, own_ship.position_m.y),
        ..Default::default()
    });

    // Draw background
    draw_rectangle(
        own_ship.position_m.x - world_view_size / 2.0,
        own_ship.position_m.y - world_view_size / 2.0,
        world_view_size,
        world_view_size,
        Color::new(0.0, 0.1, 0.1, 1.0),
    );

    // Draw a grid
    let grid_spacing = 1000.0; // 1km
    let grid_lines = (world_view_size / grid_spacing) as i32;
    let grid_color = Color::new(0.0, 0.2, 0.2, 1.0);
    let center_x = (own_ship.position_m.x / grid_spacing).floor() as i32;
    let center_y = (own_ship.position_m.y / grid_spacing).floor() as i32;

    for i in (center_x - grid_lines)..(center_x + grid_lines) {
        let x = i as f32 * grid_spacing;
        draw_line(x, own_ship.position_m.y - world_view_size, x, own_ship.position_m.y + world_view_size, 20.0, grid_color);
    }
    for i in (center_y - grid_lines)..(center_y + grid_lines) {
        let y = i as f32 * grid_spacing;
        draw_line(own_ship.position_m.x - world_view_size, y, own_ship.position_m.x + world_view_size, y, 20.0, grid_color);
    }

    // Draw Own Ship
    let ship_size = 500.0; // 500 meters long
    let heading_rad = own_ship.heading_deg.to_radians();
    let rot = nalgebra::Rotation2::new(heading_rad);

    let v1 = rot * nalgebra::Vector2::new(0.0, -ship_size);
    let v2 = rot * nalgebra::Vector2::new(-ship_size/2.0, ship_size/2.0);
    let v3 = rot * nalgebra::Vector2::new(ship_size/2.0, ship_size/2.0);

    let p1 = own_ship.position_m + v1;
    let p2 = own_ship.position_m + v2;
    let p3 = own_ship.position_m + v3;

    draw_triangle(
        vec2(p1.x, p1.y),
        vec2(p2.x, p2.y),
        vec2(p3.x, p3.y),
        GREEN,
    );

    // Draw Targets
    for target in targets {
        if target.track_history.len() > 1 {
            for i in 0..target.track_history.len() - 1 {
                draw_line(
                    target.track_history[i].x, target.track_history[i].y,
                    target.track_history[i+1].x, target.track_history[i+1].y,
                    40.0, RED
                );
            }
        }
        draw_circle(target.position_m.x, target.position_m.y, 250.0, RED); // 250m radius
    }

    set_default_camera();
    // Draw the UI text over the map
    draw_text("View: Map (1/4) - Use keys 1-4 to switch", 10.0, 30.0, 30.0, WHITE);
}


/// Panel 2: Bearing Rate Graph
fn draw_bearing_rate_graph(panel: Rect, own_ship: &OwnShip, sim: &Simulation) {
    draw_rectangle(panel.x, panel.y, panel.w, panel.h, Color::new(0.0, 0.0, 0.1, 1.0));
    draw_rectangle_lines(panel.x, panel.y, panel.w, panel.h, 3.0, WHITE);
    draw_text("View: Bearing Rate Graph (2/4) - Use keys 1-4 to switch", panel.x + 10.0, panel.y + 30.0, 30.0, WHITE);


    let font_size = 20.0;
    let padding = 5.0;

    draw_text("360", panel.x + padding, panel.y + font_size, font_size, LIGHTGRAY);
    draw_text("180", panel.x + padding, panel.y + panel.h / 2.0, font_size, LIGHTGRAY);
    draw_text("000", panel.x + padding, panel.y + panel.h - padding, font_size, LIGHTGRAY);

    draw_text("Now", panel.x + panel.w - 40.0, panel.y + panel.h - padding, font_size, LIGHTGRAY);
    let time_label = format!("-{}s", sim.time_bearing_graph_history_s as i32);
    draw_text(&time_label, panel.x, panel.y + panel.h - padding, font_size, LIGHTGRAY);
    
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


/// Panel 3: Numerical Info
fn draw_info_panel(panel: Rect, own_ship: &OwnShip, true_bearing: f32, relative_bearing: f32) {
    draw_rectangle(panel.x, panel.y, panel.w, panel.h, Color::new(0.1, 0.1, 0.0, 1.0));
    draw_rectangle_lines(panel.x, panel.y, panel.w, panel.h, 3.0, WHITE);
    draw_text("View: Telemetry (3/4) - Use keys 1-4 to switch", panel.x + 10.0, panel.y + 30.0, 30.0, WHITE);


    let font_size = 24.0;
    let start_x = panel.x + 20.0;
    let start_y = panel.y + 80.0;
    let line_height = 30.0;

    draw_text("TELEMETRY", start_x, panel.y + 45.0, 22.0, YELLOW);
    
    draw_text(&format!("Own Ship Heading: {:.1}°", own_ship.heading_deg), start_x, start_y, font_size, WHITE);
    draw_text(&format!("Own Ship Speed:   {:.1} kts", own_ship.speed_knots), start_x, start_y + line_height, font_size, WHITE);
    draw_text(&format!("Target True Bearing: {:.1}°", true_bearing), start_x, start_y + line_height * 2.5, font_size, WHITE);
    draw_text(&format!("Target Rel. Bearing: {:.1}°", relative_bearing), start_x, start_y + line_height * 3.5, font_size, GREEN);
}

/// Panel 4: Controls Help
fn draw_controls_panel(panel: Rect) {
    draw_rectangle(panel.x, panel.y, panel.w, panel.h, Color::new(0.1, 0.0, 0.1, 1.0));
    draw_rectangle_lines(panel.x, panel.y, panel.w, panel.h, 3.0, WHITE);
    draw_text("View: Controls (4/4) - Use keys 1-4 to switch", panel.x + 10.0, panel.y + 30.0, 30.0, WHITE);


    let font_size = 24.0;
    let start_x = panel.x + 20.0;
    let start_y = panel.y + 80.0;
    let line_height = 30.0;

    draw_text("CONTROLS", start_x, panel.y + 45.0, 22.0, YELLOW);

    draw_text("Turn Left:", start_x, start_y, font_size, WHITE);
    draw_text("LEFT ARROW", start_x + 200.0, start_y, font_size, LIGHTGRAY);

    draw_text("Turn Right:", start_x, start_y + line_height, font_size, WHITE);
    draw_text("RIGHT ARROW", start_x + 200.0, start_y + line_height, font_size, LIGHTGRAY);

    draw_text("Increase Speed:", start_x, start_y + line_height * 2.0, font_size, WHITE);
    draw_text("UP ARROW", start_x + 200.0, start_y + line_height * 2.0, font_size, LIGHTGRAY);

    draw_text("Decrease Speed:", start_x, start_y + line_height * 3.0, font_size, WHITE);
    draw_text("DOWN ARROW", start_x + 200.0, start_y + line_height * 3.0, font_size, LIGHTGRAY);
}


#[macroquad::main("Submarine Bearing Analyser")]
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
    
    let mut current_view = AppView::Map;

    loop {
        clear_background(BLACK);

        let dt = get_frame_time();
        time_accumulator += dt;

        handle_input(&mut own_ship);

        if is_key_pressed(KeyCode::Key1) { current_view = AppView::Map; }
        if is_key_pressed(KeyCode::Key2) { current_view = AppView::BearingGraph; }
        if is_key_pressed(KeyCode::Key3) { current_view = AppView::Info; }
        if is_key_pressed(KeyCode::Key4) { current_view = AppView::Controls; }

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
        
        let screen_rect = Rect::new(0.0, 0.0, screen_width(), screen_height());

        match current_view {
            AppView::Map => draw_map_view(screen_rect, &own_ship, &targets),
            AppView::BearingGraph => draw_bearing_rate_graph(screen_rect, &own_ship, &sim),
            AppView::Info => draw_info_panel(screen_rect, &own_ship, last_true_bearing, last_relative_bearing),
            AppView::Controls => draw_controls_panel(screen_rect),
        }

        next_frame().await
    }
}