use macroquad::prelude::*;

#[macroquad::main("dn")]
async fn main() {
    loop {
        // Clear the background to a dark blue, like the ocean!
        clear_background(Color::new(0.0, 0.1, 0.2, 1.0)); // RGBA (dark blue)

        // Draw a simple white circle in the center of the screen
        draw_circle(screen_width() / 2.0, screen_height() / 2.0, 50.0, WHITE);

        // Advance to the next frame
        next_frame().await
    }
}
