// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use std::cell::RefCell;
use std::rc::Rc;

use nalgebra::{Matrix3, Vector3};

slint::slint! {
    export { MainWindow } from "main.slint";
}

#[derive(Debug, Clone, Copy)]
struct CalibrationPoint {
    x: f32,
    y: f32,
}

#[derive(Debug, Clone)]
struct CalibrationData {
    target: CalibrationPoint,
    touch: Option<CalibrationPoint>,
}

struct Calibrator {
    points: Vec<CalibrationData>,
}

impl Calibrator {
    fn new(num_points: usize, width: f32, height: f32) -> Self {
        // Calculate margin based on aspect ratio to maintain consistent physical distance
        let margin_pixels = 40.0; // Target margin in pixels
        let margin_x = margin_pixels / width;
        let margin_y = margin_pixels / height;

        let target_positions = match num_points {
            3 => {
                // Equilateral triangle centered at (0.5, 0.5), rotated 15 degrees
                let (cx, cy, radius, rotation) = (0.5, 0.5, 0.35, 15.0_f32.to_radians());
                (0..3)
                    .map(|i| {
                        let angle = rotation + i as f32 * 2.0 * std::f32::consts::PI / 3.0;
                        (cx + radius * angle.cos(), cy + radius * angle.sin())
                    })
                    .collect()
            }
            4 => vec![
                (margin_x, margin_y),
                (1.0 - margin_x, margin_y),
                (1.0 - margin_x, 1.0 - margin_y),
                (margin_x, 1.0 - margin_y),
            ],
            _ => vec![
                (margin_x, margin_y),
                (1.0 - margin_x, margin_y),
                (1.0 - margin_x, 1.0 - margin_y),
                (margin_x, 1.0 - margin_y),
                (0.5, 0.5),
            ],
        };

        Self {
            points: target_positions
                .iter()
                .map(|&(x, y)| CalibrationData {
                    target: CalibrationPoint { x, y },
                    touch: None,
                })
                .collect(),
        }
    }

    fn get_target(&self, index: usize) -> Option<(f32, f32)> {
        self.points.get(index).map(|data| (data.target.x, data.target.y))
    }

    fn add_point(&mut self, x: f32, y: f32, index: usize) {
        if let Some(point) = self.points.get_mut(index) {
            point.touch = Some(CalibrationPoint { x, y });
        }
    }

    fn calculate_calibration(&self) -> Option<CalibrationMatrix> {
        let valid_points: Vec<_> = self
            .points
            .iter()
            .filter_map(|data| data.touch.map(|touch| (data.target, touch)))
            .collect();

        if valid_points.len() < 3 {
            return None;
        }

        // Least-squares: target = A * [touch_x, touch_y, 1]^T
        let (mut ata, mut atb_x, mut atb_y) = (Matrix3::zeros(), Vector3::zeros(), Vector3::zeros());

        for (target, touch) in &valid_points {
            let row = Vector3::new(touch.x, touch.y, 1.0);
            ata += row * row.transpose();
            atb_x += row * target.x;
            atb_y += row * target.y;
        }

        let lu = ata.lu();
        let params_x = lu.solve(&atb_x)?;
        let params_y = lu.solve(&atb_y)?;

        Some(CalibrationMatrix {
            a: params_x[0],
            b: params_x[1],
            c: params_x[2],
            d: params_y[0],
            e: params_y[1],
            f: params_y[2],
        })
    }

    fn restart(&mut self) {
        self.points.iter_mut().for_each(|p| p.touch = None);
    }
}

#[derive(Debug)]
struct CalibrationMatrix {
    a: f32,
    b: f32,
    c: f32,
    d: f32,
    e: f32,
    f: f32,
}

fn main() {
    let num_points = std::env::args()
        .nth(1)
        .and_then(|arg| arg.parse::<usize>().ok())
        .unwrap_or(5);

    let num_points = match num_points {
        3 | 4 | 5 => num_points,
        _ => {
            eprintln!("Invalid number of points. Using default (5).");
            eprintln!("Valid options: 3, 4, or 5");
            5
        }
    };

    let main_window = MainWindow::new().unwrap();

    // Get window dimensions for aspect-ratio aware target positioning
    let size = main_window.window().size();
    let width = size.width as f32;
    let height = size.height as f32;

    let calibrator = Rc::new(RefCell::new(Calibrator::new(num_points, width, height)));
    let current_point = Rc::new(RefCell::new(0_usize));

    // Setup point-clicked callback
    {
        let main_window_weak = main_window.as_weak();
        let calibrator = calibrator.clone();
        let current_point = current_point.clone();

        main_window.on_point_clicked(move |x, y| {
            let main_window = main_window_weak.unwrap();
            let mut current = current_point.borrow_mut();
            let total = calibrator.borrow().points.len();

            println!("Point {} clicked at ({:.3}, {:.3})", *current, x, y);

            calibrator.borrow_mut().add_point(x, y, *current);
            *current += 1;
            main_window.set_current_point(*current as i32);

            if *current >= total {
                main_window.set_calibration_complete(true);
                if let Some(matrix) = calibrator.borrow().calculate_calibration() {
                    let result = format!(
                        "{:.6} {:.6} {:.6}\n{:.6} {:.6} {:.6}",
                        matrix.a, matrix.b, matrix.c, matrix.d, matrix.e, matrix.f
                    );
                    println!("\nCalibration values: {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
                        matrix.a, matrix.b, matrix.c, matrix.d, matrix.e, matrix.f);
                    main_window.set_calibration_result(result.into());
                }
            } else if let Some((tx, ty)) = calibrator.borrow().get_target(*current) {
                main_window.set_target_x(tx);
                main_window.set_target_y(ty);
                main_window.set_progress_text(format!("Point {} / {}", *current + 1, total).into());
            }
        });
    }

    // Setup restart callback
    {
        let main_window_weak = main_window.as_weak();
        let calibrator = calibrator.clone();
        let current_point = current_point.clone();

        main_window.on_restart_calibration(move || {
            let main_window = main_window_weak.unwrap();
            let total = calibrator.borrow().points.len();

            calibrator.borrow_mut().restart();
            *current_point.borrow_mut() = 0;

            if let Some((tx, ty)) = calibrator.borrow().get_target(0) {
                main_window.set_target_x(tx);
                main_window.set_target_y(ty);
            }

            main_window.set_calibration_complete(false);
            main_window.set_calibration_result("".into());
            main_window.set_current_point(0);
            main_window.set_progress_text(format!("Point 1 / {}", total).into());
            println!("\nCalibration restarted");
        });
    }

    // Initialize UI
    let total = calibrator.borrow().points.len();
    main_window.set_total_points(total as i32);
    main_window.set_current_point(0);
    main_window.set_progress_text(format!("Point 1 / {}", total).into());
    if let Some((tx, ty)) = calibrator.borrow().get_target(0) {
        main_window.set_target_x(tx);
        main_window.set_target_y(ty);
    }

    println!("Touch Calibration Tool ({}-point calibration)", num_points);
    println!("======================");
    println!("Touch each target as it appears on the screen.");
    println!();

    main_window.run().unwrap();
}
